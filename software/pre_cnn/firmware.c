//import custom libraries
#include "system.h" //must be defined before versat API!!!
#include "periphs.h"
#include "iob-uart.h"
#include "iob-eth.h"
#include "iob_timer.h"
#include "new_versat.hpp"

//import c libraries
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//input image constants
#define IMG_W 768
#define IMG_H 576
#define IMG_C 4
#define NEW_W 416
#define NEW_H ((IMG_H*NEW_W)/IMG_W)	//312
#define IMAGE_INPUT (IMG_W*IMG_H*IMG_C) //already 32-byte aligned
#define NETWORK_INPUT_AUX (NEW_W*IMG_H*IMG_C)

//resize constants
#define h_scale ((float)(IMG_H-1)/(NEW_H-1))
#define ix_size (NEW_W*2)
#define dx_size (NEW_W*2)
#define dy_size (NEW_H*2)
int16_t iy[NEW_H];

//define peripheral base addresses
#ifndef PCSIM
  //USE_DDR==1 to run yolo anyways
  #if (RUN_DDR==0) // running firmware from SRAM 
    #define DDR_MEM (EXTRA_BASE)
  #else //running firmware from DDR
    #define DDR_MEM ((1<<(FIRM_ADDR_W)))
  #endif
#else
  uint8_t ddr_mem_vect[1<<(DDR_ADDR_W-2)]; //DDR addressable space, up to 2**30
  #define DDR_MEM (&ddr_mem_vect[0])
#endif //ifndef PCSIM

//define ethernet constants
#define ETH_NBYTES (1024-18) //minimum ethernet payload excluding FCS
#define INPUT_FILE_SIZE ((ix_size + dx_size + dy_size + IMAGE_INPUT)*2) //16 bits
#define NUM_INPUT_FRAMES (INPUT_FILE_SIZE/ETH_NBYTES)
#define OUTPUT_FILE_SIZE (NETWORK_INPUT_AUX*2)
#define NUM_OUTPUT_FRAMES (OUTPUT_FILE_SIZE/ETH_NBYTES)

//define DDR mapping
#define ix_BASE_ADDRESS (DDR_MEM + (1 << (FIRM_ADDR_W))) //after main mem
#define dx_BASE_ADDRESS (ix_BASE_ADDRESS + 2*ix_size)
#define dy_BASE_ADDRESS (dx_BASE_ADDRESS + 2*dx_size)
#define INPUT_IMAGE_BASE_ADDRESS (dy_BASE_ADDRESS + 2*dy_size)
#define DATA_BASE_ADDRESS (INPUT_IMAGE_BASE_ADDRESS + 2*IMAGE_INPUT)

//ETHERNET variables
int rcv_timeout = 5000;
char data_to_send[ETH_NBYTES];
char data_rcv[ETH_NBYTES+18];
unsigned int bytes_to_receive, bytes_to_send, count_bytes;

//TIMER variables
unsigned int start, end;

//initialize iy
void prepare_resize() {

  //local variables
  int i;
  float val;

  //loop to initialize iy
  for(i = 0; i < NEW_H; i++) {
  #ifdef SIM
    uart_printf("%d\n", i);
  #endif
    val = i*h_scale;
    iy[i] = (int) val;
  }
}

//receive weigths and resized padded image
void rcv_data() {

  //Local variables
  int i, j;
  count_bytes = 0;
  char * data_p = (char *) ix_BASE_ADDRESS;
  uart_printf("\nReady to receive data...\n");

  //Loop to receive intermediate data frames
  for(j = 0; j < NUM_INPUT_FRAMES+1; j++) {

     //wait to receive frame
     while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout) !=0);

     //start timer
     if(j == 0) start = timer_time_us(TIMER_BASE);

     //check if it is last packet (has less data that full payload size)
     if(j == NUM_INPUT_FRAMES) bytes_to_receive = INPUT_FILE_SIZE - count_bytes;
     else bytes_to_receive = ETH_NBYTES;

     //save in DDR
     for(i = 0; i < bytes_to_receive; i++) {
       data_p[j*ETH_NBYTES + i] = data_rcv[14+i];
       data_to_send[i] = data_p[j*ETH_NBYTES + i];
     }

     //send data back as ack
     eth_send_frame(data_to_send, ETH_NBYTES);

     //update byte counter
     count_bytes += ETH_NBYTES;
  }
  end = timer_time_us(TIMER_BASE);
  uart_printf("Image and weights received in %d ms\n", (end-start)/1000);
}

//width resize -> 768 to 416
void width_resize() {

  //local variables
  int16_t r, c, k;
  int32_t mul;
  int16_t * ix = (int16_t *) ix_BASE_ADDRESS;
  int16_t * dx = (int16_t *) dx_BASE_ADDRESS;
  int16_t * im_in = (int16_t *) INPUT_IMAGE_BASE_ADDRESS;
  int16_t * im_out = (int16_t *) DATA_BASE_ADDRESS;

  //perform width reduction
#ifdef SIM
  for(r = 0; r < 2; r++) {
    uart_printf("%d\n", r);
#else
  for(r = 0; r < IMG_H; r++) {
#endif
    for(c = 0; c < NEW_W; c++) {
      for(k = 0; k < IMG_C; k++) {
	mul = (int32_t)((int32_t)dx[2*c]*(int32_t)(im_in[r*IMG_W*IMG_C + ix[2*c]*IMG_C + k])); //Q2.14 * Q8.8 = Q10.22
        mul += (int32_t)((int32_t)dx[2*c+1]*(int32_t)(im_in[r*IMG_W*IMG_C + ix[2*c+1]*IMG_C + k])); //Q10.22
	im_out[r*NEW_W*IMG_C + c*IMG_C + k] = (int16_t) (mul >> 7); //Q10.22 to Q1.15
      }
    }
  }
}

//send results back
void send_data() {

  //Loop to send data
  int i, j;
  count_bytes = 0;
  char * fp_data_char = (char *) DATA_BASE_ADDRESS;
  for(j = 0; j < NUM_OUTPUT_FRAMES+1; j++) {

    //start timer
    if(j == 0) start = timer_time_us(TIMER_BASE);

     //check if it is last packet (has less data that full payload size)
     if(j == NUM_OUTPUT_FRAMES) bytes_to_send = OUTPUT_FILE_SIZE - count_bytes;
     else bytes_to_send = ETH_NBYTES;

     //prepare variable to be sent
     for(i = 0; i < bytes_to_send; i++) data_to_send[i] = fp_data_char[j*ETH_NBYTES + i];

     //send frame
     eth_send_frame(data_to_send, ETH_NBYTES);

     //wait to receive frame as ack
     if(j != NUM_OUTPUT_FRAMES) while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout) !=0);

     //update byte counter
     count_bytes += ETH_NBYTES;
  }

  //measure transference time
  end = timer_time_us(TIMER_BASE);
  uart_printf("\n\nOutput layer transferred in %d ms\n\n", (end-start)/1000);
}

int main(int argc, char **argv) {

  //init UART
  uart_init(UART_BASE,FREQ/BAUD);

  //send init message
  uart_printf("\nPRE CNN\n\n");

  //initialize iy
  uart_printf("\nPreparing resize...\n");
  start = timer_time_us(TIMER_BASE);
  //prepare_resize();
  end = timer_time_us(TIMER_BASE);
  uart_printf("Done in %d us\n\n", end-start);

#ifndef SIM

  //init ETHERNET
  eth_init(ETHERNET_BASE);
  eth_set_rx_payload_size(ETH_NBYTES);

  //receive data via ethernet
  rcv_data();
#endif

  //width resize
  uart_printf("\nWidth resizing...\n");
  start = timer_time_us(TIMER_BASE);
  width_resize();
  end = timer_time_us(TIMER_BASE);
  uart_printf("Done in %d us\n\n", end-start);

#ifdef SIM
  //verify results
  int16_t * fp_data = (int16_t *) DATA_BASE_ADDRESS;
  int i, j, k;
  uart_printf("\nVerifying...\n");
  for(i = 0; i < 2; i++)
    for(j = 0; j < NEW_W; j++)
      for(k = 0; k < IMG_C; k++)
        if(fp_data[i*NEW_W*IMG_C + j*IMG_C + k] != fp_data[NETWORK_INPUT_AUX + i*NEW_W*IMG_C + j*IMG_C + k])
	  uart_printf("(%d) res = %x, act = %x\n", i*NEW_W*IMG_C + j*IMG_C + k, fp_data[i*NEW_W*IMG_C + j*IMG_C + k] & 0xFFFF, fp_data[NETWORK_INPUT_AUX + i*NEW_W*IMG_C + j*IMG_C + k] & 0xFFFF);   
#endif

#ifndef SIM
  send_data();
#endif

  //finish
  uart_putc(ETX);
  uart_txwait();
  return 0;
}
