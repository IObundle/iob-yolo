//import custom libraries
#include "system.h"
#include "iob-uart.h"
#include "iob-eth.h"
#include "iob_timer.h"
#include "firmware.h"
#include "iob-cache.h"
#include "versat.hpp"

//import c libraries
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//define peripheral base addresses
#define UART (UART_BASE<<(DATA_W-N_SLAVES_W))
#define ETHERNET (ETHERNET_BASE<<(ADDR_W-N_SLAVES_W))
#define TIMER (TIMER_BASE<<(ADDR_W-N_SLAVES_W))
#define DDR_MEM (CACHE_BASE<<(ADDR_W-N_SLAVES_W))
#define VERSAT (VERSAT_BASE<<(ADDR_W-N_SLAVES_W))

//Constants for image resize
#define w_scale ((float)(IMG_W-1)/(NEW_W-1))
#define h_scale ((float)(IMG_H-1)/(NEW_H-1))

//define general constants
#define ETH_NBYTES (1024-18) //minimum ethernet payload excluding FCS
#define INPUT_FILE_SIZE IMAGE_INPUT //8 bits per point
#define NUM_INPUT_FRAMES (INPUT_FILE_SIZE/ETH_NBYTES)
#define OUTPUT_FILE_SIZE (NETWORK_INPUT*2) //16 bits per output
#define NUM_OUTPUT_FRAMES (OUTPUT_FILE_SIZE/ETH_NBYTES)
#define ix_size (NEW_W*4)
#define iy_size (NEW_H)
#define dx_size (NEW_W*2)
#define dy_size (NEW_H*2)

//define DDR mapping
#define ix_BASE_ADDRESS (DDR_MEM + (int)pow(2,MAINRAM_ADDR_W)) //after main mem
#define iy_BASE_ADDRESS (ix_BASE_ADDRESS + ix_size*2) //16 bits
#define dx_BASE_ADDRESS (iy_BASE_ADDRESS + iy_size*2) //16 bits
#define dy_BASE_ADDRESS (dx_BASE_ADDRESS + dx_size*2) //16 bits
#define DATA_BASE_ADDRESS (dy_BASE_ADDRESS + dy_size*2) //16 bits

//ETHERNET variables
int rcv_timeout = 5000;
char data_to_send[ETH_NBYTES];
char data_rcv[ETH_NBYTES+18];
unsigned int bytes_to_receive, bytes_to_send, count_bytes;

//TIMER variables
unsigned int start, end;

//data base address pointers
int16_t * fp_data;
int16_t * fp_image;
int16_t * ix, * iy, * dx, * dy;

//define base address of data pointers
void define_memory_regions() {

  //image
  fp_image = (int16_t *) DATA_BASE_ADDRESS;

  //data
  fp_data = (int16_t *) (DATA_BASE_ADDRESS + 2*IMAGE_INPUT);

  //resize
  ix = (int16_t *) ix_BASE_ADDRESS;
  iy = (int16_t *) iy_BASE_ADDRESS;
  dx = (int16_t *) dx_BASE_ADDRESS;
  dy = (int16_t *) dy_BASE_ADDRESS;
}

//reset certain DDR positions to zero due to padding
void reset_DDR() {
  
  //local variables
  unsigned int i;

  //measure initial time
  uart_printf("\nSetting DDR positions to zero\n");
  start = timer_get_count_us(TIMER);

  //resized image
  for(i = 0; i < IMAGE_INPUT; i++) fp_image[i] = 0;

  //input network
  for(i = 0; i < NETWORK_INPUT; i++) fp_data[i] = 0;

  //measure final time
  end = timer_get_count_us(TIMER);
  uart_printf("DDR reset to zero done in %d ms\n", (end-start)/1000);
}

void rcv_frame(unsigned int NUM_DATA_FRAMES, unsigned int DATA_SIZE, char * data_p) {

  //Local variables
  int i, j;
  char * fp_data_char = (char *) data_p;
  count_bytes = 0;

  //Loop to receive intermediate data frames
  for(j = 0; j < NUM_DATA_FRAMES+1; j++) {

     //wait to receive frame
     while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout) !=0);

     // start timer
     if(j == 0) start = timer_get_count_us(TIMER);   

     //check if it is last packet (has less data that full payload size)
     if(j == NUM_DATA_FRAMES) bytes_to_receive = DATA_SIZE - count_bytes;
     else bytes_to_receive = ETH_NBYTES;

     //save in DDR
     for(i = 0; i < bytes_to_receive; i++) {
       fp_data_char[j*ETH_NBYTES*2 + i*2] = data_rcv[14+i];
       data_to_send[i] = data_rcv[14+i];
     }

     //send data back as ack
     eth_send_frame(data_to_send, ETH_NBYTES);

     //update byte counter
     count_bytes += ETH_NBYTES;
  }
}

//receive input image
void receive_data() {

  //Receive input image
  uart_printf("\nReady to receive input image...\n");
  char * fp_image_char = (char *) DATA_BASE_ADDRESS;
  rcv_frame(NUM_INPUT_FRAMES, INPUT_FILE_SIZE, fp_image_char);
  end = timer_get_count_us(TIMER);
  uart_printf("Image received in %d ms\n", (end-start)/1000);
}

//fill resized image region with grey (0.5 = 0x0080 in Q8.8)
void fill_grey() {
  int i, j, k;
  for(i = 0; i < NTW_IN_C; i++)
    for(j = 0; j < NTW_IN_H; j++)
      for(k = 0; k < NTW_IN_W; k++)
	fp_data[i*(NTW_IN_H+2)*(NTW_IN_W+2) + (j+1)*(NTW_IN_W+2) + (k+1)] = 0x0080;
}

//initializes ix, iy, dx and dy variables
void prepare_resize() {

  //local variables
  int i, val_i;
  float val, val_d;

  //loop to initialize ix and dx
  for(i = 0; i < NEW_W; i++) {
    val = i*w_scale;
    val_i = (int) val;
    val_d = val - val_i;
    //for iy  
    ix[2*i] = val_i;
    ix[2*i+1] = val_i + 1;
    //for iy+1
    ix[2*i+2*NEW_W] = val_i + IMG_W;
    ix[2*i+1+2*NEW_W] = val_i + 1 + IMG_W;
    //dx
    dx[2*i] = (int16_t)((1-val_d)*((int16_t)1<<14)); //Q2.14
    dx[2*i+1] = (int16_t)(val_d*((int16_t)1<<14)); //Q2.14
  }

  //loop to initialize iy and dy
  for(i = 0; i < NEW_H; i++) {
    val = i*h_scale;
    iy[i] = (int) val;
    val_d = val - iy[i];
    //dy
    dy[2*i] = (int16_t)((1-val_d)*((int16_t)1<<14)); //Q2.14
    dy[2*i+1] = (int16_t)(val_d*((int16_t)1<<14)); //Q2.14
  }
}

//resize input image
void resize_image() {

  //local variables
  int i, j, k;
#ifdef SIM
  int num_err = 0;
#endif

  //load ix and dx to versat
  for(i = 0; i < ix_size; i++) stage[0].memA[0].write(i, ix[i]);
  for(i = 0; i < dx_size; i++) stage[0].memA[1].write(i, dx[i]);
  stage[1].memA[0].write(0, 0);
  stage[1].memA[0].write(2, 0);

  //configure mem0 (stage 0) to read: ix, ix+1, ix+2*NEW_W, ix+2*NEW_W+1 sequence
  //start, iter, incr, delay, per, duty, sel, shift, in_wr
  stage[0].memA[0].setConf(0, 2, 1, 0, 2, 2, 0, 2*NEW_W-2, 0);
  //iter2, per2, shift2, incr2
  stage[0].memA[0].setConf(1, NEW_W, 0, 2);
  stage[0].memA[0].writeConf();

  //configure mem1(stage 0) to read 1-dx, dx sequence twice
  stage[0].memA[1].setConf(0, 2, 1, MEMP_LAT, 2, 2, 0, -2, 0);
  //iter2, per2, shift2, incr2
  stage[0].memA[1].setConf(1, NEW_W, 0, 2);
  stage[0].memA[1].writeConf();

  //configure mem2 (stage 0) to read input pixels (addressed by mem0)
  //start, iter, incr, delay, per, duty, sel, shift, in_wr, rvrs, ext
  stage[0].memA[2].setConf(0, 1, 0, MEMP_LAT, 4*NEW_W, 4*NEW_W, sMEMA[0], 0, 0, 0, 1);
  stage[0].memA[2].writeConf();

  //pixel * 1-dx/dx = res0
  //sela, selb, fns, iter, per, delay, shift
  stage[0].muladd[0].setConf(sMEMA[1], sMEMA[2], MULADD_MACC, 2*NEW_W, 2, 2*MEMP_LAT, 7); //Q10.22 to Q1.15
  stage[0].muladd[0].writeConf();

  //configure mem0 (stage 1) to read 0, 1-dy, 0, dy sequence
  stage[1].memA[0].setConf(0, NEW_W, 1, MEMP_LAT+MULADD_LAT, 4, 4, 0, -4, 0);
  stage[1].memA[0].writeConf();

  //res0 * 0/1-dy/0/dy = res1
  stage[1].muladd[0].setConf(sMEMA[0], sMULADD_p[0], MULADD_MACC, NEW_W, 4, 2*MEMP_LAT+MULADD_LAT, 21); //Q3.29 to Q8.8
  stage[1].muladd[0].writeConf();

  //store res1 in mem1 (stage 1)
  stage[1].memA[1].setConf(0, NEW_W, 1, 2*MEMP_LAT+2*MULADD_LAT+(4-1), 4, 1, sMULADD[0], 0, 1);
  stage[1].memA[1].writeConf();

  //loops for performing resizing
#ifdef SIM
  for(k = 0; k < 1; k++) {
    for(j = 0; j < 1; j++) {
#else
  for(k = 0; k < NTW_IN_C; k++) {
    for(j = 0; j < NEW_H; j++) {
#endif

      //Store 2 lines of pixels in mem2 (stage 0)
      //Extra pixel is necessary to be multiplied by zero
      for(i = 0; i < IMG_W*2+1; i++) stage[0].memA[2].write(i, fp_image[k*IMG_W*IMG_H + iy[j]*IMG_W + i]);

      //store dy in mem0 (stage 1)
      stage[1].memA[0].write(1, dy[2*j]);
      stage[1].memA[0].write(3, dy[2*j+1]);

      //Wait until done
      run();
      while(done() == 0);

      //store result in DDR
      for(i = 0; i < NEW_W; i++)
      #ifdef SIM
        if(fp_data[k*(NTW_IN_W+2)*(NTW_IN_H+2) + (j+1)*(NTW_IN_W+2) + (i+1) + EXTRA_W + ((NTW_IN_W+2)*EXTRA_H)] != stage[1].memA[1].read(i)) num_err++;
      #else
        fp_data[k*(NTW_IN_W+2)*(NTW_IN_H+2) + (j+1)*(NTW_IN_W+2) + (i+1) + EXTRA_W + ((NTW_IN_W+2)*EXTRA_H)] = stage[1].memA[1].read(i);
      #endif

    }
  }
#ifdef SIM
  uart_printf("Resizing done with %d errors\n", num_err);
#endif
}

//send detection results back
void send_data() {

  //Loop to send output of yolo layer
  int i, j;
  count_bytes = 0;
  char * fp_data_char = (char *) fp_data;
  for(j = 0; j < NUM_OUTPUT_FRAMES+1; j++) {

     // start timer
     if(j == 0) start = timer_get_count_us(TIMER);

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
  end = timer_get_count_us(TIMER);
  uart_printf("\noutput layer transferred in %d ms\n\n", (end-start)/1000);
}

int main(int argc, char **argv) {

  //init UART
  uart_init(UART,UART_CLK_FREQ/UART_BAUD_RATE);

  //send init message
  uart_printf("\nYOLO HW FULL\n\n");
  uart_txwait();

#ifndef SIM
  //init ETHERNET
  eth_init(ETHERNET);
  eth_set_rx_payload_size(ETH_NBYTES);
#endif

  //init VERSAT
  versat_init(VERSAT);

  //define memory regions
  define_memory_regions();
  unsigned int total_time;

  //load data and reset DDR to zero
#ifndef SIM
  reset_DDR();
  receive_data();
  fill_grey();
#endif

  //initialize ix, iy, dx and dy arrays
  prepare_resize();

  //resize input image
  uart_printf("\nResizing input image...\n");
  start = timer_get_count_us(TIMER);
  resize_image();
  end = timer_get_count_us(TIMER);
  uart_printf("Resize image done in %d ms\n", (end-start)/1000);
  total_time = (end-start)/1000;

  //return data
  uart_printf("\ntotal_time = %d seconds (%d minutes) \n", total_time/1000, (total_time/1000)/60);
#ifndef SIM
  send_data();
#endif
  uart_putc(4);
  return 0;
}
