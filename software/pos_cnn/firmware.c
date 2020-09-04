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
#define IMG_C 3
#define NEW_W 416
#define NEW_H ((IMG_H*NEW_W)/IMG_W)	//312

//yolo constants
#define LAYER_16_W 13
#define LAYER_23_W 26

//constants for bounding boxes
#define threshold ((int16_t)(((float)0.5)*((int32_t)1<<8))) //Q8.8
#define yolo1_div ((int16_t)(((float)1/LAYER_16_W)*((int32_t)1<<15))) //Q1.15
#define yolo2_div ((int16_t)(((float)1/LAYER_23_W)*((int32_t)1<<15))) //Q1.15
#define y_scales ((int16_t)(((float)NEW_W/NEW_H)*((int32_t)1<<14))) //Q2.14
#define y_bias ((int16_t)(((float)(NEW_W-NEW_H)/(NEW_H*2))*((int32_t)1<<14))) //Q2.14
#define w_scales ((int16_t)(((float)1/NEW_W)*((int32_t)1<<14))) //Q2.14
#define h_scales ((int16_t)(((float)1/NEW_H)*((int32_t)1<<14))) //Q2.14
#define c3 ((int16_t)0x0AAA) // pow(2,-3)+pow(2,-5)+pow(2,-7)+pow(2,-9)+pow(2,-11)+pow(2,-13) in Q2.14
#define c4 ((int16_t)0x02C0) // pow(2,-5)+pow(2,-7)+pow(2,-8) in Q2.14

//variables for bounding boxes
int16_t boxes[84*10]; //max 10 boxes
uint8_t nboxes = 0;

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
#define INPUT_FILE_SIZE ((13*13*256 + 26*26*256)*2) //16 bits
#define NUM_INPUT_FRAMES (INPUT_FILE_SIZE/ETH_NBYTES)
#define OUTPUT_FILE_SIZE (84*nboxes*2) //16 bits
#define NUM_OUTPUT_FRAMES (OUTPUT_FILE_SIZE/ETH_NBYTES)

//define DDR mapping
#define DATA_BASE_ADDRESS (DDR_MEM + (1 << (FIRM_ADDR_W))) //after main mem

//ETHERNET variables
int rcv_timeout = 5000;
char data_to_send[ETH_NBYTES];
char data_rcv[ETH_NBYTES+18];
unsigned int bytes_to_receive, bytes_to_send, count_bytes;

//TIMER variables
unsigned int start, end;

//receive weigths and resized padded image
void rcv_data() {

  //Local variables
  int i, j;
  count_bytes = 0;
  char * data_p = (char *) DATA_BASE_ADDRESS;
  uart_printf("\nReady to receive yolo layers output...\n");

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

// polynomial approximation of exponential function
int16_t exp_fnc(int16_t val) {
  int16_t val_16, exp_val_fixed;
  int32_t val_32;
  exp_val_fixed = val + 0x0100; //1+w -> Q8.8
  exp_val_fixed = exp_val_fixed << 6; //Q8.8 to Q2.14
  val_32 = (int32_t)((int32_t)val*(int32_t)val); //w^2 -> Q8.8*Q8.8 = Q16.16
  val_16 = (int16_t)(val_32 >> 2); //w^2 -> Q16.16 to Q2.14
  val_32 = (int32_t)((int32_t)0x2000*(int32_t)val_16); //0.5*w^2 -> Q2.14*Q2.14 = Q4.28
  exp_val_fixed += (int16_t)(val_32 >> 14); //1+w+0.5*w^2 -> Q4.28 to Q2.14
  val_32 = (int32_t)((int32_t)val_16*(int32_t)val); //w^3 -> Q2.14*Q8.8 = Q10.22
  val_16 = (int16_t)(val_32 >> 8); //w^3 -> Q10.22 to Q2.14
  val_32 = (int32_t)((int32_t)c3*(int32_t)val_16); //c3*w^3 -> Q2.14*Q2.14 = Q4.28
  exp_val_fixed += (int16_t)(val_32 >> 14); //1+w+0.5*w^2+c3*w^3 -> Q4.28 to Q2.14
  val_32 = (int32_t)((int32_t)val_16*(int32_t)val); //w^4 -> Q2.14*Q8.8 = Q10.22
  val_16 = (int16_t)(val_32 >> 8); //w^4 -> Q10.22 to Q2.14
  val_32 = (int32_t)((int32_t)c4*(int32_t)val_16); //c4*w^4 -> Q2.14*Q2.14 = Q4.28
  exp_val_fixed += (int16_t)(val_32 >> 14); //1+w+0.5*w^2+c3*w^3+c4*w^4 -> Q4.28 to Q2.14
  return exp_val_fixed; //Q2.14
}

// create boxes
void create_boxes(int w, unsigned int pos, int16_t xy_div, int first_yolo) {

  //local variable
  int i, j, k, m, n, n_start, n_end;
  int16_t val_16, obj_score;
  int16_t * fp_data = (int16_t *) DATA_BASE_ADDRESS + pos;
  int32_t val_32;
  int16_t yolo_bias[12] = {0x0280, 0x0380, 0x05C0, 0x06C0, 0x0940, 0x0E80, 0x1440, 0x1480, 0x21C0, 0x2A40, 0x5600, 0x4FC0}; //Q10.6

  //print detections
  for(i = 0; i < w; i++) {
    for(j = 0; j < w; j++) {

      //check if obj score is higher than threshold
      for(k = 0; k < 3; k++) {
	obj_score = fp_data[i*w*256 + j*16 + 5*k*w*16 + 5*k + 4];
        if(obj_score > threshold) {

          //Calculate x
          val_32 = (int32_t)((int32_t)(fp_data[i*w*256 + j*16 + 5*k*w*16 + 5*k] + (j<<8))*(int32_t)xy_div);
 	  boxes[84*nboxes] = (int16_t)(val_32 >> 9); //Q9.23 to Q2.14
          
	  //Calculate y
	  val_32 = (int32_t)((int32_t)(fp_data[i*w*256 + j*16 + 5*k*w*16 + 5*k + 1] + (i<<8))*(int32_t)xy_div); //Q8.8 *Q1.15 = Q9.23
	  val_16 = (int16_t)(val_32 >> 9); //Q9.23 to Q2.14
          val_32 = (int32_t)((int32_t)val_16*(int32_t)y_scales); //Q2.14 * Q2.14 = Q4.28
  	  val_16 = (int16_t)(val_32 >> 14); //Q4.28 to Q2.14
	  val_16 -= (int16_t)y_bias; //Q2.14
	  boxes[84*nboxes+1] = val_16;

	  //Calculate w
	  val_32 = (int32_t)((int32_t)exp_fnc(fp_data[i*w*256 + j*16 + 5*k*w*16 + 5*k + 2])*(int32_t)w_scales); //Q2.14 * Q2.14 = Q4.28
	  val_16 = (int16_t)(val_32 >> 14); //Q4.28 to Q2.14
  	  val_32 = (int32_t)((int32_t)val_16*(int32_t)yolo_bias[2*(k+(1-first_yolo)+3*first_yolo)]); //Q2.14 * Q10.6 = Q12.20
	  boxes[84*nboxes+2] = (int16_t)(val_32 >> 6); //Q12.20 to Q2.14

	  //Calculate h
	  val_32 = (int32_t)((int32_t)exp_fnc(fp_data[i*w*256 + j*16 + 5*k*w*16 + 5*k + 3])*(int32_t)h_scales); //Q2.14 * Q2.14 = Q4.28
	  val_16 = (int16_t)(val_32 >> 14); //Q4.28 to Q2.14
	  val_32 = (int32_t)((int32_t)val_16*(int32_t)yolo_bias[2*(k+(1-first_yolo)+3*first_yolo)+1]); //Q2.14 * Q10.6 = Q12.20
	  boxes[84*nboxes+3] = (int16_t)(val_32 >> 6); //Q12.20 to Q2.14

          //check if prob score is higher than threshold
          n_start = 5*k + 5;
	  n_end = 16;
          for(m = 0; m < 6; m++) {
            for(n = n_start; n < n_end; n++) {
              val_32 = (int32_t)((int32_t)fp_data[i*w*256 + j*16 + 5*k*w*16 + m*w*16 + n]*(int32_t)obj_score<<6); //Q8.8 * Q2.14 = Q10.22
              val_16 = (int16_t)(val_32 >> 8); //Q10.22 to Q2.14
	      if(val_16 > (threshold << 6)) boxes[84*nboxes+4+m*16+n-(5*k+5)] = val_16;
 	      n_start = 0;
	    }
	    if(m == 4) n_end = 5*k + 5;
	  }

	  //Update number of candidate boxes
	  nboxes++;
        }
      }
    }
  }
}

//send results back
void send_data() {

  //Loop to send data
  int i, j;
  count_bytes = 0;
  char * fp_data_char = (char *) boxes;
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
  uart_printf("\nPOS CNN\n\n");

#ifndef SIM

  //init ETHERNET
  eth_init(ETHERNET_BASE);
  eth_set_rx_payload_size(ETH_NBYTES);

  //receive data via ethernet
  rcv_data();
#endif

  //create boxes from 1st yolo layer
  uart_printf("\nCreating boxes from first yolo layer\n");
  start = timer_time_us(TIMER_BASE);
  create_boxes(LAYER_16_W, 0, yolo1_div, 1);
  end = timer_time_us(TIMER_BASE);
  uart_printf("Done in %d us\n\n", end-start);

  //create boxes from 2nd yolo layer
  uart_printf("\nCreating boxes from second yolo layer\n");
  start = timer_time_us(TIMER_BASE);
  create_boxes(LAYER_23_W, 256*13*13, yolo2_div, 0);
  end = timer_time_us(TIMER_BASE);
  uart_printf("Done in %d us\n\n", end-start);

  //verify results
#ifdef SIM
  int16_t * fp_data = (int16_t *) DATA_BASE_ADDRESS;
  fp_data += 256*13*13 + 256*26*26;
  int i;
  uart_printf("\nVerifying...\n");
  for(i = 0; i < 84*nboxes; i++)
    if(boxes[i] != fp_data[i])
      uart_printf("(%d) res = %x, act = %x\n", i, boxes[i] & 0xFFFF, fp_data[i] & 0xFFFF);
#endif

#ifndef SIM
  send_data();
#endif

  //finish
  uart_putc(ETX);
  uart_txwait();
  return 0;
}
