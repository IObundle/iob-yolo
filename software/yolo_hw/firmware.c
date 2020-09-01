//import custom libraries
#include "system.h" //must be defined before versat API!!!
#include "periphs.h"
#include "iob-uart.h"
#include "iob-eth.h"
#include "iob_timer.h"
#include "new_versat.hpp"
#include "firmware.h"

//import c libraries
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//print time of each run
//#define TIME_RUN

#ifdef SIM
  int k_delta;
#endif

//obj and prob score threshold
#define threshold ((int16_t)(((float)0.5)*((int32_t)1<<8))) //Q8.8

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

//TILE sizes
#define LAYER_1_TILE_W 208
#define LAYER_3_TILE_W 104
#define LAYER_5_TILE_W 52
#define LAYER_7_TILE_W 26

//define ethernet constants
#define ETH_NBYTES (1024-18) //minimum ethernet payload excluding FCS
#define INPUT_FILE_SIZE ((TOTAL_WEIGHTS + DATA_LAYER_1)*2) //16 bits
#define NUM_INPUT_FRAMES (INPUT_FILE_SIZE/ETH_NBYTES)
#define OUTPUT_FILE_SIZE (DATA_LAYER_23*2) //16 bits
#define NUM_OUTPUT_FRAMES (OUTPUT_FILE_SIZE/ETH_NBYTES)

//define DDR mapping
#define WEIGHTS_BASE_ADDRESS (DDR_MEM + (1 << (FIRM_ADDR_W))) //after main mem
#define DATA_BASE_ADDRESS (WEIGHTS_BASE_ADDRESS + 2*TOTAL_WEIGHTS) //16 bits

//ETHERNET variables
int rcv_timeout = 5000;
char data_to_send[ETH_NBYTES];
char data_rcv[ETH_NBYTES+18];
unsigned int bytes_to_receive, bytes_to_send, count_bytes;

//TIMER variables
unsigned int start, end;

//weights and data updatable initial positions
unsigned int w_pos = 0, p_pos = 0;

//reset certain DDR positions to zero due to padding
void reset_DDR() {

  //local variables
  unsigned int i, pos = DATA_LAYER_1;
  int16_t * fp_data = (int16_t *) DATA_BASE_ADDRESS;

  //measure initial time
  uart_printf("\nSetting DDR positions to zero\n");
  start = timer_time_us(TIMER_BASE);

  //layer 2
  for(i = 0; i < DATA_LAYER_2; i++) fp_data[pos + i] = 0;
  pos += DATA_LAYER_2;

  //layer 4
  for(i = 0; i < DATA_LAYER_4; i++) fp_data[pos + i] = 0;
  pos += DATA_LAYER_4;

  //layer 6
  for(i = 0; i < DATA_LAYER_6; i++) fp_data[pos + i] = 0;
  pos += DATA_LAYER_6;

  //layer 8
  for(i = 0; i < DATA_LAYER_8; i++) fp_data[pos + i] = 0;
  pos += DATA_LAYER_8;

  //layer 10
  for(i = 0; i < DATA_LAYER_10; i++) fp_data[pos + i] = 0;
  pos += DATA_LAYER_10;

  //layer 11
  for(i = 0; i < DATA_LAYER_11; i++) fp_data[pos + i] = 0x8000; //min value
  pos += DATA_LAYER_11;

  //layer 12
  for(i = 0; i < DATA_LAYER_12; i++) fp_data[pos + i] = 0;
  pos += DATA_LAYER_12 + DATA_LAYER_13;

  //layer 14
  for(i = 0; i < DATA_LAYER_14; i++) fp_data[pos + i] = 0;
  pos += DATA_LAYER_14 + DATA_LAYER_15 + DATA_LAYER_16;

  //layer 19/20 and 9
  for(i = 0; i < DATA_LAYER_19 + DATA_LAYER_9; i++) fp_data[pos + i] = 0;

  //measure final time
  end = timer_time_us(TIMER_BASE);
  uart_printf("DDR reset to zero done in %d ms\n", (end-start)/1000);
}

//receive weigths and resized padded image
void rcv_data() {

  //Local variables
  int i, j;
  count_bytes = 0;
  char * data_p = (char *) WEIGHTS_BASE_ADDRESS;
  uart_printf("\nReady to receive input image and weights...\n");

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

/////////////////////////////////////////////////////////////////////////////////////////////////
//layer 1 -> reads full line so we can use tile width > 32
/////////////////////////////////////////////////////////////////////////////////////////////////
void layer1() {
  
  //local variables
  int j, k, l;
  unsigned int p_out;
#ifdef SIM
  k_delta = LAYER_1_W/(2*nSTAGES);
#endif

  //update initial positions
  w_pos += WEIGHTS_LAYER_1;
  p_pos += DATA_LAYER_1;
  p_out = DATA_BASE_ADDRESS + 2*(p_pos + (LAYER_1_W/2+2+1)*LAYER_1_NUM_KER); //pass first padding line and column

  /////////////////////////////////////////////////////////////////////////
  //                          FIXED CONFIGURATIONS
  /////////////////////////////////////////////////////////////////////////
  
  // configure xyolo_read vreads to read bias and kernel from DDR
  versat.yread.setLen(LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*LAYER_1_C + LAYER_1_W_OFF);
  versat.yread.setOffset(2*(LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*LAYER_1_C + LAYER_1_W_OFF));
  versat.yread.setExtPer((LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*LAYER_1_C + LAYER_1_W_OFF)/16);
  versat.yread.setExtIncr(16);
  versat.yread.setPingPong(1);

  // configure xyolo_write vread to read full line
  versat.ywrite.read.setLen((LAYER_1_C*(LAYER_1_W+2)+LAYER_1_P_OFF)*(nSTAGES*2+2)/16-1);
  versat.ywrite.read.setOffset(2*(2*((LAYER_1_W+2)*LAYER_1_C+LAYER_1_P_OFF)));
  versat.ywrite.read.setExtPer((LAYER_1_C*(LAYER_1_W+2)+LAYER_1_P_OFF)*(LAYER_1_KER_SIZE+1)/16); //+10 so each line is 32 byte aligned
  versat.ywrite.read.setExtIncr(16);
  versat.ywrite.read.setPingPong(1);

  // configure xyolo_read vreads to write 1 + 3x3x3 kernel to flow_outs
  versat.yread.setIntPer(LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*LAYER_1_C/nYOLOmacs);
  versat.yread.setIntIncr(1);
  versat.yread.setIntIter(2*LAYER_1_TILE_W);
  versat.yread.setIntShift(-LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*LAYER_1_C/nYOLOmacs);

  // configure xyolo_write vread to write FM tile to xyolo
  versat.ywrite.read.setIntPer(LAYER_1_KER_SIZE*LAYER_1_C/nYOLOmacs);
  versat.ywrite.read.setIntIncr(1);
  versat.ywrite.read.setIntIter(LAYER_1_KER_SIZE);
  versat.ywrite.read.setIntShift(((LAYER_1_W+2)*LAYER_1_C+LAYER_1_P_OFF)/nYOLOmacs - LAYER_1_KER_SIZE*LAYER_1_C/nYOLOmacs);
  versat.ywrite.read.setIntPer2(2);
  versat.ywrite.read.setIntIncr2(LAYER_1_C/nYOLOmacs);
  versat.ywrite.read.setIntIter2(2);
  versat.ywrite.read.setIntShift2(((LAYER_1_W+2)*LAYER_1_C+LAYER_1_P_OFF)/nYOLOmacs - 2*LAYER_1_C/nYOLOmacs);
  versat.ywrite.read.setIntPer3(LAYER_1_TILE_W/2);
  versat.ywrite.read.setIntIncr3(2*LAYER_1_C/nYOLOmacs);
  versat.ywrite.read.setIntIter3(1);

  // configure xyolo to perform convolution + maxpool
  versat.ywrite.yolo.setIter(2*LAYER_1_TILE_W);
  versat.ywrite.yolo.setPer(LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*LAYER_1_C/nYOLOmacs);
  versat.ywrite.yolo.setShift(10);
  versat.ywrite.yolo.setBias(1);
  versat.ywrite.yolo.setLeaky(1);
  versat.ywrite.yolo.setMaxpool(1);

  // configure xwrite to write convolution results
  versat.ywrite.write.setIntDuty(1);
  versat.ywrite.write.setIntDelay(XYOLO_READ_LAT + XYOLO_WRITE_LAT - 2);
  versat.ywrite.write.setIntPer(4*LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*LAYER_1_C/nYOLOmacs);
  versat.ywrite.write.setIntIncr(1);
  versat.ywrite.write.setIntIter(LAYER_1_TILE_W/2);

  // configure xyolo_write vwrite to write result back to DDR
  versat.ywrite.write.setLen(LAYER_1_TILE_W/2-1);
  versat.ywrite.write.setOffset(2*((LAYER_1_W/2+2)*LAYER_1_NUM_KER));
  versat.ywrite.write.setExtPer(1);
  versat.ywrite.write.setExtIncr(nYOLOvect);
  versat.ywrite.write.setExtIter(LAYER_1_TILE_W/2);
  versat.ywrite.write.setExtShift(LAYER_1_NUM_KER-nYOLOvect);

  /////////////////////////////////////////////////////////////////////////
  //                      VARIABLE CONFIGURATIONS
  /////////////////////////////////////////////////////////////////////////
  
  // perform layer 1 convolution + layer 2 maxpool
  for(l = 0; l < LAYER_1_NUM_KER/nYOLOvect; l++) {

    // read filter
    versat.yread.setExtIter(1);
    versat.yread.setExtAddr(WEIGHTS_BASE_ADDRESS + 2*LAYER_1_NUM_KER + 2*l*nYOLOvect*(LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*LAYER_1_C + LAYER_1_W_OFF));
    versat.yread.setBiasExtAddr(WEIGHTS_BASE_ADDRESS + 2*l*nYOLOvect);

  #ifdef SIM
    for(k = 0; k < k_delta; k++) {
  #else
    for(k = 0; k < LAYER_1_W/(2*nSTAGES); k++) {
  #endif

      // configure xyolo_write vread to read tile from input fm
      versat.ywrite.read.setExtIter(1);
      versat.ywrite.read.setExtAddr(DATA_BASE_ADDRESS + 2*(k*2*((LAYER_1_W+2)*LAYER_1_C+LAYER_1_P_OFF)*nSTAGES));

      for(j = 0; j < LAYER_1_W/LAYER_1_TILE_W; j++) {
      
        // configure xyolo_write vread start
        versat.ywrite.read.setIntStart(j*LAYER_1_TILE_W*LAYER_1_C/nYOLOmacs);

        // configure xyolo_write vwrite to write result back to DDR
        versat.ywrite.write.setExtAddr(p_out + 2*(k*(LAYER_1_W/2+2)*LAYER_1_NUM_KER*nSTAGES + j*(LAYER_1_TILE_W/2)*LAYER_1_NUM_KER + l*nYOLOvect));

        // wait until done
        while(versat.done()==0);
      #ifdef TIME_RUN
        end = (unsigned int) timer_get_count(TIMER_BASE);
        if(l != 0 || k != 0 || j != 0) uart_printf("%d\n", (end - start)*8);
      #endif

        // run configuration
        versat.run();
      #ifdef TIME_RUN
        start = (unsigned int) timer_get_count(TIMER_BASE);
      #endif

        // stop reading from DDR
        versat.ywrite.read.setExtIter(0);
        versat.yread.setExtIter(0);
      }
    }
  }

  // clear configs
  versat.clear();
}

// w -> input feature map width (same as height)
// c -> number of input channels
// num_ker -> number of kernels
// ker_size -> width of kernel (same as height)
// til_w -> width of tile
// w_start -> flag to indicate if weight mem starts writing from zero (0) or from given position (1) -> only for layer 3
/////////////////////////////////////////////////////////////////////////////////////////////////
// STRATEGY -> Pixel reuse : apply all kernels to current FM tile before moving to next tile
////////////////////////////////////////////////////////////////////////////////////////////////
void conv(int w, int c, int num_ker, int ker_size, int til_w, int w_start) {

  //local variables
  int j, k, l;
#ifdef SIM
  k_delta = w/(2*nSTAGES);
#endif
  unsigned int w_in = WEIGHTS_BASE_ADDRESS + 2*w_pos;
  unsigned int p_in = DATA_BASE_ADDRESS + 2*p_pos;
  unsigned int p_out;

  //update initial positions
  w_pos += num_ker*(1 + ker_size*ker_size*c);
  p_pos += (w+2)*(w+2)*c;
  p_out = DATA_BASE_ADDRESS + 2*(p_pos + (w/2+2+1)*num_ker); //pass first padding line and column

  /////////////////////////////////////////////////////////////////////////
  //                          FIXED CONFIGURATIONS
  /////////////////////////////////////////////////////////////////////////

  // configure xyolo_read vreads to read bias and kernel from DDR
  versat.yread.setLen(ker_size*ker_size*c);
  versat.yread.setOffset(2*(ker_size*ker_size*c));
  versat.yread.setExtPer((ker_size*ker_size*c)/16);
  versat.yread.setExtIncr(16);
  versat.yread.setExtIter(1);

  // configure xyolo_write vread to read tile from input fm
  versat.ywrite.read.setLen((c*(til_w+2))/16-1);
  versat.ywrite.read.setOffset(2*(2*((w+2)*c)));
  versat.ywrite.read.setExtPer((c*(til_w+2))/16);
  versat.ywrite.read.setExtIncr(16);
  versat.ywrite.read.setExtShift(((w+2)*c) - (c*(til_w+2))); //+2 due to padding
  versat.ywrite.read.setPingPong(1);

  // configure xyolo_read vreads to write 1 + 3x3x3 kernel to flow_outs
  versat.yread.setIntPer(ker_size*ker_size*c/nYOLOmacs);
  versat.yread.setIntIncr(1);
  versat.yread.setIntIter(2*til_w);
  versat.yread.setIntShift(-ker_size*ker_size*c/nYOLOmacs);

  // configure xyolo_write vread to write FM tile to xyolo
  versat.ywrite.read.setIntPer(ker_size*c/nYOLOmacs);
  versat.ywrite.read.setIntIncr(1);
  versat.ywrite.read.setIntIter(ker_size);
  versat.ywrite.read.setIntShift((til_w+2)*c/nYOLOmacs - ker_size*c/nYOLOmacs); //+2 due to padding
  versat.ywrite.read.setIntIncr2(c/nYOLOmacs);
  versat.ywrite.read.setIntPer2(2);
  versat.ywrite.read.setIntIter2(2);
  versat.ywrite.read.setIntShift2((til_w+2)*c/nYOLOmacs - 2*c/nYOLOmacs); //+2 due to padding
  versat.ywrite.read.setIntPer3(til_w/2); // /2 due to maxpool
  versat.ywrite.read.setIntIncr3(2*c/nYOLOmacs);
  versat.ywrite.read.setIntIter3(1);

  // configure xyolo to perform convolution + maxpool
  versat.ywrite.yolo.setIter(2*til_w);
  versat.ywrite.yolo.setPer(ker_size*ker_size*c/nYOLOmacs);
  versat.ywrite.yolo.setShift(10);
  versat.ywrite.yolo.setBias(1);
  versat.ywrite.yolo.setLeaky(1);
  versat.ywrite.yolo.setMaxpool(1);

  // configure xwrite to write convolution results
  versat.ywrite.write.setIntDuty(1);
  versat.ywrite.write.setIntDelay(XYOLO_READ_LAT + XYOLO_WRITE_LAT - 2);
  versat.ywrite.write.setIntPer(4*ker_size*ker_size*c/nYOLOmacs);
  versat.ywrite.write.setIntIncr(num_ker/nYOLOvect);
  versat.ywrite.write.setIntIter(til_w/2);

  // configure xyolo_write vwrite to write result back to DDR
  versat.ywrite.write.setLen(((til_w/2)*(num_ker/nYOLOvect))-1);
  versat.ywrite.write.setOffset(2*((w/2+2)*num_ker));
  versat.ywrite.write.setExtPer(1);

  /////////////////////////////////////////////////////////////////////////
  //                      VARIABLE CONFIGURATIONS
  /////////////////////////////////////////////////////////////////////////
  
#ifdef SIM
  for(k = 0; k < k_delta; k++) {
#else
  for(k = 0; k < w/(2*nSTAGES); k++) {
#endif
    for(j = 0; j < w/til_w; j++) {

      // configure xyolo_write vread to read tile from input fm
      versat.ywrite.read.setExtIter(ker_size+1);
      versat.ywrite.read.setExtAddr(p_in + 2*(k*2*((w+2)*c)*nSTAGES + j*til_w*c));

      // simulate for first yolo layer
      for(l = 0; l < num_ker/nYOLOvect; l++) {

        // read weights
        versat.yread.setExtAddr(w_in + 2*(nYOLOvect + l*nYOLOvect*(1+ker_size*ker_size*c)));
	versat.yread.setIntAddr(((ker_size*ker_size*c)*l + 144*w_start)/16); //144 is the final position of the first filter of layer 5
        versat.yread.setIntStart((ker_size*ker_size*c)*l/nYOLOmacs + 144*w_start/nYOLOmacs);

	// read bias
        versat.yread.setBiasExtAddr(w_in + 2*l*nYOLOvect*(1+ker_size*ker_size*c));
        versat.yread.setBiasIntAddr(l + w_start);
	versat.yread.setBiasIntStart(l + w_start);

        // configure xyolo_write vwrite start
        versat.ywrite.write.setIntStart(l);

        // configure xyolo_write vwrite to write result back to DDR
        if(l == num_ker/nYOLOvect-1) {
          versat.ywrite.write.setExtAddr(p_out + 2*(k*(w/2+2)*num_ker*nSTAGES + j*(til_w/2*num_ker)));
  	  versat.ywrite.write.setExtIter(((til_w/2)*(num_ker/nYOLOvect)));
        } else versat.ywrite.write.setExtIter(0);

        // wait until done
        while(versat.done()==0);
      #ifdef TIME_RUN
        end = (unsigned int) timer_get_count(TIMER_BASE);
        uart_printf("%d\n", (end - start)*8);
        if(k == 0 && j == 0 && l == 0) uart_printf("\n");
      #endif

        // run configuration
        versat.run();
      #ifdef TIME_RUN
        start = (unsigned int) timer_get_count(TIMER_BASE);
      #endif

        // stop xyolo_write vread reading from DDR
        versat.ywrite.read.setExtIter(0);
      }

      //stop xyolo_read vread reading from DDR
      versat.yread.setExtIter(0);
    }
  }

  // clear configs
  versat.clear();
}

// w -> input feature map width (same as height)
// c -> number of input channels
// num_ker -> number of kernels
// ker_size -> width of kernel (same as height)
/////////////////////////////////////////////////////////////////////////////////////////////////
// STRATEGY -> Weight reuse : apply current kernel to all tiles before moving to next kernel
////////////////////////////////////////////////////////////////////////////////////////////////
void conv2(int w, int c, int num_ker, int ker_size, int outpadd, int stride, int pp, int inpadd, int zxy, int leaky, int ignorepadd, unsigned int outpos, int upsample) {

  //local variables
  int k, l;
#ifdef SIM
  k_delta = w/nSTAGES;
#endif
  unsigned int w_in = WEIGHTS_BASE_ADDRESS + 2*w_pos;
  unsigned int p_in = DATA_BASE_ADDRESS + 2*(p_pos + ignorepadd*(w+2)*c);
  unsigned int p_out;

  //update initial positions
  w_pos += num_ker*(1 + ker_size*ker_size*c);
  //pass first padding line and first 16 pixels of next line
  if(outpos == 0) { 
    p_pos += (w+2*inpadd)*(w+2*inpadd)*c;
    p_out = DATA_BASE_ADDRESS + 2*(p_pos + ((w+2)*num_ker + 16)*outpadd);
  //else only for layers 9 and 14, whose output is later on joined for layer 22
  } else {
    p_pos = outpos;
    p_out = DATA_BASE_ADDRESS + 2*(outpos + (LAYER_22_W+2)*(LAYER_9_NUM_KER+LAYER_19_NUM_KER) + 16);
  }

  // extra run to make last computation
  if(!upsample) {
    while(versat.done()==0);
   #ifdef TIME_RUN
    end = (unsigned int) timer_get_count(TIMER_BASE);
    uart_printf("%d\n", (end - start)*8);
   #endif
    versat.run();
   #ifdef TIME_RUN
    start = (unsigned int) timer_get_count(TIMER_BASE);
   #endif
  }

  /////////////////////////////////////////////////////////////////////////
  //                          FIXED CONFIGURATIONS
  /////////////////////////////////////////////////////////////////////////

  // configure xyolo_read vreads to read bias and kernel from DDR
  versat.yread.setLen(ker_size*ker_size*c);
  versat.yread.setOffset(2*(ker_size*ker_size*c));
  versat.yread.setExtPer((ker_size*ker_size*c)/16);
  versat.yread.setExtIncr(16);
  versat.yread.setExtIter(1);
  versat.yread.setPingPong(1);

  // configure xyolo_write vread to read input fm
  versat.ywrite.read.setLen(c*(w+2*inpadd+2*ignorepadd)*(nSTAGES+2*inpadd)/16-1);
  versat.ywrite.read.setOffset(2*((w+2*inpadd+2*ignorepadd)*c));
  versat.ywrite.read.setExtPer((c*(w+2*inpadd+2*ignorepadd)*ker_size)/16);
  versat.ywrite.read.setExtIncr(16);
  versat.ywrite.read.setPingPong(pp);

  // configure xyolo_read vreads to write 1 + 3x3x3 kernel to flow_outs
  versat.yread.setIntPer(ker_size*ker_size*c/nYOLOmacs);
  versat.yread.setIntIncr(1);
  versat.yread.setIntIter(w*(1+upsample));
  versat.yread.setIntShift(-ker_size*ker_size*c/nYOLOmacs);

  // configure xyolo_write vread to write FM tile to xyolo
  versat.ywrite.read.setIntIncr(1);
  if(zxy) {
    versat.ywrite.read.setIntPer(ker_size*c/nYOLOmacs);
    versat.ywrite.read.setIntIter(ker_size);
    versat.ywrite.read.setIntShift((w+2*inpadd)*c/nYOLOmacs - ker_size*c/nYOLOmacs); //+2 due to padding
    versat.ywrite.read.setIntIncr2(c/nYOLOmacs);
    versat.ywrite.read.setIntPer2(w);
    versat.ywrite.read.setIntIter2(1);
  } else {
    versat.ywrite.read.setIntStart(16*ignorepadd/nYOLOmacs);
    versat.ywrite.read.setIntPer(16/nYOLOmacs);
    versat.ywrite.read.setIntIter(c/16);
    versat.ywrite.read.setIntShift((w+2*inpadd+2*ignorepadd)*16/nYOLOmacs - 16/nYOLOmacs);
    versat.ywrite.read.setIntIncr2(16/nYOLOmacs);
    if(ker_size == 1) {
      versat.ywrite.read.setIntPer2(w);
      versat.ywrite.read.setIntIter2(1*(1+upsample));
      versat.ywrite.read.setIntShift2(-16*w/nYOLOmacs);
    } else {
      versat.ywrite.read.setIntPer2(ker_size);
      versat.ywrite.read.setIntIter2(ker_size);
      versat.ywrite.read.setIntShift2((w+2*inpadd)*c/nYOLOmacs - 16*ker_size/nYOLOmacs);
      versat.ywrite.read.setIntIncr3(16/nYOLOmacs);
      versat.ywrite.read.setIntPer3(w);
      versat.ywrite.read.setIntIter3(1);
    } 
  }

  // configure xyolo to perform convolution
  versat.ywrite.yolo.setIter(w*(1+upsample)+upsample); //+upsample to use duty of 2
  versat.ywrite.yolo.setPer(ker_size*ker_size*c/nYOLOmacs);
  versat.ywrite.yolo.setShift(10);
  versat.ywrite.yolo.setBias(1);
  versat.ywrite.yolo.setLeaky(leaky);
  versat.ywrite.yolo.setSigmoid(1-leaky);

  // configure xwrite to write convolution results
  versat.ywrite.write.setIntDuty(1+upsample);
  versat.ywrite.write.setIntDelay(XYOLO_READ_LAT + XYOLO_WRITE_LAT - 2 + (1-leaky));
  versat.ywrite.write.setIntPer(ker_size*ker_size*c/nYOLOmacs);
  versat.ywrite.write.setIntIncr(1);
  versat.ywrite.write.setIntIter(w*(1+upsample)+upsample); //+upsample to use duty of 2

  // configure xyolo_write vwrite to write result back to DDR
  versat.ywrite.write.setLen((1+upsample)*w-1);
  if(outpos == 0) versat.ywrite.write.setOffset(2*((w+2*outpadd+stride)*num_ker));
  else versat.ywrite.write.setOffset(2*((1+upsample)*(w*(1+upsample)+2*outpadd+stride)*(LAYER_9_NUM_KER+LAYER_19_NUM_KER)));
  versat.ywrite.write.setExtPer(w*(1+upsample));
  versat.ywrite.write.setExtIter(1+upsample);
  versat.ywrite.write.setExtShift((w*(1+upsample)+2*outpadd+stride)*(LAYER_9_NUM_KER+LAYER_19_NUM_KER));

  /////////////////////////////////////////////////////////////////////////
  //                      VARIABLE CONFIGURATIONS
  /////////////////////////////////////////////////////////////////////////
  
#ifdef SIM
  for(k = 0; k < k_delta; k++) {
#else
  for(k = 0; k < w/nSTAGES; k++) {
#endif

    // reset run for second tile of layer 22 (each tile fits in the whole memory)
    if(c == LAYER_9_NUM_KER+LAYER_19_NUM_KER && k != 0) {

      //stop reading weights and computation
      versat.yread.setExtIter(0);
      versat.yread.setIntIter(0);
      versat.ywrite.read.setIntIter(0);
      versat.ywrite.read.setIntIter2(0);
      versat.ywrite.read.setIntIter3(0);
      versat.ywrite.yolo.setIter(0);
      versat.ywrite.write.setIntIter(0);
      versat.ywrite.write.setExtIter(0);

      //extra run (otherwise mems must be bigger for layer 22)
      while(versat.done()==0);
    #ifdef TIME_RUN
      end = (unsigned int) timer_get_count(TIMER_BASE);
      uart_printf("%d\n", (end - start)*8);
    #endif
      versat.run();
    #ifdef TIME_RUN
      start = (unsigned int) timer_get_count(TIMER_BASE);
    #endif

      //recover former configs
      versat.yread.setExtIter(1);
      versat.yread.setIntIter(w*(1+upsample));
      versat.ywrite.read.setIntIter(c/16);
      versat.ywrite.read.setIntIter2(ker_size);
      versat.ywrite.read.setIntIter3(1);
      versat.ywrite.yolo.setIter(w*(1+upsample)+upsample); //+upsample to use duty of 2
      versat.ywrite.write.setIntIter(w*(1+upsample)+upsample); //+upsample to use duty of 2
      versat.ywrite.write.setExtIter(1+upsample);
    }

    // configure xyolo_write vread to read tile from input fm
    versat.ywrite.read.setExtIter(1);
    versat.ywrite.read.setExtAddr(p_in + 2*(k*(w+2*inpadd)*c*nSTAGES));

    // run convolution
    for(l = 0; l < num_ker/nYOLOvect; l++) {

      // set sigmoid mask
      if(l == 0) versat.ywrite.yolo.setSigMask(65523);  	// 1111111111110011
      else if(l == 5) versat.ywrite.yolo.setSigMask(65151);  	// 1111111001111111
      else if(l == 10) versat.ywrite.yolo.setSigMask(53247); 	// 1100111111111111
      else versat.ywrite.yolo.setSigMask(65535);        	// 1111111111111111

      // read weights
      versat.yread.setExtAddr(w_in + 2*(nYOLOvect + l*nYOLOvect*(1+ker_size*ker_size*c)));

      // read bias
      versat.yread.setBiasExtAddr(w_in + 2*l*nYOLOvect*(1+ker_size*ker_size*c));

      // configure xyolo_write vwrite to write result back to DDR
      if(outpos == 0) versat.ywrite.write.setExtAddr(p_out + 2*(k*(w+2*outpadd+stride)*num_ker*nSTAGES + l*(w*(1+upsample)+2*outpadd+stride)*16));
      else versat.ywrite.write.setExtAddr(p_out + 2*(k*(w+2*outpadd+stride)*(LAYER_9_NUM_KER+LAYER_19_NUM_KER)*nSTAGES + l*(w*(1+upsample)+2*outpadd+stride)*16));

      // wait until done
      while(versat.done()==0);
    #ifdef TIME_RUN
      end = (unsigned int) timer_get_count(TIMER_BASE);
      uart_printf("%d\n", (end - start)*8);
      if(k == 0 && l == 0) uart_printf("\n");
    #endif

      // run configuration
      versat.run();
    #ifdef TIME_RUN
      start = (unsigned int) timer_get_count(TIMER_BASE);
    #endif

      // stop xyolo_write vread reading from DDR
      versat.ywrite.read.setExtIter(0);
    }
  }

  // clear configs
  versat.clear();
}

//run maxpool layer in versat
//input data is in format 16 channels per pixel
//ouput data is converted back again to zxy format
void maxpool(int w, int c, int inpadd, int stride, unsigned int outpos) {

  // extra run to make last computation
  while(versat.done()==0);
 #ifdef TIME_RUN
  end = (unsigned int) timer_get_count(TIMER_BASE);
  uart_printf("%d\n", (end - start)*8);
 #endif
  versat.run();
 #ifdef TIME_RUN
  start = (unsigned int) timer_get_count(TIMER_BASE);
 #endif

  //local variables
  int l;
  unsigned int p_in, p_out;
  if(outpos == 0) p_in = DATA_BASE_ADDRESS + 2*(p_pos + (w+2)*c*inpadd); //pass first padding line
  else p_in = DATA_BASE_ADDRESS + 2*(p_pos + (LAYER_22_W+2)*(LAYER_9_NUM_KER+LAYER_19_NUM_KER));

  //update initial positions
  if(outpos == 0) p_pos += (w+2*inpadd+stride)*(w+2*inpadd+stride)*c;
  else p_pos = outpos;
  p_out = DATA_BASE_ADDRESS + 2*(p_pos + (w/(1+inpadd)+2+1)*c); //pass first padding line and column

  /////////////////////////////////////////////////////////////////////////
  //                          FIXED CONFIGURATIONS
  /////////////////////////////////////////////////////////////////////////

  // configure xyolo_write vread to read input fm
  if(outpos == 0) {
    versat.ywrite.read.setLen(c*(w+2*inpadd+stride)*(nSTAGES*(1+inpadd)+stride)/16-1);
    versat.ywrite.read.setOffset(2*((1+inpadd)*(w+2*inpadd+stride)*c)); 
    versat.ywrite.read.setExtPer((c*(w+2*inpadd+stride)*2)/16); //each vread reads 2 lines
    versat.ywrite.read.setExtIter(1);
  } else {
    versat.ywrite.read.setLen(c*(w+2*inpadd+stride)/16-1);
    versat.ywrite.read.setOffset(2*((1+inpadd)*(w+2*inpadd+stride)*(LAYER_9_NUM_KER+LAYER_19_NUM_KER)));
    versat.ywrite.read.setExtPer(c*(w+2*inpadd+stride)/16); //each vread reads 2 lines
    versat.ywrite.read.setExtIter(2);
    versat.ywrite.read.setExtShift((w+2*inpadd+stride)*(LAYER_9_NUM_KER+LAYER_19_NUM_KER) - c*(w+2*inpadd+stride));
  }
  versat.ywrite.read.setExtIncr(16);
  versat.ywrite.read.setExtAddr(p_in);

  // configure xyolo_write vread to write FM tile to xyolo
  versat.ywrite.read.setIntPer(2);
  versat.ywrite.read.setIntIncr(16/nYOLOmacs);
  versat.ywrite.read.setIntIter(2);
  versat.ywrite.read.setIntShift((w+2*inpadd+stride)*c/nYOLOmacs - 32/nYOLOmacs);
  versat.ywrite.read.setIntPer2(16/nYOLOmacs);
  versat.ywrite.read.setIntIter2(1);
  versat.ywrite.read.setIntIncr3(1);
  versat.ywrite.read.setIntPer3(nYOLOmacs);
  versat.ywrite.read.setIntIter3(c/16);
  versat.ywrite.read.setIntShift3((w+2*inpadd+stride)*16/nYOLOmacs - nYOLOmacs);

  // configure xyolo to perform maxpool
  versat.ywrite.yolo.setIter(4*c/nYOLOmacs);
  versat.ywrite.yolo.setPer(nYOLOmacs);
  versat.ywrite.yolo.setMaxpool(1);
  versat.ywrite.yolo.setBypass(1);

  // configure xwrite to write convolution results
  // hw internally has counter to adress each vwrite individually when bypass is enabled
  versat.ywrite.write.setIntDuty(4*16);
  versat.ywrite.write.setIntDelay(XYOLO_READ_LAT + XYOLO_WRITE_BYPASS_LAT - 2);
  versat.ywrite.write.setIntPer(4*16);
  versat.ywrite.write.setIntIter(c/16);
  versat.ywrite.write.setIntShift(1);

  // configure xyolo_write vwrite to write result back to DDR
  versat.ywrite.write.setLen(c/16-1);
  versat.ywrite.write.setOffset(2*((w/(1+inpadd)+2)*c));
  versat.ywrite.write.setExtPer(c/16);
  versat.ywrite.write.setExtIter(1);
  
  // perform maxpool
  for(l = 0; l < w/(1+inpadd); l++) {

    // configure xyolo_write vread start
    versat.ywrite.read.setIntStart(16*inpadd/nYOLOmacs + (2-stride)*16*l/nYOLOmacs);

    // configure xyolo_write vwrite to write result back to DDR
    versat.ywrite.write.setExtAddr(p_out + 2*c*l);

    // wait until done
    while(versat.done()==0);
  #ifdef TIME_RUN
    end = (unsigned int) timer_get_count(TIMER_BASE);
    uart_printf("%d\n", (end - start)*8);
    if(l == 0) uart_printf("\n");
  #endif

    // run configuration
    versat.run();
  #ifdef TIME_RUN
    start = (unsigned int) timer_get_count(TIMER_BASE);
  #endif

    // stop xyolo_write vread reading from DDR
    versat.ywrite.read.setExtIter(0);
  }

  // clear configs
  versat.clear();
}

//print detected objects and corresponding probability scores
void print_results(int w, unsigned int pos) {

  //local variable
  int i, j, k, m;
  int16_t arr[256], val_16;
  const char *class_names[80] = {"person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed", "dining table", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};
  int16_t * fp_data = (int16_t *) DATA_BASE_ADDRESS + pos;
  int32_t val_32;

  //print detections
  for(i = 0; i < w; i++) {
    for(j = 0; j < w; j++) {

      //fill array
      for(k = 0; k < 16; k++)
        for(m = 0; m < 16; m++)
          arr[k*16 + m] = fp_data[i*w*256 + j*16 + k*w*16 + m];

      //check if obj score is higher than threshold
      for(k = 0; k < 255; k+=85) {
        if(arr[k+4] > threshold) {
          //check if prob score is higher than threshold
          for(m = 0; m < 80; m++) {
            val_32 = (int32_t)((int32_t)arr[k+5+m]*(int32_t)arr[k+4]<<6); //Q8.8 * Q2.14 = Q10.22
            val_16 = (int16_t)(val_32 >> 8); //Q10.22 to Q2.14
	    if(val_16 > (threshold << 6)) {
	      val_32 = (uint32_t)((uint32_t)val_16*(uint32_t)100); //Q2.14 * Q16.0 = Q18.14
	      if((val_32&0x3FFF) > 0x2000) uart_printf("%s: %d%%\n", class_names[m], (val_32>>14)+1);
	      else uart_printf("%s: %d%%\n", class_names[m], (val_32>>14));
            }
	  }
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
  char * fp_data_char = (char *) DATA_BASE_ADDRESS + 2*(DATA_LAYER_1 + DATA_LAYER_2 + DATA_LAYER_4 + DATA_LAYER_6 + DATA_LAYER_8 + DATA_LAYER_10 + DATA_LAYER_11 + DATA_LAYER_12 + DATA_LAYER_13 + DATA_LAYER_14 + DATA_LAYER_15 + DATA_LAYER_16 + DATA_LAYER_19 + DATA_LAYER_9 + DATA_LAYER_22);
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
  uart_printf("\nYOLO HW\n\n");

  //init VERSAT
  versat_init(VERSAT_BASE);

 #ifndef TIME_RUN
  //local variable to measure total time
  unsigned int total_time;
 #endif

  //initial address of layers 9/10/14/19 output
#ifdef SIM
  unsigned int data_pos_layer10 = DATA_LAYER_8;
  unsigned int data_pos_layer14 = data_pos_layer10 + DATA_LAYER_8;
  unsigned int data_pos_layer19 = data_pos_layer14;
#else
  unsigned int data_pos_layer10 = DATA_LAYER_1 + DATA_LAYER_2 + DATA_LAYER_4 + DATA_LAYER_6 + DATA_LAYER_8;
  unsigned int data_pos_layer14 = data_pos_layer10 + DATA_LAYER_10 + DATA_LAYER_11 + DATA_LAYER_12 + DATA_LAYER_13;
  unsigned int data_pos_layer19 = data_pos_layer14 + DATA_LAYER_14 + DATA_LAYER_15 + DATA_LAYER_16;
#endif
  unsigned int data_pos_layer9 = data_pos_layer19 + (LAYER_19_W*2+2)*LAYER_19_NUM_KER;

#ifndef SIM

  //init ETHERNET
  eth_init(ETHERNET_BASE);
  eth_set_rx_payload_size(ETH_NBYTES);

  //reset DDR due to zero-padding
  reset_DDR();

  //receive data via ethernet
  rcv_data();

  //layers 1 and 2
 #ifndef TIME_RUN
  uart_printf("\nRunning layers 1 and 2...\n");
  start = timer_time_us(TIMER_BASE);
 #endif
  layer1();
 #ifndef TIME_RUN
  end = timer_time_us(TIMER_BASE);
  uart_printf("Convolution + maxpool done in %d us\n\n", end-start);
  total_time = end-start;
 #endif

  //layers 3 and 4
 #ifndef TIME_RUN
  uart_printf("\nRunning layers 3 and 4...\n");
  start = timer_time_us(TIMER_BASE);
 #endif
  conv(LAYER_3_W, LAYER_1_NUM_KER, LAYER_3_NUM_KER, LAYER_3_KER_SIZE, LAYER_3_TILE_W, 1); //w_start(1)
 #ifndef TIME_RUN
  end = timer_time_us(TIMER_BASE);
  uart_printf("Convolution + maxpool done in %d us\n\n", end-start);
  total_time += end-start;
 #endif

  //layers 5 and 6
 #ifndef TIME_RUN
  uart_printf("\nRunning layers 5 and 6...\n");
  start = timer_time_us(TIMER_BASE);
 #endif
  conv(LAYER_5_W, LAYER_3_NUM_KER, LAYER_5_NUM_KER, LAYER_5_KER_SIZE, LAYER_5_TILE_W, 0); //w_start(0)
 #ifndef TIME_RUN
  end = timer_time_us(TIMER_BASE);
  uart_printf("Convolution + maxpool done in %d us\n\n", end-start);
  total_time += end-start;
 #endif

  //layers 7 and 8
 #ifndef TIME_RUN
  uart_printf("\nRunning layers 7 and 8...\n");
  start = timer_time_us(TIMER_BASE);
 #endif
  conv(LAYER_7_W, LAYER_5_NUM_KER, LAYER_7_NUM_KER, LAYER_7_KER_SIZE, LAYER_7_TILE_W, 0); //w_start(0)
 #ifndef TIME_RUN
  end = timer_time_us(TIMER_BASE);
  uart_printf("Convolution + maxpool done in %d us\n\n", end-start);
  total_time += end-start;
 #endif

  //layer 9
 #ifndef TIME_RUN
  uart_printf("\nRunning layer 9...\n");
  start = timer_time_us(TIMER_BASE);
 #endif
  conv2(LAYER_9_W, LAYER_7_NUM_KER, LAYER_9_NUM_KER, LAYER_9_KER_SIZE, LAYER_9_OUTPADD, LAYER_9_STRIDE, LAYER_9_PINGPONG, LAYER_7_OUTPADD, LAYER_9_ZXY, LAYER_9_LEAKY, LAYER_9_IGNOREPAD, data_pos_layer9, LAYER_9_UPSAMPLE);
 #ifndef TIME_RUN
  end = timer_time_us(TIMER_BASE);
  uart_printf("Convolution done in %d us\n\n", end-start);
  total_time += end-start;
 #endif

  //layer 10
 #ifndef TIME_RUN
  uart_printf("\nRunning layer 10...\n");
  start = timer_time_us(TIMER_BASE);
 #endif
  maxpool(LAYER_9_W, LAYER_9_NUM_KER, LAYER_10_INPADD, LAYER_10_STRIDE, data_pos_layer10);
 #ifndef TIME_RUN
  end = timer_time_us(TIMER_BASE);
  uart_printf("Maxpool done in %d us\n\n", end-start);
  total_time += end-start;
 #endif

  //layer 11
 #ifndef TIME_RUN
  uart_printf("\nRunning layer 11...\n");
  start = timer_time_us(TIMER_BASE);
 #endif
  conv2(LAYER_11_W, LAYER_9_NUM_KER, LAYER_11_NUM_KER, LAYER_11_KER_SIZE, LAYER_11_OUTPADD, LAYER_11_STRIDE, LAYER_11_PINGPONG, LAYER_10_OUTPADD, LAYER_11_ZXY, LAYER_11_LEAKY, LAYER_11_IGNOREPAD, 0, LAYER_11_UPSAMPLE); //outpos(0)
 #ifndef TIME_RUN
  end = timer_time_us(TIMER_BASE);
  uart_printf("Convolution done in %d us\n\n", end-start);
  total_time += end-start;
 #endif

  //layer 12
 #ifndef TIME_RUN
  uart_printf("\nRunning layer 12...\n");
  start = timer_time_us(TIMER_BASE);
 #endif
  maxpool(LAYER_11_W, LAYER_11_NUM_KER, LAYER_12_INPADD, LAYER_12_STRIDE, 0); //outpos(0)
 #ifndef TIME_RUN
  end = timer_time_us(TIMER_BASE);
  uart_printf("Maxpool done in %d us\n\n", end-start);
  total_time += end-start;
 #endif

  //layer 13
 #ifndef TIME_RUN
  uart_printf("\nRunning layer 13...\n");
  start = timer_time_us(TIMER_BASE);
 #endif
  conv2(LAYER_13_W, LAYER_11_NUM_KER, LAYER_13_NUM_KER, LAYER_13_KER_SIZE, LAYER_13_OUTPADD, LAYER_13_STRIDE, LAYER_13_PINGPONG, LAYER_12_OUTPADD, LAYER_13_ZXY, LAYER_13_LEAKY, LAYER_13_IGNOREPAD, 0, LAYER_13_UPSAMPLE); //outpos(0)
 #ifndef TIME_RUN
  end = timer_time_us(TIMER_BASE);
  uart_printf("Convolution done in %d us\n\n", end-start);
  total_time += end-start;
 #endif

  //layer 14
 #ifndef TIME_RUN
  uart_printf("\nRunning layer 14...\n");
  start = timer_time_us(TIMER_BASE);
 #endif
  conv2(LAYER_14_W, LAYER_13_NUM_KER, LAYER_14_NUM_KER, LAYER_14_KER_SIZE, LAYER_14_OUTPADD, LAYER_14_STRIDE, LAYER_14_PINGPONG, LAYER_13_OUTPADD, LAYER_14_ZXY, LAYER_14_LEAKY, LAYER_14_IGNOREPAD, 0, LAYER_14_UPSAMPLE); //outpos(0)
 #ifndef TIME_RUN
  end = timer_time_us(TIMER_BASE);
  uart_printf("Convolution done in %d us\n\n", end-start);
  total_time += end-start;
 #endif

  //layer 15
 #ifndef TIME_RUN
  uart_printf("\nRunning layer 15...\n");
  start = timer_time_us(TIMER_BASE);
 #endif
  conv2(LAYER_15_W, LAYER_14_NUM_KER, LAYER_15_NUM_KER, LAYER_15_KER_SIZE, LAYER_15_OUTPADD, LAYER_15_STRIDE, LAYER_15_PINGPONG, LAYER_14_OUTPADD, LAYER_15_ZXY, LAYER_15_LEAKY, LAYER_15_IGNOREPAD, 0, LAYER_15_UPSAMPLE); //outpos(0)
 #ifndef TIME_RUN
  end = timer_time_us(TIMER_BASE);
  uart_printf("Convolution done in %d us\n\n", end-start);
  total_time += end-start;
 #endif

  //layer 16 and 17
 #ifndef TIME_RUN
  uart_printf("\nRunning layers 16 and 17...\n");
  start = timer_time_us(TIMER_BASE);
 #endif
  conv2(LAYER_16_W, LAYER_15_NUM_KER, LAYER_16_NUM_KER, LAYER_16_KER_SIZE, LAYER_16_OUTPADD, LAYER_16_STRIDE, LAYER_16_PINGPONG, LAYER_15_OUTPADD, LAYER_16_ZXY, LAYER_16_LEAKY, LAYER_16_IGNOREPAD, 0, LAYER_16_UPSAMPLE); //outpos(0)
 #ifndef TIME_RUN
  end = timer_time_us(TIMER_BASE);
  uart_printf("Convolution + yolo done in %d us\n\n", end-start);
  total_time += end-start;
 #endif

  //layers 19 and 20
 #ifndef TIME_RUN
  uart_printf("\nRunning layers 19 and 20...\n");
  start = timer_time_us(TIMER_BASE);
 #endif
  p_pos = data_pos_layer14;
  conv2(LAYER_19_W, LAYER_14_NUM_KER, LAYER_19_NUM_KER, LAYER_19_KER_SIZE, LAYER_19_OUTPADD, LAYER_19_STRIDE, LAYER_19_PINGPONG, 0, LAYER_19_ZXY, LAYER_19_LEAKY, LAYER_19_IGNOREPAD, data_pos_layer19, LAYER_19_UPSAMPLE); //inpadd(0)
 #ifndef TIME_RUN
  end = timer_time_us(TIMER_BASE);
  uart_printf("Convolution + upsample done in %d us\n\n", end-start);
  total_time += end-start;
 #endif

  //layer 22
 #ifndef TIME_RUN
  uart_printf("\nRunning layer 22...\n");
  start = timer_time_us(TIMER_BASE);
 #endif
  conv2(LAYER_22_W, LAYER_9_NUM_KER+LAYER_19_NUM_KER, LAYER_22_NUM_KER, LAYER_22_KER_SIZE, LAYER_22_OUTPADD, LAYER_22_STRIDE, LAYER_22_PINGPONG, LAYER_19_OUTPADD, LAYER_22_ZXY, LAYER_22_LEAKY, LAYER_22_IGNOREPAD, 0, LAYER_22_UPSAMPLE); //outpos(0)
 #ifndef TIME_RUN
  end = timer_time_us(TIMER_BASE);
  uart_printf("Convolution done in %d us\n\n", end-start);
  total_time += end-start;
 #endif

#endif

  //layers 23 and 24
 #ifndef TIME_RUN
  uart_printf("\nRunning layers 23 and 24...\n");
  start = timer_time_us(TIMER_BASE);
 #endif
  conv2(LAYER_23_W, LAYER_22_NUM_KER, LAYER_23_NUM_KER, LAYER_23_KER_SIZE, LAYER_23_OUTPADD, LAYER_23_STRIDE, LAYER_23_PINGPONG, LAYER_22_OUTPADD, LAYER_23_ZXY, LAYER_23_LEAKY, LAYER_23_IGNOREPAD, 0, LAYER_23_UPSAMPLE); //outpos(0)
  // end versat
 #ifdef TIME_RUN
  while(versat.done()==0);
  end = (unsigned int) timer_get_count(TIMER_BASE);
  uart_printf("%d\n", (end - start)*8);
  versat.run();
  start = (unsigned int) timer_get_count(TIMER_BASE);
  while(versat.done()==0);
  end = (unsigned int) timer_get_count(TIMER_BASE);
  uart_printf("%d\n", (end - start)*8);
  versat.run();
  start = (unsigned int) timer_get_count(TIMER_BASE);
  while(versat.done()==0);
  end = (unsigned int) timer_get_count(TIMER_BASE);
  uart_printf("%d\n", (end - start)*8);
 #else
  versat_end();
  end = timer_time_us(TIMER_BASE);
  uart_printf("Convolution + yolo done in %d us\n\n", end-start);
  total_time += end-start;
  uart_printf("\n\n TOTAL_TIME = %d us\n\n", total_time);
 #endif

#ifndef SIM
  //print detected objects and corresponding probability scores
  uart_printf("\nResults of first yolo layer:\n");
  print_results(LAYER_16_W, data_pos_layer19-DATA_LAYER_16);
  uart_printf("\nResults of second yolo layer:\n");
  print_results(LAYER_23_W, p_pos);
#endif

#ifdef SIM
  int16_t * fp_data = (int16_t *) DATA_BASE_ADDRESS;
  fp_data += DATA_LAYER_22;
  int i, j, k;
  uart_printf("\nVerifying...\n");
  for(i = 0; i < LAYER_23_W; i++) {
    uart_printf("Line %d base addr = %x\n", i, DATA_BASE_ADDRESS + 2*(DATA_LAYER_22 + i*LAYER_23_W*LAYER_23_NUM_KER));
    for(j = 0; j < LAYER_23_W; j++)
      for(k = 0; k < LAYER_23_NUM_KER; k++)
        if(fp_data[i*LAYER_23_W*LAYER_23_NUM_KER + j*LAYER_23_NUM_KER + k] != fp_data[DATA_LAYER_23 + i*LAYER_23_W*LAYER_23_NUM_KER + j*LAYER_23_NUM_KER + k])
          uart_printf("(%x) res = %x, act = %x\n", DATA_BASE_ADDRESS + 2*(DATA_LAYER_22 + i*LAYER_23_W*LAYER_23_NUM_KER + j*LAYER_23_NUM_KER + k), fp_data[i*LAYER_23_W*LAYER_23_NUM_KER + j*LAYER_23_NUM_KER + k] & 0xFFFF, fp_data[DATA_LAYER_23 + i*LAYER_23_W*LAYER_23_NUM_KER + j*LAYER_23_NUM_KER + k] & 0xFFFF);
  }
#endif

#ifndef SIM
  send_data();
#endif

  //finish
  uart_putc(ETX);
  uart_txwait();
  return 0;
}
