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

//print time of each run
//#define TIME_RUN
#define CLK_NS 8

//layer 1 padding
#if nYOLOmacs == 1
  #define IMG_C 3
  #define LAYER_1_P_OFF 10
  #define C_PADD 0
  #define NEW_W_PADD 416
  #define ix_PADD 0
#elif nYOLOmacs == 4
  #define IMG_C 4
  #define LAYER_1_P_OFF 8
  #define C_PADD 1
  #define NEW_W_PADD 420
  #define ix_PADD 4
#endif

//input image constants
#define IMG_W 768
#define IMG_H 576
#define NEW_W 416
#define NEW_H ((IMG_H*NEW_W)/IMG_W)	//312
#define EXTRA_H ((NEW_W-NEW_H)/2) 	//52
#define IMAGE_INPUT (IMG_W*IMG_H*IMG_C) //already 32-byte aligned

//Padding to width
#define NETWORK_INPUT_AUX (NEW_W_PADD*IMG_H*IMG_C)
#define DATA_LAYER_1 ((NEW_W+2)*((NEW_W+2)*IMG_C+LAYER_1_P_OFF))

//Padding to height
#define IMG_H_PADD (IMG_H+9) //(576+9)/13 = 45
#define NETWORK_INPUT_AUX_PADD (NEW_W_PADD*IMG_H_PADD*IMG_C)

//resize constants
#define w_scale ((float)(IMG_W-1)/(NEW_W-1))
#define h_scale ((float)(IMG_H-1)/(NEW_H-1))
#define ix_size ((NEW_W_PADD+ix_PADD)*2)
#define dx_size (NEW_W_PADD*2*nYOLOmacs)
#define dy_size (NEW_H*2*nYOLOmacs)
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
#define INPUT_FILE_SIZE (IMAGE_INPUT*2) //16 bits
#define NUM_INPUT_FRAMES (INPUT_FILE_SIZE/ETH_NBYTES)
#define OUTPUT_FILE_SIZE (DATA_LAYER_1*2)
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
  int i, j, val_i, val_i_first;
  float val, val_d;
  int16_t * ix = (int16_t *) ix_BASE_ADDRESS;
  int16_t * dx = (int16_t *) dx_BASE_ADDRESS;
  int16_t * dy = (int16_t *) dy_BASE_ADDRESS;

  //loop to initialize ix and dx
#ifndef SIM
  for(i = 0; i < NEW_W; i++) {
    val = i*w_scale;
    val_i = (int) val;
    val_d = val - val_i;
    if(i == 0) {
      //add 2 extra initial positions
      val_i_first = val_i;
      ix[0] = val_i;
      ix[1] = val_i;
      for(j = 0; j < 2*nYOLOmacs; j++) dx[j] = 0;
    }
    //normal positions
    ix[2+2*i] = val_i;
    for(j = 0; j < nYOLOmacs; j++) {
      dx[2*nYOLOmacs + 2*nYOLOmacs*i + j] = (int16_t)((1-val_d)*((int16_t)1<<14)); //Q2.14
      dx[2*nYOLOmacs + 2*nYOLOmacs*i + nYOLOmacs + j] = (int16_t)(val_d*((int16_t)1<<14)); //Q2.14
    }
    //add 7 extra final positions for ix
    if(i == NEW_W-1) for(j = 0; j < 8; j++) ix[2+2*i+1+j] = val_i_first;
    else ix[2+2*i+1] = val_i + 1;
  }
  //add 3 extra final positions for dx
  for(j = 0; j < 2*nYOLOmacs*3; j++) dx[2*nYOLOmacs*(NEW_W+1)  + j] = 0;
#endif

  //loop to initialize iy and dy
#ifdef SIM
  for(i = 0; i < 10; i++) {
    uart_printf("%d\n", i);
#else
  for(i = 0; i < NEW_H; i++) {
#endif
    val = i*h_scale;
    iy[i] = (int) val;
  #ifndef SIM
    val_d = val - iy[i];
    //dy
    for(j = 0; j < nYOLOmacs; j++) {
      dy[2*nYOLOmacs*i + j] = (int16_t)((1-val_d)*((int16_t)1<<14)); //Q2.14
      dy[2*nYOLOmacs*i + nYOLOmacs + j] = (int16_t)(val_d*((int16_t)1<<14)); //Q2.14
    }
  #endif
  }

  //configure ix vread to read ix from DDR
  versat.dma.ywrite_read_setLen(2*(NEW_W_PADD+ix_PADD)/16-1);
  versat.ywrite.read.setIxExtAddr(ix_BASE_ADDRESS);
  versat.ywrite.read.setIxExtIter(1);
  versat.ywrite.read.setIxExtPer(2*(NEW_W_PADD+ix_PADD)/16);
  versat.ywrite.read.setIxExtIncr(16);
  versat.run();
  while(versat.done()==0);
  versat.clear();
}

//reset certain DDR positions to zero due to padding
void reset_DDR() {

  //local variables
  int i;
  int16_t * fp_data = (int16_t *) DATA_BASE_ADDRESS + NETWORK_INPUT_AUX_PADD;

  //measure initial time
  uart_printf("\nSetting DDR positions to zero\n");
  start = timer_time_us(TIMER_BASE);

  //input network
  for(i = 0; i < DATA_LAYER_1; i++) fp_data[i] = 0;

  //measure final time
  end = timer_time_us(TIMER_BASE);
  uart_printf("DDR reset to zero done in %d ms\n", (end-start)/1000);
}

//fill grey CNN input image (except padding)
void fill_grey() {
  int i, j, k;
  int16_t * fp_data = (int16_t *) DATA_BASE_ADDRESS;
  //pass first padding line and padding column
  fp_data += NETWORK_INPUT_AUX_PADD + (NEW_W+2)*IMG_C + LAYER_1_P_OFF + IMG_C;
  for(j = 0; j < NEW_W; j++)
    for(k = 0; k < NEW_W; k++) 
      for(i = 0; i < IMG_C-C_PADD; i++)
	fp_data[j*((NEW_W+2)*IMG_C + LAYER_1_P_OFF) + k*IMG_C + i] = 0x4000; //0.5 in Q1.15
}

//receive weigths and resized padded image
void rcv_data() {

  //Local variables
  int i, j;
  count_bytes = 0;
  char * data_p = (char *) INPUT_IMAGE_BASE_ADDRESS;
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
  uart_printf("Data received in %d ms\n", (end-start)/1000);
}

//width resize -> 768 to 416
void width_resize() {

  //local variables
  int i;

  /////////////////////////////////////////////////////////////////////////
  //                          FIXED CONFIGURATIONS
  /////////////////////////////////////////////////////////////////////////

  // configure xyolo_read vreads to read 1-dx/dx from DDR
  versat.dma.yread_setLen(2*NEW_W_PADD*nYOLOmacs/16-1);
  versat.yread.setPingPong(1);
  versat.yread.setExtPer(2*NEW_W_PADD*nYOLOmacs/16);
  versat.yread.setExtIncr(16);
  versat.yread.setExtIter(1);
  versat.yread.setExtAddr(dx_BASE_ADDRESS);

  // configure xyolo_write vread to read line from DDR
  versat.dma.ywrite_read_setLen(IMG_W*IMG_C*nSTAGES/16-1);
  versat.ywrite.read.setPingPong(1);
  versat.ywrite.read.setOffset(2*(IMG_W*IMG_C));
  versat.ywrite.read.setExtPer(IMG_W*IMG_C/16);
  versat.ywrite.read.setExtIncr(16);
  versat.ywrite.read.setExtIter(1);

  // configure xyolo_read vreads to write 1-dx/dx sequence to xyolo
  versat.yread.setIntDelay(1); //due to reading ix/ix+1 from mem
  versat.yread.setIntPer(2*NEW_W_PADD);
  versat.yread.setIntIncr(1);
  versat.yread.setIntIter(1);

  // configure xyolo_write vreads to write ix/ix+1 sequence pixels to xyolo
  versat.ywrite.read.setExt(1);
  versat.ywrite.read.setIntPer(2*NEW_W_PADD);
  versat.ywrite.read.setIntIncr(1);
  versat.ywrite.read.setIntIter(1);
  
  // configure xyolo to multiply pixel with dx
  versat.ywrite.yolo.setIter(NEW_W_PADD);
  versat.ywrite.yolo.setPer(2);
  versat.ywrite.yolo.setShift(7);
  versat.ywrite.yolo.setBypassAdder(1);

  // configure xwrite to write results
  versat.ywrite.write.setIntDuty(2*nYOLOvect/IMG_C);
  versat.ywrite.write.setIntDelay(XYOLO_READ_LAT + XYOLO_WRITE_LAT - 4);
  versat.ywrite.write.setIntPer(2*nYOLOvect/IMG_C);
  versat.ywrite.write.setIntIter(NEW_W_PADD/(nYOLOvect/IMG_C));
  versat.ywrite.write.setIntShift(1);

  // configure xyolo_write vwrite to write result back to DDR
  versat.dma.ywrite_write_setLen(NEW_W_PADD*IMG_C/nYOLOvect-1);
  versat.ywrite.write.setOffset(2*(NEW_W_PADD*IMG_C));
  versat.ywrite.write.setExtPer(NEW_W_PADD*IMG_C/nYOLOvect);
  versat.ywrite.write.setExtIter(1);

  /////////////////////////////////////////////////////////////////////////
  //                      VARIABLE CONFIGURATIONS
  /////////////////////////////////////////////////////////////////////////

#ifdef SIM
  for(i = 0; i < 2; i++) {
#else
  for(i = 0; i < IMG_H_PADD/nSTAGES; i++) {
#endif

    // configure xyolo_write vread to read lines from input fm
    versat.ywrite.read.setExtAddr(INPUT_IMAGE_BASE_ADDRESS + 2*i*IMG_W*IMG_C*nSTAGES);

    // configure xyolo_write vwrite to write result back to DDR
    versat.ywrite.write.setExtAddr(DATA_BASE_ADDRESS + 2*i*NEW_W_PADD*IMG_C*nSTAGES);

    // wait until done
    while(versat.done()==0);
  #ifdef TIME_RUN
    end = (unsigned int) timer_get_count(TIMER_BASE);
    if(i != 0) uart_printf("%d\n", (end - start)*CLK_NS);
  #endif

    // run configuration
    versat.run();
  #ifdef TIME_RUN
    start = (unsigned int) timer_get_count(TIMER_BASE);
  #endif

    // stop xyolo_read vread reading from DDR
    versat.yread.setExtIter(0);
  }

  // clear configs
  versat.clear();
}

//height resize -> 576 to 312
void height_resize() {
  
  //local variables
  int i;
  //pass extra and first padding lines and padding column
  unsigned int p_out = DATA_BASE_ADDRESS + 2*(NETWORK_INPUT_AUX_PADD + NEW_W_PADD*IMG_C*(EXTRA_H+1));

  /////////////////////////////////////////////////////////////////////////
  //                          FIXED CONFIGURATIONS
  /////////////////////////////////////////////////////////////////////////

  // configure xyolo_read vreads to read 1-dy/dy from DDR
  versat.dma.yread_setLen(2*NEW_H*nYOLOmacs/16-1);
  versat.yread.setPingPong(1);
  versat.yread.setExtPer(2*NEW_H*nYOLOmacs/16);
  versat.yread.setExtIncr(16);
  versat.yread.setExtIter(1);
  versat.yread.setExtAddr(dy_BASE_ADDRESS);

  // configure xyolo_write vread to read line from DDR
  versat.dma.ywrite_read_setLen(NEW_W_PADD*IMG_C*2/16-1); //2 lines
  versat.ywrite.read.setPingPong(1);
  versat.ywrite.read.setExtPer(NEW_W_PADD*IMG_C*2/16);
  versat.ywrite.read.setExtIncr(16);
  versat.ywrite.read.setExtIter(1);

  // configure xyolo_read vreads to write 1-dy/dy sequence to xyolo
  versat.yread.setIntPer(2);
  versat.yread.setIntIncr(1);
  versat.yread.setIntIter(NEW_W_PADD);
  versat.yread.setIntShift(-2);

  // configure xyolo_write vreads to write pixels to xyolo
  versat.ywrite.read.setIntPer(2);
  versat.ywrite.read.setIntIncr(NEW_W_PADD);
  versat.ywrite.read.setIntIter(NEW_W_PADD);
  versat.ywrite.read.setIntShift(-2*NEW_W_PADD+1);

  // configure xyolo to multiply pixel with dy
  versat.ywrite.yolo.setIter(NEW_W_PADD);
  versat.ywrite.yolo.setPer(2);
  versat.ywrite.yolo.setShift(14);
  versat.ywrite.yolo.setBypassAdder(1);

  // configure xwrite to write results
  versat.ywrite.write.setIntDuty(2*nYOLOvect/IMG_C);
  versat.ywrite.write.setIntDelay(XYOLO_READ_LAT + XYOLO_WRITE_LAT - 5);
  versat.ywrite.write.setIntPer(2*nYOLOvect/IMG_C);
  versat.ywrite.write.setIntIter(NEW_W_PADD/(nYOLOvect/IMG_C));
  versat.ywrite.write.setIntShift(1);

  // configure xyolo_write vwrite to write result back to DDR
  versat.dma.ywrite_write_setLen(NEW_W_PADD*IMG_C/nYOLOvect-1);
  versat.ywrite.write.setExtPer(NEW_W_PADD*IMG_C/nYOLOvect);
  versat.ywrite.write.setExtIter(1);

  /////////////////////////////////////////////////////////////////////////
  //                      VARIABLE CONFIGURATIONS
  /////////////////////////////////////////////////////////////////////////

#ifdef SIM
  for(i = 0; i < 10; i++) {
#else
  for(i = 0; i < NEW_H; i++) {
#endif

    // configure xyolo_read vread start
    versat.yread.setIntStart(2*i);

    // configure xyolo_write vread to read lines from input fm
    versat.ywrite.read.setExtAddr(DATA_BASE_ADDRESS + 2*iy[i]*NEW_W_PADD*IMG_C);

    // configure xyolo_write vwrite to write result back to DDR
    versat.ywrite.write.setExtAddr(p_out + 2*i*NEW_W_PADD*IMG_C);

    // wait until done
    while(versat.done()==0);
  #ifdef TIME_RUN
    end = (unsigned int) timer_get_count(TIMER_BASE);
    uart_printf("%d\n", (end - start)*CLK_NS);
    if(i == 0) uart_printf("\n");
  #endif

    // run configuration
    versat.run();
  #ifdef TIME_RUN
    start = (unsigned int) timer_get_count(TIMER_BASE);
  #endif

    // stop xyolo_read vread reading from DDR
    versat.yread.setExtIter(0);
  }

  // clear configs
  versat.clear();
}

//send results back
void send_data() {

  //Loop to send data
  int i, j;
  count_bytes = 0;
  char * fp_data_char = (char *) DATA_BASE_ADDRESS + 2*NETWORK_INPUT_AUX_PADD;
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

#ifndef SIM
  //init ETHERNET
  eth_init(ETHERNET_BASE);
  eth_set_rx_payload_size(ETH_NBYTES);
#endif

  //init VERSAT
  versat_init(VERSAT_BASE);

  //fill CNN input with grey
#ifndef SIM
  reset_DDR();
  rcv_data();
  fill_grey();
#endif

  //initialize ix, iy, dx and dy arrays
  uart_printf("\nPreparing resize...\n");
  start = timer_time_us(TIMER_BASE);
  prepare_resize();
  end = timer_time_us(TIMER_BASE);
  uart_printf("Done in %d us\n\n", end-start);

#ifndef SIM

  //width resize
 #ifndef TIME_RUN
  uart_printf("\nWidth resizing...\n");
  start = timer_time_us(TIMER_BASE);
 #endif
  width_resize();
 #ifndef TIME_RUN
  end = timer_time_us(TIMER_BASE);
  uart_printf("Done in %d us\n\n", end-start);
 #endif

#endif

  //height resize
 #ifndef TIME_RUN
  uart_printf("\nHeight resizing...\n");
  start = timer_time_us(TIMER_BASE);
 #endif
  height_resize();
  //end versat
 #ifdef TIME_RUN
  while(versat.done()==0);
  end = (unsigned int) timer_get_count(TIMER_BASE);
  uart_printf("%d\n", (end - start)*CLK_NS);
  versat.run();
  start = (unsigned int) timer_get_count(TIMER_BASE);
  while(versat.done()==0);
  end = (unsigned int) timer_get_count(TIMER_BASE);
  uart_printf("%d\n", (end - start)*CLK_NS);
  versat.run();
  start = (unsigned int) timer_get_count(TIMER_BASE);
  while(versat.done()==0);
  end = (unsigned int) timer_get_count(TIMER_BASE);
  uart_printf("%d\n", (end - start)*CLK_NS);
 #else
  versat_end();
  end = timer_time_us(TIMER_BASE);
  uart_printf("Done in %d us\n\n", end-start);
 #endif

#ifdef SIM
  //verify results
  int16_t * fp_data = (int16_t *) DATA_BASE_ADDRESS;
  fp_data += NETWORK_INPUT_AUX_PADD + ((NEW_W+2)*IMG_C+LAYER_1_P_OFF)*(EXTRA_H+1);
  int i, j, k;
  uart_printf("\nVerifying...\n");
  for(i = 0; i < 10; i++) {
    uart_printf("Line %d\n", i);
    for(j = 0; j < NEW_W+2; j++)
      for(k = 0; k < IMG_C; k++)
        if(fp_data[i*(NEW_W+2)*IMG_C + j*IMG_C + k] != fp_data[DATA_LAYER_1 + i*(NEW_W+2)*IMG_C + j*IMG_C + k])
	  uart_printf("(%d) res = %x, act = %x\n", i*(NEW_W+2)*IMG_C + j*IMG_C + k, fp_data[i*(NEW_W+2)*IMG_C + j*IMG_C + k] & 0xFFFF, fp_data[DATA_LAYER_1 + i*(NEW_W+2)*IMG_C + j*IMG_C + k] & 0xFFFF);   
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
