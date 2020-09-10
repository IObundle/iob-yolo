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
#define CLK_NS 8
#ifdef SIM
  int k_delta;
#endif

//resize constants
#define w_scale ((float)(IMG_W-1)/(NEW_W-1))
#define h_scale ((float)(IMG_H-1)/(NEW_H-1))
#define ix_size ((NEW_W_PADD+ix_PADD)*2)
#define dx_size (NEW_W_PADD*2*nYOLOmacs)
#define dy_size (NEW_H*2*nYOLOmacs)
int16_t iy[NEW_H];

//obj and prob score threshold
#define threshold ((int16_t)(((float)0.5)*((int32_t)1<<8))) //Q8.8
#define nms_threshold ((int16_t)(((float)0.45)*((int32_t)1<<14))) //Q2.14
#define yolo1_div ((int16_t)(((float)1/LAYER_16_W)*((int32_t)1<<15))) //Q1.15
#define yolo2_div ((int16_t)(((float)1/LAYER_23_W)*((int32_t)1<<15))) //Q1.15
#define y_scales ((int16_t)(((float)NEW_W/NEW_H)*((int32_t)1<<14))) //Q2.14
#define y_bias ((int16_t)(((float)(NEW_W-NEW_H)/(NEW_H*2))*((int32_t)1<<14))) //Q2.14
#define w_scales ((int16_t)(((float)1/NEW_W)*((int32_t)1<<14))) //Q2.14
#define h_scales ((int16_t)(((float)1/NEW_H)*((int32_t)1<<14))) //Q2.14
#define c3 ((int16_t)0x0AAA) // pow(2,-3)+pow(2,-5)+pow(2,-7)+pow(2,-9)+pow(2,-11)+pow(2,-13) in Q2.14
#define c4 ((int16_t)0x02C0) // pow(2,-5)+pow(2,-7)+pow(2,-8) in Q2.14
#define MAX_NUM_BOXES 10
#define box_width 3
#define label_height 20

//label constants
#define MAX_LABEL_SIZE 2340
#define LABELS_FILE_SIZE (81 + MAX_LABEL_SIZE*81 + 11) //+11 to be 32-byte aligned

//variables for bounding boxes
int16_t boxes[84*MAX_NUM_BOXES];
uint8_t nboxes = 0;
uint8_t box_IDs[MAX_NUM_BOXES];

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
#define INPUT_FILE_SIZE (LABELS_FILE_SIZE + (TOTAL_WEIGHTS + IMAGE_INPUT)*2) //16 bits
#define NUM_INPUT_FRAMES (INPUT_FILE_SIZE/ETH_NBYTES)
#define OUTPUT_FILE_SIZE (IMAGE_INPUT) //8 bits
#define NUM_OUTPUT_FRAMES (OUTPUT_FILE_SIZE/ETH_NBYTES)

//define DDR mapping
#define ix_BASE_ADDRESS (DDR_MEM + (1 << (FIRM_ADDR_W))) //after main mem
#define dx_BASE_ADDRESS (ix_BASE_ADDRESS + 2*ix_size)
#define dy_BASE_ADDRESS (dx_BASE_ADDRESS + 2*dx_size)
#define WEIGHTS_BASE_ADDRESS (dy_BASE_ADDRESS + 2*dy_size)
#define LABEL_BASE_ADDRESS (WEIGHTS_BASE_ADDRESS + 2*TOTAL_WEIGHTS) //16 bits
#define INPUT_IMAGE_BASE_ADDRESS (LABEL_BASE_ADDRESS + LABELS_FILE_SIZE)
#define RESIZED_IMAGE_BASE_ADDRESS (INPUT_IMAGE_BASE_ADDRESS + 2*IMAGE_INPUT)
#ifdef SIM
  #define DATA_BASE_ADDRESS (DDR_MEM + (1 << (FIRM_ADDR_W))) //after main mem
#else
  #define DATA_BASE_ADDRESS (RESIZED_IMAGE_BASE_ADDRESS + 2* NETWORK_INPUT_AUX)
#endif

//ETHERNET variables
int rcv_timeout = 5000;
char data_to_send[ETH_NBYTES];
char data_rcv[ETH_NBYTES+18];
unsigned int bytes_to_receive, bytes_to_send, count_bytes;

//TIMER variables
unsigned int start, end;

//weights and data updatable initial positions
unsigned int w_pos = 0, p_pos = 0;

//input image pointer
uint8_t * fp_labels = (uint8_t *) LABEL_BASE_ADDRESS;
int16_t * fp_image = (int16_t *) INPUT_IMAGE_BASE_ADDRESS;

//reset certain DDR positions to zero due to padding
void reset_DDR() {

  //local variables
  unsigned int i, pos = DATA_LAYER_1;
  int16_t * fp_data = (int16_t *) DATA_BASE_ADDRESS;

  //measure initial time
  uart_printf("\nSetting DDR positions to zero\n");
  start = timer_time_us(TIMER_BASE);

  //input network
  for(i = 0; i < DATA_LAYER_1; i++) fp_data[i] = 0;

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

//fill grey CNN input image (except padding)
void fill_grey() {
  int i, j, k;
  int16_t * fp_data = (int16_t *) DATA_BASE_ADDRESS;
  //pass first padding line and padding column
  fp_data += (NEW_W+2)*LAYER_1_C + LAYER_1_P_OFF + LAYER_1_C;
  for(j = 0; j < NEW_W; j++)
    for(k = 0; k < NEW_W; k++)
      for(i = 0; i < IMG_C; i++)
        fp_data[j*((NEW_W+2)*LAYER_1_C + LAYER_1_P_OFF) + k*LAYER_1_C + i] = 0x0080;
}

//initialize ix, iy, dx and dy arrays
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
  versat.dma.ywrite_read_setLen(IMG_W*LAYER_1_C*nSTAGES/16-1);
  versat.ywrite.read.setPingPong(1);
  versat.ywrite.read.setOffset(2*(IMG_W*LAYER_1_C));
  versat.ywrite.read.setExtPer(IMG_W*LAYER_1_C/16);
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
  versat.ywrite.write.setIntDuty(2*nYOLOvect/LAYER_1_C);
  versat.ywrite.write.setIntDelay(XYOLO_READ_LAT + XYOLO_WRITE_LAT - 2);
  versat.ywrite.write.setIntPer(2*nYOLOvect/LAYER_1_C);
  versat.ywrite.write.setIntIter(NEW_W_PADD/(nYOLOvect/LAYER_1_C));
  versat.ywrite.write.setIntShift(1);

  // configure xyolo_write vwrite to write result back to DDR
  versat.dma.ywrite_write_setLen(NEW_W_PADD*LAYER_1_C/nYOLOvect-1);
  versat.ywrite.write.setOffset(2*(NEW_W_PADD*LAYER_1_C));
  versat.ywrite.write.setExtPer(NEW_W_PADD*LAYER_1_C/nYOLOvect);
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
    versat.ywrite.read.setExtAddr(INPUT_IMAGE_BASE_ADDRESS + 2*i*IMG_W*LAYER_1_C*nSTAGES);

    // configure xyolo_write vwrite to write result back to DDR
    versat.ywrite.write.setExtAddr(RESIZED_IMAGE_BASE_ADDRESS + 2*i*NEW_W_PADD*LAYER_1_C*nSTAGES);

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
  unsigned int p_out = DATA_BASE_ADDRESS + 2*NEW_W_PADD*LAYER_1_C*(EXTRA_H+1);

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
  versat.dma.ywrite_read_setLen(NEW_W_PADD*LAYER_1_C*2/16-1); //2 lines
  versat.ywrite.read.setPingPong(1);
  versat.ywrite.read.setExtPer(NEW_W_PADD*LAYER_1_C*2/16);
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
  versat.ywrite.yolo.setShift(21);
  versat.ywrite.yolo.setBypassAdder(1);

  // configure xwrite to write results
  versat.ywrite.write.setIntDuty(2*nYOLOvect/LAYER_1_C);
  versat.ywrite.write.setIntDelay(XYOLO_READ_LAT + XYOLO_WRITE_LAT - 3);
  versat.ywrite.write.setIntPer(2*nYOLOvect/LAYER_1_C);
  versat.ywrite.write.setIntIter(NEW_W_PADD/(nYOLOvect/LAYER_1_C));
  versat.ywrite.write.setIntShift(1);

  // configure xyolo_write vwrite to write result back to DDR
  versat.dma.ywrite_write_setLen(NEW_W_PADD*LAYER_1_C/nYOLOvect-1);
  versat.ywrite.write.setExtPer(NEW_W_PADD*LAYER_1_C/nYOLOvect);
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
    versat.ywrite.read.setExtAddr(RESIZED_IMAGE_BASE_ADDRESS + 2*iy[i]*NEW_W_PADD*LAYER_1_C);

    // configure xyolo_write vwrite to write result back to DDR
    versat.ywrite.write.setExtAddr(p_out + 2*i*NEW_W_PADD*LAYER_1_C);

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
  versat.dma.yread_setLen(LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*LAYER_1_C + LAYER_1_W_OFF);
  versat.yread.setOffset(2*(LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*LAYER_1_C + LAYER_1_W_OFF));
  versat.yread.setExtPer((LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*LAYER_1_C + LAYER_1_W_OFF)/16);
  versat.yread.setExtIncr(16);
  versat.yread.setPingPong(1);

  // configure xyolo_write vread to read full line
  versat.dma.ywrite_read_setLen((LAYER_1_C*(LAYER_1_W+2)+LAYER_1_P_OFF)*(nSTAGES*2+2)/16-1);
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
  versat.dma.ywrite_write_setLen(LAYER_1_TILE_W/2-1);
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
    versat.yread.setBiasExtIter(1);
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
        uart_printf("%d\n", (end - start)*CLK_NS);
        if(l == 0 && k == 0 && j == 0) uart_printf("\n");
      #endif

        // run configuration
        versat.run();
      #ifdef TIME_RUN
        start = (unsigned int) timer_get_count(TIMER_BASE);
      #endif

        // stop reading from DDR
        versat.ywrite.read.setExtIter(0);
        versat.yread.setExtIter(0);
        versat.yread.setBiasExtIter(0);
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
  versat.dma.yread_setLen(ker_size*ker_size*c);
  versat.yread.setOffset(2*(ker_size*ker_size*c));
  versat.yread.setExtPer((ker_size*ker_size*c)/16);
  versat.yread.setExtIncr(16);
  versat.yread.setExtIter(1);
  versat.yread.setBiasExtIter(1);

  // configure xyolo_write vread to read tile from input fm
  versat.dma.ywrite_read_setLen((c*(til_w+2))/16-1);
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
  versat.dma.ywrite_write_setLen(((til_w/2)*(num_ker/nYOLOvect))-1);
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
        uart_printf("%d\n", (end - start)*CLK_NS);
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
      versat.yread.setBiasExtIter(0);
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
    uart_printf("%d\n", (end - start)*CLK_NS);
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
  versat.dma.yread_setLen(ker_size*ker_size*c);
  versat.yread.setOffset(2*(ker_size*ker_size*c));
  versat.yread.setExtPer((ker_size*ker_size*c)/16);
  versat.yread.setExtIncr(16);
  versat.yread.setExtIter(1);
  versat.yread.setBiasExtIter(1);
  versat.yread.setPingPong(1);

  // configure xyolo_write vread to read input fm
  versat.dma.ywrite_read_setLen(c*(w+2*inpadd+2*ignorepadd)*(nSTAGES+2*inpadd)/16-1);
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
  versat.ywrite.write.setIntDelay(XYOLO_READ_LAT + XYOLO_WRITE_LAT - 2 + 2*(1-leaky));
  versat.ywrite.write.setIntPer(ker_size*ker_size*c/nYOLOmacs);
  versat.ywrite.write.setIntIncr(1);
  versat.ywrite.write.setIntIter(w*(1+upsample)+upsample); //+upsample to use duty of 2

  // configure xyolo_write vwrite to write result back to DDR
  versat.dma.ywrite_write_setLen((1+upsample)*w-1);
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
      versat.yread.setBiasExtIter(0);
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
      uart_printf("%d\n", (end - start)*CLK_NS);
    #endif
      versat.run();
    #ifdef TIME_RUN
      start = (unsigned int) timer_get_count(TIMER_BASE);
    #endif

      //recover former configs
      versat.yread.setExtIter(1);
      versat.yread.setBiasExtIter(1);
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
      uart_printf("%d\n", (end - start)*CLK_NS);
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
  uart_printf("%d\n", (end - start)*CLK_NS);
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
    versat.dma.ywrite_read_setLen(c*(w+2*inpadd+stride)*(nSTAGES*(1+inpadd)+stride)/16-1);
    versat.ywrite.read.setOffset(2*((1+inpadd)*(w+2*inpadd+stride)*c)); 
    versat.ywrite.read.setExtPer((c*(w+2*inpadd+stride)*2)/16); //each vread reads 2 lines
    versat.ywrite.read.setExtIter(1);
  } else {
    versat.dma.ywrite_read_setLen(c*(w+2*inpadd+stride)/16-1);
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
  versat.ywrite.read.setIntPer2(nYOLOmacs);
  versat.ywrite.read.setIntIter2(1);
  versat.ywrite.read.setIntIncr3(1);
  versat.ywrite.read.setIntPer3(nYOLOvect/nYOLOmacs);
  versat.ywrite.read.setIntIter3(c/16);
  versat.ywrite.read.setIntShift3((w+2*inpadd+stride)*16/nYOLOmacs - nYOLOvect/nYOLOmacs);

  // configure xyolo to perform maxpool
  versat.ywrite.yolo.setIter(c);
  versat.ywrite.yolo.setPer(4);
  versat.ywrite.yolo.setMaxpool(1);
  versat.ywrite.yolo.setBypass(1);

  // configure xwrite to write maxpool results
  // hw internally has counter to adress each vwrite individually when bypass is enabled
  versat.ywrite.write.setIntDuty(4*16);
  versat.ywrite.write.setIntDelay(XYOLO_READ_LAT + XYOLO_WRITE_BYPASS_LAT - 2);
  versat.ywrite.write.setIntPer(4*16);
  versat.ywrite.write.setIntIter(c/16);
  versat.ywrite.write.setIntShift(1);

  // configure xyolo_write vwrite to write result back to DDR
  versat.dma.ywrite_write_setLen(c/16-1);
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
    uart_printf("%d\n", (end - start)*CLK_NS);
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

//Calculate overlapp between 2 boxes
int16_t overlap(int16_t x1, int16_t w1, int16_t x2, int16_t w2) {
  int16_t l1, l2, left, r1, r2, right;
  l1 = x1 - (w1>>1);
  l2 = x2 - (w2>>1);
  left = l1 > l2 ? l1 : l2;
  r1 = x1 + (w1>>1);
  r2 = x2 + (w2>>1);
  right = r1 < r2 ? r1 : r2;
  return right - left;
}

//Apply non-maximum-suppresion to filter repeated boxes
void filter_boxes() {

  //Local variables
  int i, j, k, l;
  int obj_cnt;
  int16_t w, h, b_union, b_iou;
  int16_t x1, y1, w1, h1, x2, y2, w2, h2;
  int32_t mul_32, b_inter;

  //Loop to go through classes from candidate boxes
  for(i = 0; i < 80; i++) {

    //Count number of candidate boxes for given class
    obj_cnt = 0;
    for(j = 0; j < nboxes; j++) {
      if(boxes[84*j+4+i] != 0) {

        //Store box ID in descending order of prob score
        if(obj_cnt == 0) box_IDs[0] = j;
        else {

          //Search for position of new box ID
          for(k = 0; k < obj_cnt; k++)
            if(boxes[84*j+4+i] > boxes[84*box_IDs[k]+4+i])
              break;

          //Store box ID
          if(k < obj_cnt)
            for(l = obj_cnt; l > k; l--)
              box_IDs[l] = box_IDs[l-1];
          box_IDs[k] = j; //min prob score
        }

        //Update object counter
        obj_cnt++;
      }
    }

    //Apply NMS if more than 1 object from same class was detected
    if(obj_cnt > 1) {
      for(j = 0; j < obj_cnt; j++) {
        if(boxes[84*box_IDs[j]+4+i] == 0) continue;
        for(k = j+1; k < obj_cnt; k++) {

          //Get boxes coordinates
          x1 = boxes[84*box_IDs[j]];
          y1 = boxes[84*box_IDs[j]+1];
          w1 = boxes[84*box_IDs[j]+2];
          h1 = boxes[84*box_IDs[j]+3];
          x2 = boxes[84*box_IDs[k]];
          y2 = boxes[84*box_IDs[k]+1];
          w2 = boxes[84*box_IDs[k]+2];
          h2 = boxes[84*box_IDs[k]+3];

          //Calculate IoU (intersection over union)
          w = overlap(x1, w1, x2, w2); //Q2.14
          h = overlap(y1, h1, y2, h2); //Q2.14
          if(w > 0 && h > 0) {
            b_inter = (int32_t)((int32_t)w*(int32_t)h); //Q2.14 * Q2.14 = Q4.28
            mul_32 = (int32_t)((int32_t)w1*(int32_t)h1); //Q2.14 * Q2.14 = Q4.28
            b_union = (int16_t)(mul_32 >> 14); //w1*h1 -> Q4.28 to Q2.14
            mul_32 = (int32_t)((int32_t)w2*(int32_t)h2); //Q2.14 * Q2.14 = Q4.28
            b_union += (int16_t)(mul_32 >> 14); //w1*h1+w2*h2 -> Q4.28 to Q2.14
            b_union -= (int16_t)(b_inter >> 14); //w1*h1+w2*h2-inter -> Q4.28 to Q2.14
            b_iou = (int16_t)((int32_t)b_inter/(int32_t)b_union); //Q4.28 / Q2.14 = Q2.14
            if(b_iou > nms_threshold) boxes[84*box_IDs[k]+4+i] = 0;
          }
        }
      }
    }
  }
}

//Draw bounding box in input image
void draw_box(int left, int top, int right, int bot, uint8_t red, uint8_t green, uint8_t blue) {

  //Limit box coordinates
  if(left < 0) left = 0; else if(left >= IMG_W) left = IMG_W-1;
  if(right < 0) right = 0; else if(right >= IMG_W) right = IMG_W-1;
  if(top < 0) top = 0; else if(top >= IMG_H) top = IMG_H-1;
  if(bot < 0) bot = 0; else if(bot >= IMG_H) bot = IMG_H-1;

  //Draw horizontally
  int i;
  for(i = left; i <= right; i++) {
    fp_image[top*IMG_W*LAYER_1_C + i*LAYER_1_C] = red;
    fp_image[top*IMG_W*LAYER_1_C + i*LAYER_1_C + 1] = green;
    fp_image[top*IMG_W*LAYER_1_C + i*LAYER_1_C + 2] = blue;
    fp_image[bot*IMG_W*LAYER_1_C + i*LAYER_1_C] = red;
    fp_image[bot*IMG_W*LAYER_1_C + i*LAYER_1_C + 1] = green;
    fp_image[bot*IMG_W*LAYER_1_C + i*LAYER_1_C + 2] = blue;
  }

  //Draw vertically
  for(i = top; i <= bot; i++) {
    fp_image[i*IMG_W*LAYER_1_C + left*LAYER_1_C] = red;
    fp_image[i*IMG_W*LAYER_1_C + left*LAYER_1_C + 1] = green;
    fp_image[i*IMG_W*LAYER_1_C + left*LAYER_1_C + 2] = blue;
    fp_image[i*IMG_W*LAYER_1_C + right*LAYER_1_C] = red;
    fp_image[i*IMG_W*LAYER_1_C + right*LAYER_1_C + 1] = green;
    fp_image[i*IMG_W*LAYER_1_C + right*LAYER_1_C + 2] = blue;
  }
}

//Draw class label in input image
void draw_class(int label_w, int j, int top_width, int left, int previous_w, uint8_t r, uint8_t g, uint8_t b) {
  int l, k;
  uint8_t label;
  for(l = 0; l < label_height && (l+top_width) < IMG_H; l++) {
    for(k = 0; k < label_w && (k+left+previous_w) < IMG_W; k++) {
      label = fp_labels[81+MAX_LABEL_SIZE*j+l*label_w+k];
      //Q8.0*Q8.0=Q16.0 to Q8.0 -> red
      fp_image[(l+top_width)*IMG_W*LAYER_1_C+(k+left+previous_w)*LAYER_1_C] = ((uint16_t)((uint16_t)r*(uint16_t)label)) >> 8;
      //green
      fp_image[(l+top_width)*IMG_W*LAYER_1_C+(k+left+previous_w)*LAYER_1_C + 1] = ((uint16_t)((uint16_t)g*(uint16_t)label)) >> 8;
      //blue
      fp_image[(l+top_width)*IMG_W*LAYER_1_C+(k+left+previous_w)*LAYER_1_C + 2] = ((uint16_t)((uint16_t)b*(uint16_t)label)) >> 8;
    }
  }
}

//Draw detections (bounding boxes and class labels) in input image
void draw_detections() {

  //local variables
  int i, j, k;
  uint8_t colors[6][3] = { {255,0,255}, {0,0,255},{0,255,255},{0,255,0},{255,255,0},{255,0,0} }; //Q8.0
  uint8_t ratio, red, green, blue, label_w;
  uint16_t mul_16;
  int32_t mul_32;
  int offset, ratio_min, ratio_max;
  int left, right, top, bot, top_width, previous_w;

  //Check valid detections
  for(i = 0; i < nboxes; i++) {

    //Find detected classes
    previous_w = 0;
    for(j = 0; j < 80; j++) {
      if(boxes[84*i+4+j] != 0) {

        //Check if this was the first class detected for given box
        if(previous_w == 0) {

          //Randomly pick rgb colors for the box
          offset = j*123457 % 80;
          mul_16 = (uint16_t)((uint16_t)offset*(uint16_t)((uint8_t)0x10)); //Q8.0 *Q0.8 = Q8.8
          ratio = (uint8_t)(mul_16>>2); //Q8.8 to Q2.6
          ratio_min = (ratio >> 6);
          ratio_max = ratio_min + 1;
          ratio = ratio & 0x3F; //Q2.6
          mul_16 = (uint16_t)((uint16_t)(0x40-ratio)*(uint16_t)colors[ratio_min][2]); //Q2.6 *Q8.0 = Q10.6
          mul_16 += (uint16_t)((uint16_t)ratio*(uint16_t)colors[ratio_max][2]); //Q2.6 *Q8.0 = Q10.6
          red = (mul_16 >> 6); //Q10.6 to Q8.0
          mul_16 = (uint16_t)((uint16_t)(0x40-ratio)*(uint16_t)colors[ratio_min][1]); //Q2.6 *Q8.0 = Q10.6
          mul_16 += (uint16_t)((uint16_t)ratio*(uint16_t)colors[ratio_max][1]); //Q2.6 *Q8.0 = Q10.6
          green = (mul_16 >> 6); //Q10.6 to Q8.0
          mul_16 = (uint16_t)((uint16_t)(0x40-ratio)*(uint16_t)colors[ratio_min][0]); //Q2.6 *Q8.0 = Q10.6
          mul_16 += (uint16_t)((uint16_t)ratio*(uint16_t)colors[ratio_max][0]); //Q2.6 *Q8.0 = Q10.6
          blue = (mul_16 >> 6); //Q10.6 to Q8.0

          //Calculate box coordinates in image frame
          mul_16 = boxes[84*i] - (boxes[84*i+2]>>1); //Q2.14
          mul_32 = (int32_t)((int32_t)mul_16 * (int32_t)IMG_W); //Q2.14 * Q16.0 = Q18.14
          left = (mul_32 >> 14);
          mul_16 = boxes[84*i] + (boxes[84*i+2]>>1); //Q2.14
          mul_32 = (int32_t)((int32_t)mul_16 * (int32_t)IMG_W); //Q2.14 * Q16.0 = Q18.14
          right = (mul_32 >> 14);
          mul_16 = boxes[84*i+1] - (boxes[84*i+3]>>1); //Q2.14
          mul_32 = (int32_t)((int32_t)mul_16 * (int32_t)IMG_H); //Q2.14 * Q16.0 = Q18.14
          top = (mul_32 >> 14);
          mul_16 = boxes[84*i+1] + (boxes[84*i+3]>>1); //Q2.14
          mul_32 = (int32_t)((int32_t)mul_16 * (int32_t)IMG_H); //Q2.14 * Q16.0 = Q18.14
          bot = (mul_32 >> 14);

          //Draw box
          for(k = 0; k < box_width; k++) draw_box(left+k, top+k, right-k, bot-k, red, green, blue);

          //Limit top and left box coordinates
          if(top < 0) top = 0;
          if(left < 0) left = 0;
          top_width = top + box_width;
          if(top_width - label_height >= 0) top_width -= label_height;

        //Otherwise, add comma and space
        } else {
          label_w = fp_labels[80];
          draw_class(label_w, 80, top_width, left, previous_w, red, green, blue);
          previous_w += label_w;
        }

        //Draw class labels
        label_w = fp_labels[j];
        draw_class(label_w, j, top_width, left, previous_w, red, green, blue);
        previous_w += label_w;
      }
    }
  }
}

//print detected objects and corresponding probability scores
void print_results() {

  //local variables
  int i, j;
  const char *class_names[80] = {"person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed", "dining table", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};
  uint32_t pred_32;

  //print detections
  for(i = 0; i < nboxes; i++) {
    for(j = 0; j < 80; j++) {
      if(boxes[84*i+4+j] != 0) {
	pred_32 = (uint32_t)((uint32_t)boxes[84*i+4+j]*(uint32_t)100); //Q2.14 * Q16.0 = Q18.14
	if( (pred_32&0x3FFF) > 0x2000) uart_printf("\n%s: %d%%", class_names[j], (pred_32>>14)+1);
	else uart_printf("\n%s: %d%%", class_names[j], (pred_32>>14));
      }
    }
  }
  uart_printf("\n");
}

//send results back
void send_data() {

  //Loop to send data
  int i, j;
  count_bytes = 0;
  char * fp_data_char = (char *) fp_image;
  for(j = 0; j < NUM_OUTPUT_FRAMES+1; j++) {

    //start timer
    if(j == 0) start = timer_time_us(TIMER_BASE);

     //check if it is last packet (has less data that full payload size)
     if(j == NUM_OUTPUT_FRAMES) bytes_to_send = OUTPUT_FILE_SIZE - count_bytes;
     else bytes_to_send = ETH_NBYTES;

     //prepare variable to be sent
     for(i = 0; i < bytes_to_send; i++) data_to_send[i] = fp_data_char[j*ETH_NBYTES*2 + i*2];

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
  uart_printf("\nYOLOV3 TINY\n\n");

#ifndef SIM
  //init ETHERNET
  eth_init(ETHERNET_BASE);
  eth_set_rx_payload_size(ETH_NBYTES);
#endif

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

  //pre-initialize DDR
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
  total_time = end-start;
 #endif

  //height resize
  #ifndef TIME_RUN
  uart_printf("\nHeight resizing...\n");
  start = timer_time_us(TIMER_BASE);
 #endif
  height_resize();
 #ifndef TIME_RUN
  end = timer_time_us(TIMER_BASE);
  uart_printf("Done in %d us\n\n", end-start);
  total_time += end-start;
 #endif

  //layers 1 and 2
 #ifndef TIME_RUN
  uart_printf("\nRunning layers 1 and 2...\n");
  start = timer_time_us(TIMER_BASE);
 #endif
  layer1();
 #ifndef TIME_RUN
  end = timer_time_us(TIMER_BASE);
  uart_printf("Convolution + maxpool done in %d us\n\n", end-start);
  total_time += end-start;
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
  uart_printf("Convolution + yolo done in %d us\n\n", end-start);
  total_time += end-start;
 #endif

#endif

  //create boxes from 1st yolo layer
 #ifndef TIME_RUN
  uart_printf("\nCreating boxes from first yolo layer\n");
  start = timer_time_us(TIMER_BASE);
 #endif
 #ifdef SIM
  create_boxes(LAYER_16_W, 0, yolo1_div, 1);
 #else
  create_boxes(LAYER_16_W, data_pos_layer19-DATA_LAYER_16, yolo1_div, 1);
 #endif
 #ifndef TIME_RUN
  end = timer_time_us(TIMER_BASE);
  uart_printf("Done in %d us\n\n", end-start);
  total_time += end-start;
 #endif

  //create boxes from 2nd yolo layer
 #ifndef TIME_RUN
  uart_printf("\nCreating boxes from second yolo layer\n");
  start = timer_time_us(TIMER_BASE);
 #endif
 #ifdef SIM
  create_boxes(LAYER_23_W, 256*13*13, yolo2_div, 0);
 #else
  create_boxes(LAYER_23_W, p_pos, yolo2_div, 0);
 #endif
 #ifndef TIME_RUN
  end = timer_time_us(TIMER_BASE);
  uart_printf("Done in %d us\n\n", end-start);
  total_time += end-start;
 #endif

  //filter boxes
 #ifndef TIME_RUN
  uart_printf("\nFiltering boxes...\n");
  start = timer_time_us(TIMER_BASE);
 #endif
  filter_boxes();
 #ifndef TIME_RUN
  end = timer_time_us(TIMER_BASE);
  uart_printf("Done in %d us\n\n", end-start);
  total_time += end-start;
 #endif

#ifndef SIM
  //draw boxes and labels
  uart_printf("\nDrawing boxes and labels...\n");
  start = timer_time_us(TIMER_BASE);
  draw_detections();
  end = timer_time_us(TIMER_BASE);
  uart_printf("Done in %d us\n\n", end-start);
  total_time += end-start;
#else
  //verify results
  int16_t * fp_data = (int16_t *) DATA_BASE_ADDRESS;
  fp_data += 256*13*13 + 256*26*26;
  int i;
  uart_printf("\nVerifying...\n");
  for(i = 0; i < 84*nboxes; i++)
    if(boxes[i] != fp_data[i])
      uart_printf("(%d) res = %x, act = %x\n", i, boxes[i] & 0xFFFF, fp_data[i] & 0xFFFF);
#endif

#ifndef SIM
  uart_printf("\n\n TOTAL_TIME = %d us\n\n", total_time);
  //print detected objects and corresponding probability scores
  uart_printf("\nDetections:\n");
  print_results();
#endif

#ifndef SIM
  send_data();
#endif

  //finish
  uart_putc(ETX);
  uart_txwait();
  return 0;
}
