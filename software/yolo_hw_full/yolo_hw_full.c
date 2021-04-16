//import custom libraries
#include "system.h"
#include "periphs.h"
#include "iob-uart.h"
#include "iob-eth.h"
#include "iob_timer.h"
#include "yolo_hw_full.h"
/* #include "iob-cache.h" */
#include "versat.hpp"

//import c libraries
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//define tiling of each layer
#ifdef SIM
  #define LAYER_1_TILING_W 4 //must be divisor of 416
  #define LAYER_1_TILING_H 4
  #define LAYER_3_TILING_W 4 //must be divisor of 208
  #define LAYER_3_TILING_H 4
  #define LAYER_5_TILING_W 4 //must be divisor of 104
  #define LAYER_5_TILING_H 4
  #define LAYER_7_TILING_W 4  //must be divisor of 52
  #define LAYER_7_TILING_H 4
  #define LAYER_9_TILING_W 1  //must be divisor of 26
  #define LAYER_9_TILING_H 1
  #define LAYER_11_TILING_W 1  //must be divisor of 13
  #define LAYER_11_TILING_H 1
  #define LAYER_16_TILING_W 1 //must be divisor of 13
  #define LAYER_16_TILING_H 1
  #define LAYER_22_TILING_W 1  //must be divisor of 26
  #define LAYER_22_TILING_H 1
  #define LAYER_23_TILING_W 1  //must be divisor of 26
  #define LAYER_23_TILING_H 1
#else
  #define LAYER_1_TILING_W 416 //must be divisor of 416
  #define LAYER_1_TILING_H 416
  #define LAYER_3_TILING_W 208 //must be divisor of 208
  #define LAYER_3_TILING_H 208
  #define LAYER_5_TILING_W 104 //must be divisor of 104
  #define LAYER_5_TILING_H 104
  #define LAYER_7_TILING_W 52  //must be divisor of 52
  #define LAYER_7_TILING_H 52
  #define LAYER_9_TILING_W 26  //must be divisor of 26
  #define LAYER_9_TILING_H 26
  #define LAYER_11_TILING_W 13  //must be divisor of 13
  #define LAYER_11_TILING_H 13
  #define LAYER_16_TILING_W 13 //must be divisor of 13
  #define LAYER_16_TILING_H 13
  #define LAYER_22_TILING_W 26  //must be divisor of 26
  #define LAYER_22_TILING_H 26
  #define LAYER_23_TILING_W 26  //must be divisor of 26
  #define LAYER_23_TILING_H 26
#endif

//define peripheral base addresses
/* #define DDR_MEM (CACHE_BASE<<(ADDR_W-N_SLAVES_W)) */
// set pointer to DDR base
#ifndef PCSIM
  //USE_DDR==1 to run yolo anyways
  #if (RUN_DDR==0) // running firmware from SRAM 
    #define DDR_MEM ((uint8_t*) EXTRA_BASE)
  #else //running firmware from DDR
    #define DDR_MEM ((uint8_t*) (1<<(FIRM_ADDR_W)))
  #endif
#else
  uint8_t ddr_mem_vect[1<<(DDR_ADDR_W-2)]; //DDR addressable space, up to 2**30
  #define DDR_MEM (&ddr_mem_vect[0])
#endif //ifndef PCSIM



//Constants for image resize
#define w_scale ((float)(IMG_W-1)/(NEW_W-1))
#define h_scale ((float)(IMG_H-1)/(NEW_H-1))

//define general constants
#define ETH_NBYTES (1024-18) //minimum ethernet payload excluding FCS
#define INPUT_FILE_SIZE IMAGE_INPUT //8 bits per pixel
#define NUM_INPUT_FRAMES (INPUT_FILE_SIZE/ETH_NBYTES)
#define WEIGHTS_FILE_SIZE (TOTAL_WEIGHTS*2) //16 bits per weight
#define NUM_WEIGHT_FRAMES (WEIGHTS_FILE_SIZE/ETH_NBYTES)
#define OUTPUT_FILE_SIZE (nboxes*84*2) //16 bits per output
#define NUM_OUTPUT_FRAMES (OUTPUT_FILE_SIZE/ETH_NBYTES)
#define ix_size (NEW_W*4)
#define iy_size (NEW_H)
#define dx_size (NEW_W*2)
#define dy_size (NEW_H*2)

//define DDR mapping
#define ix_BASE_ADDRESS (DDR_MEM + (int)pow(2,FIRM_ADDR_W)) //after main mem
#define iy_BASE_ADDRESS (ix_BASE_ADDRESS + ix_size*2) //16 bits
#define dx_BASE_ADDRESS (iy_BASE_ADDRESS + iy_size*2) //16 bits
#define dy_BASE_ADDRESS (dx_BASE_ADDRESS + dx_size*2) //16 bits
#define WEIGHTS_BASE_ADDRESS (dy_BASE_ADDRESS + dy_size*2) //16 bits
#define DATA_BASE_ADDRESS (WEIGHTS_BASE_ADDRESS + WEIGHTS_FILE_SIZE) //16 bits

//ETHERNET variables
int rcv_timeout = 5000;
char data_to_send[ETH_NBYTES];
char data_rcv[ETH_NBYTES+18];
unsigned int bytes_to_receive, bytes_to_send, count_bytes;

//TIMER variables
unsigned int start, end; 
unsigned int total_data_time, total_config_time, total_run_time, total_time;

//data base address pointers
int16_t * fp_data, * fp_image, * fp_weights;
int16_t * ix, * iy, * dx, * dy;

//weights and data updatable pointers
unsigned int weight_pos = 0, data_pos = NETWORK_INPUT_AUX;

//Constants for bounding boxes
#define threshold ((int16_t)(((float)0.5)*((int32_t)1<<8))) //Q8.8
#define yolo1_div ((int16_t)(((float)1/LAYER_16_W)*((int32_t)1<<15))) //Q1.15
#define yolo2_div ((int16_t)(((float)1/LAYER_23_W)*((int32_t)1<<15))) //Q1.15
#define x_scales ((int16_t)(((float)YOLO_INPUT/NEW_W)*((int32_t)1<<14))) //Q2.14
#define x_bias ((int16_t)(((float)(YOLO_INPUT-NEW_W)/(NEW_W*2))*((int32_t)1<<14))) //Q2.14
#define y_scales ((int16_t)(((float)YOLO_INPUT/NEW_H)*((int32_t)1<<14))) //Q2.14
#define y_bias ((int16_t)(((float)(YOLO_INPUT-NEW_H)/(NEW_H*2))*((int32_t)1<<14))) //Q2.14
#define w_scales ((int16_t)(((float)1/NEW_W)*((int32_t)1<<14))) //Q2.14
#define h_scales ((int16_t)(((float)1/NEW_H)*((int32_t)1<<14))) //Q2.14
#define c3 ((int16_t)0x0AAA) // pow(2,-3)+pow(2,-5)+pow(2,-7)+pow(2,-9)+pow(2,-11)+pow(2,-13) in Q2.14
#define c4 ((int16_t)0x02C0) // pow(2,-5)+pow(2,-7)+pow(2,-8) in Q2.14
uint8_t nboxes = 0;

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

  //weights
  fp_weights = (int16_t *) WEIGHTS_BASE_ADDRESS;
}

//reset certain DDR positions to zero due to padding
void reset_DDR() {
  
  //local variables
  unsigned int i, pos;

  //measure initial time
  printf("\nSetting DDR positions to zero\n");
  start = timer_time_us();

  //resized image
  for(i = 0; i < IMAGE_INPUT; i++) fp_image[i] = 0;
  pos = NETWORK_INPUT_AUX;

  //input network
  for(i = 0; i < NETWORK_INPUT; i++) fp_data[pos + i] = 0;
  pos += NETWORK_INPUT;

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
  pos += DATA_LAYER_14 + DATA_LAYER_15 + DATA_LAYER_16 + DATA_LAYER_19;

  //layer 20 and 9
  for(i = 0; i < DATA_LAYER_20 + DATA_LAYER_9; i++) fp_data[pos + i] = 0;

  //measure final time
  end = timer_time_us();
  printf("DDR reset to zero done in %d ms\n", (end-start)/1000);
}

void rcv_frame(unsigned int NUM_DATA_FRAMES, unsigned int DATA_SIZE, char * data_p) {

  //Local variables
  int i, j;
  count_bytes = 0;

  //Loop to receive intermediate data frames
  for(j = 0; j < NUM_DATA_FRAMES+1; j++) {

     //wait to receive frame
     while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout) !=0);

     // start timer
     if(j == 0) start = timer_time_us();   

     //check if it is last packet (has less data that full payload size)
     if(j == NUM_DATA_FRAMES) bytes_to_receive = DATA_SIZE - count_bytes;
     else bytes_to_receive = ETH_NBYTES;

     //save in DDR
     for(i = 0; i < bytes_to_receive; i++) {
       if(DATA_SIZE == INPUT_FILE_SIZE) data_p[j*ETH_NBYTES*2 + i*2] = data_rcv[14+i];
       else data_p[j*ETH_NBYTES + i] = data_rcv[14+i];
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
  printf("\nReady to receive input image and weights...\n");
  char * fp_image_char = (char *) DATA_BASE_ADDRESS;
  rcv_frame(NUM_INPUT_FRAMES, INPUT_FILE_SIZE, fp_image_char);
  end = timer_time_us();
  printf("Image received in %d ms\n", (end-start)/1000);

  //Receive weights
  char * fp_weights_char = (char *) WEIGHTS_BASE_ADDRESS;
  rcv_frame(NUM_WEIGHT_FRAMES, WEIGHTS_FILE_SIZE, fp_weights_char);
  end = timer_time_us();
  printf("weights transferred in %d ms\n", (end-start)/1000);
}

//fill resized image region with grey (0.5 = 0x0080 in Q8.8)
void fill_grey() {
  int i, j, k;
  for(i = 0; i < IMG_C; i++)
    for(j = 0; j < YOLO_INPUT; j++)
      for(k = 0; k < YOLO_INPUT; k++)
	fp_data[i*(YOLO_INPUT+2)*(YOLO_INPUT+2) + (j+1)*(YOLO_INPUT+2) + (k+1) + NETWORK_INPUT_AUX] = 0x0080;
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

//resize input image to 416x312x3
void resize_image() {

  //local variables
  int i, j, k;
  unsigned int data_time, config_time, run_time = 0;
#ifdef SIM
  int num_err = 0;
#endif

  ///////////////////////////////////////////////////////////////////////////////
  //     1st step -> IMG_WxIMG_HxIMG_C to (NEW_W*2)xNEW_H*IMG_C
  ///////////////////////////////////////////////////////////////////////////////
  
  //load ix and dx to versat mem0 and mem2
  start = timer_time_us();
  for(i = 0; i < ix_size; i++) stage[0].memA[0].write(i, ix[i]);
  for(i = 0; i < dx_size; i++) stage[0].memA[2].write(i, dx[i]);
  end = timer_time_us();
  data_time = end - start;

  //configure mem0 to read: ix, ix+1, ix+2*NEW_W, ix+2*NEW_W+1 sequence
  start = timer_time_us();
  stage[0].memA[0].setDuty(2);
  stage[0].memA[0].setPer(2);
  stage[0].memA[0].setIncr(1);
  stage[0].memA[0].setIter(2);
  stage[0].memA[0].setShift(2*NEW_W-2);
  stage[0].memA[0].setPer2(NEW_W);
  stage[0].memA[0].setIncr2(2);
  stage[0].memA[0].setIter2(1);

  //configure mem1 to read input pixels (addressed by mem0)
  stage[0].memA[1].setDelay(MEMP_LAT);
  stage[0].memA[1].setDuty(4*NEW_W);
  stage[0].memA[1].setPer(4*NEW_W);
  stage[0].memA[1].setIter(1);
  stage[0].memA[1].setSel(sMEMA[0]);
  stage[0].memA[1].setExt(1);

  //configure mem2 to read 1-dx, dx sequence twice
  stage[0].memA[2].setDuty(2);
  stage[0].memA[2].setDelay(MEMP_LAT);
  stage[0].memA[2].setPer(2);
  stage[0].memA[2].setIncr(1);
  stage[0].memA[2].setIter(2);
  stage[0].memA[2].setShift(-2);
  stage[0].memA[2].setPer2(NEW_W);
  stage[0].memA[2].setIncr2(2);
  stage[0].memA[2].setIter2(1);

  //pixel * 1-dx/dx = res0
  stage[0].yolo[0].setSelA(sMEMA[1]);
  stage[0].yolo[0].setSelB(sMEMA[2]);
  stage[0].yolo[0].setIter(2*NEW_W);
  stage[0].yolo[0].setPer(2);
  stage[0].yolo[0].setDelay(2*MEMP_LAT);
  stage[0].yolo[0].setShift(7); //Q10.22 to Q1.15

  //store res0 in mem3
  stage[0].memA[3].setDelay(2*MEMP_LAT+YOLO_LAT-1);
  stage[0].memA[3].setDuty(1);
  stage[0].memA[3].setPer(2);
  stage[0].memA[3].setIncr(1);
  stage[0].memA[3].setIter(2*NEW_W);
  stage[0].memA[3].setSel(sYOLO[0]);
  stage[0].memA[3].setInWr(1);
  end = timer_time_us();
  config_time = end - start;

  //loops for performing resizing 1st step
#ifdef SIM
  for(k = 0; k < 1; k++) {
    for(j = 0; j < 1; j++) {
#else
  for(k = 0; k < IMG_C; k++) {
    for(j = 0; j < NEW_H; j++) {
#endif

      //Store 2 lines of pixels in mem1
      //Extra pixel is necessary to be multiplied by zero
      start = timer_time_us();
      for(i = 0; i < IMG_W*2+1; i++)
        stage[0].memA[1].write(i, fp_image[k*IMG_W*IMG_H + iy[j]*IMG_W + i]);
      end = timer_time_us();
      data_time += (end - start);

      //Wait until done
      start = timer_time_us();
      run();
      while(done() == 0);
      end = timer_time_us();
      run_time += (end - start);

      //store result in DDR
      start = timer_time_us();
      for(i = 0; i < NEW_W*2; i++)
      #ifdef SIM
        if(fp_data[k*2*NEW_W*NEW_H + j*2*NEW_W + i] != stage[0].memA[3].read(i)) num_err++;
      #else
        fp_data[k*2*NEW_W*NEW_H + j*2*NEW_W + i] = stage[0].memA[3].read(i);
      #endif
      end = timer_time_us();
      data_time += (end - start);
    }
  }
#ifdef SIM
  printf("Resize 1st step done with %d errors\n", num_err);
  num_err = 0;
#endif

  ///////////////////////////////////////////////////////////////////////////////
  //     2nd step -> (NEW_W*2)xNEW_H*IMG_C to NEW_W*NEW_H*IMG_C
  ///////////////////////////////////////////////////////////////////////////////
  
  //clear configuration of all stages
  globalClearConf();

  //load dy to versat mem1
  start = timer_time_us();
  for(i = 0; i < dy_size; i++) stage[0].memA[1].write(i, dy[i]);
  end = timer_time_us();
  data_time += end - start;

  //configure mem0 to read res0
  start = timer_time_us();
  stage[0].memA[0].setDuty(2);
  stage[0].memA[0].setPer(2);
  stage[0].memA[0].setIncr(1);
  stage[0].memA[0].setIter(NEW_W);

  //configure mem1 to read 1-dy, dy
  stage[0].memA[1].setDuty(2);
  stage[0].memA[1].setPer(2);
  stage[0].memA[1].setIncr(1);
  stage[0].memA[1].setIter(NEW_W);
  stage[0].memA[1].setShift(-2);

  //pixel * 1-dy/dy = res1
  stage[0].yolo[0].setSelA(sMEMA[0]);
  stage[0].yolo[0].setSelB(sMEMA[1]);
  stage[0].yolo[0].setIter(NEW_W);
  stage[0].yolo[0].setPer(2);
  stage[0].yolo[0].setDelay(MEMP_LAT);
  stage[0].yolo[0].setShift(21); //Q3.29 to Q8.8

  //store res1 in mem3
  stage[0].memA[3].setDelay(MEMP_LAT+YOLO_LAT-1);
  stage[0].memA[3].setDuty(1);
  stage[0].memA[3].setPer(2);
  stage[0].memA[3].setIncr(1);
  stage[0].memA[3].setIter(NEW_W);
  stage[0].memA[3].setSel(sYOLO[0]);
  stage[0].memA[3].setInWr(1);
  end = timer_time_us();
  config_time += end - start;

  //loops for performing resizing 2nd step
#ifdef SIM
  for(k = 0; k < 1; k++) {
    for(j = 0; j < 1; j++) {
#else
  for(k = 0; k < IMG_C; k++) {
    for(j = 0; j < NEW_H; j++) {
#endif

      //Store res1 in mem0
      start = timer_time_us();
      for(i = 0; i < 2*NEW_W; i++)
        stage[0].memA[0].write(i, fp_data[k*2*NEW_W*NEW_H + j*2*NEW_W + i]);
      end = timer_time_us();
      data_time += (end - start);

      //run
      start = timer_time_us();
      run();

      //configure mem1 start
    #ifdef SIM
      if(j == 0)
    #else
      if(j == NEW_H-1)
    #endif
        stage[0].memA[0].setStart(0);
      else
        stage[0].memA[1].setStart(2*(j+1));

      //Wait until done
      while(done() == 0);
      end = timer_time_us();
      run_time += (end - start);

      //store result in DDR
      start = timer_time_us();
      for(i = 0; i < NEW_W; i++)
      #ifdef SIM
        if(fp_data[k*(YOLO_INPUT+2)*(YOLO_INPUT+2) + (j+1)*(YOLO_INPUT+2) + (i+1) + EXTRA_W + ((YOLO_INPUT+2)*EXTRA_H) + NETWORK_INPUT_AUX] != stage[0].memA[3].read(i)) num_err++;
      #else
        fp_data[k*(YOLO_INPUT+2)*(YOLO_INPUT+2) + (j+1)*(YOLO_INPUT+2) + (i+1) + EXTRA_W + ((YOLO_INPUT+2)*EXTRA_H) + NETWORK_INPUT_AUX] = stage[0].memA[3].read(i);
      #endif
      end = timer_time_us();
      data_time += (end - start);
    }
  }
#ifdef SIM
  printf("Resize 2nd step done with %d errors\n", num_err);
#endif
  printf("FU config time = %d us\n", config_time);
  printf("FU run time = %d us\n", run_time);
  printf("FU load/store data time = %d us\n", data_time);
  printf("Total time = %d ms\n", (config_time + run_time + data_time)/1000);
  total_config_time = config_time;
  total_run_time = run_time;
  total_data_time = data_time;
  total_time = (config_time + run_time + data_time)/1000;
}

//perform convolutional layer
void conv_layer(int w, int c, int num_ker, int ker_size, int til_w, int til_h, int maxpool, int outpadd, int stride, int inpadd, int ignorepadd, unsigned int outpos) {

  //clear configuration of all stages
  globalClearConf();

  //update pointers
  int16_t * input = (int16_t *) fp_data + data_pos;
  if(outpos == 0) data_pos += (w+2*inpadd)*(w+2*inpadd)*c;
  else data_pos = outpos;
  int16_t * output = (int16_t *) fp_data + data_pos;
  int16_t * weights = (int16_t *) fp_weights + weight_pos;
  weight_pos += num_ker*ker_size*ker_size*c;
  int16_t * bias = (int16_t *) fp_weights + weight_pos;
  weight_pos += num_ker;

  //local variables
  int i, j, k, l, m;
#ifdef SIM
  int num_err = 0;
#endif
  unsigned int data_time = 0, config_time, run_time = 0;

  //configure mem0 to read ker_size*ker_size*c blocks from tiled FM
  start = timer_time_us();
  stage[0].memA[0].setDuty(ker_size*c);
  stage[0].memA[0].setPer(ker_size*c);
  stage[0].memA[0].setIncr(1);
  stage[0].memA[0].setIter(ker_size);
  stage[0].memA[0].setShift((til_w+2*inpadd)*c-ker_size*c);
  stage[0].memA[0].setIncr2(c);
  if(maxpool) {
    stage[0].memA[0].setPer2(2);
    stage[0].memA[0].setIter2(2);
    stage[0].memA[0].setShift2((til_w+2)*c-2*c);
    stage[0].memA[0].setPer3(til_w/2);
    stage[0].memA[0].setIncr3(2*c);
    stage[0].memA[0].setIter3(til_h/2);
    stage[0].memA[0].setShift3((til_w+2)*c*2-til_w*c);
  } else if (til_w != 1 || til_h != 1) {
    stage[0].memA[0].setPer2(til_w);
    stage[0].memA[0].setIter2(til_h);
    stage[0].memA[0].setShift2((til_w+2*inpadd)*c-til_w*c);
  }

  //configure mem1 to read weights
  stage[0].memA[1].setDuty(ker_size*ker_size*c);
  stage[0].memA[1].setPer(ker_size*ker_size*c);
  stage[0].memA[1].setIncr(1);
  stage[0].memA[1].setIter(til_w*til_h);
  stage[0].memA[1].setShift(-ker_size*ker_size*c);

  //configure mem2 to read bias
  stage[0].memA[2].setIter(til_w*til_h);
  stage[0].memA[2].setPer(ker_size*ker_size*c);

  //configure yolo0 to perform convolutions
  stage[0].yolo[0].setSelA(sMEMA[0]);
  stage[0].yolo[0].setSelB(sMEMA[1]);
  stage[0].yolo[0].setSelC(sMEMA[2]);
  stage[0].yolo[0].setIter(til_w*til_h);
  stage[0].yolo[0].setPer(ker_size*ker_size*c);
  stage[0].yolo[0].setDelay(MEMP_LAT);
  stage[0].yolo[0].setBias(1);
  stage[0].yolo[0].setShift(10);
  if(num_ker != 255) stage[0].yolo[0].setLeaky(1);
  stage[0].yolo[0].setMaxpool(maxpool);

  //configure mem3 to write convolution results
  stage[0].memA[3].setDelay(MEMP_LAT+YOLO_LAT-1);
  stage[0].memA[3].setDuty(1);
  stage[0].memA[3].setIncr(1);
  stage[0].memA[3].setPer(ker_size*ker_size*c*(1+3*maxpool)); //CAREFUL WITH PERIOD_W!!!
  stage[0].memA[3].setIter(til_w*til_h/(1+3*maxpool));
  stage[0].memA[3].setSel(sYOLO[0]);
  stage[0].memA[3].setInWr(1);
  end = timer_time_us();
  config_time = end - start;

  //loops for performing convolution
#ifdef SIM
  for(l = 0; l < 1; l++) {
    for(m = 0; m < 1; m++) {
#else
  for(l = 0; l < w/til_h; l++) {
    for(m = 0; m < w/til_w; m++) {
#endif

      //Send input FM tile (z-x-y format)
      start = timer_time_us();
      for(i = 0; i < c; i++)
	for(j = 0; j < til_h+2*inpadd; j++)
          for(k = 0; k < til_w+2*inpadd; k++)
	    stage[0].memA[0].write((j*(til_w+2*inpadd)+k)*c + i, input[i*(w+2*inpadd+2*ignorepadd)*(w+2*inpadd+2*ignorepadd) + (l*til_h+j+ignorepadd)*(w+2*inpadd+2*ignorepadd) + (k+m*til_w+ignorepadd)]);
      end = timer_time_us();
      data_time += (end - start);

#ifdef SIM
      for(k = 0; k < 1; k++) {
    #else
      for(k = 0; k < num_ker; k++) {
    #endif

        //load weights (z-x-y format)
        start = timer_time_us();
        for(j = 0; j < c; j++)
          for(i = 0; i < ker_size*ker_size; i++)
            stage[0].memA[1].write(i*c + j, weights[k*c*ker_size*ker_size + j*ker_size*ker_size + i]);
        stage[0].memA[2].write(0, bias[k]);
        end = timer_time_us();
        data_time += end - start;

        //run
        start = timer_time_us();
        run();

        //Wait until done
        while(done() == 0);
        end = timer_time_us();
        run_time += (end - start);

        //store result in DDR
        start = timer_time_us();
	for(i = 0; i < til_h/(1+maxpool); i++)
          for(j = 0; j < til_w/(1+maxpool); j++)
          #ifdef SIM
            if(output[k*(w/(1+maxpool)+2*outpadd+stride)*(w/(1+maxpool)+2*outpadd+stride) + ((til_h/(1+maxpool))*l+i+outpadd)*(w/(1+maxpool)+2*outpadd+stride) + ((til_w/(1+maxpool))*m+j+outpadd)] != stage[0].memA[3].read(i*(til_w/(1+maxpool))+j)) num_err++;
          #else
            output[k*(w/(1+maxpool)+2*outpadd+stride)*(w/(1+maxpool)+2*outpadd+stride) + ((til_h/(1+maxpool))*l+i+outpadd)*(w/(1+maxpool)+2*outpadd+stride) + ((til_w/(1+maxpool))*m+j+outpadd)] = stage[0].memA[3].read(i*(til_w/(1+maxpool))+j);
          #endif

        //end measuring data load/store time
        end = timer_time_us();
        data_time += (end - start);
      }
    }
  }
#ifdef SIM
  printf("Layer done with %d errors\n", num_err);
#endif
  printf("FU config time = %d us\n", config_time);
  printf("FU run time = %d us\n", run_time);
  printf("FU load/store data time = %d us\n", data_time);
  printf("Total time = %d ms\n", (config_time + run_time + data_time)/1000);
  total_config_time += config_time;
  total_run_time += run_time;
  total_data_time += data_time;
  total_time += (config_time + run_time + data_time)/1000;
}

//perform maxpool layer
//when inpadd = 1, it does downsampling
void maxpool_layer(int w, int c, int inpadd, int stride, unsigned int outpos) {

  //clear configuration of all stages
  globalClearConf();

  //update pointers
  int16_t * input = (int16_t *) fp_data + data_pos;
  if(outpos == 0) data_pos += (w+2*inpadd+stride)*(w+2*inpadd+stride)*c;
  else data_pos = outpos;
  int16_t * output = (int16_t *) fp_data + data_pos;

  //local variables
  int i, j, k, l;
#ifdef SIM
  int num_err = 0;
#endif
  unsigned int data_time = 0, config_time, run_time = 0;

  //configure mem0 to read input FMs
  start = timer_time_us();
  stage[0].memA[0].setDuty(2);
  stage[0].memA[0].setPer(2);
  stage[0].memA[0].setIncr(1);
  stage[0].memA[0].setIter(2);
  stage[0].memA[0].setShift(w+stride-2);
  stage[0].memA[0].setPer2(w/(1+inpadd));
  stage[0].memA[0].setIncr2(1+inpadd);
  stage[0].memA[0].setIter2(w/(1+inpadd));
  stage[0].memA[0].setShift2(w+stride-w*stride);

  //configure yolo0 to perform maxpool
  stage[0].yolo[0].setSelA(sMEMA[0]);
  stage[0].yolo[0].setIter(w*(1+stride));
  stage[0].yolo[0].setPer(w*(1+stride));
  stage[0].yolo[0].setDelay(MEMP_LAT);
  stage[0].yolo[0].setMaxpool(1);
  stage[0].yolo[0].setBypass(1);

  //configure mem3 to write maxpool results
  stage[0].memA[3].setDuty(1);
  stage[0].memA[3].setIncr(1);
  stage[0].memA[3].setPer(4);
  stage[0].memA[3].setIter(w*w/(1+3*inpadd));
  stage[0].memA[3].setDelay(MEMP_LAT+YOLO_BYPASS_LAT-1);
  stage[0].memA[3].setSel(sYOLO[0]);
  stage[0].memA[3].setInWr(1);
  end = timer_time_us();
  config_time = end - start;

#ifdef SIM
  for(k = 0; k < 1; k++) {
#else
  for(k = 0; k < c; k++) {
#endif

    //Send input FM tile (x-y-z format)
    start = timer_time_us();
    for(j = 0; j < w+stride; j++)
      for(i = 0; i < w+stride; i++)
        stage[0].memA[0].write(j*(w+stride) + i, input[k*(w+2*inpadd+stride)*(w+2*inpadd+stride) + (j+inpadd)*(w+2*inpadd+stride) + i+inpadd]);
    end = timer_time_us();
    data_time += (end - start);

    //run
    start = timer_time_us();
    run();

    //Wait until done
    while(done() == 0);
    end = timer_time_us();
    run_time += (end - start);

    //store result in DDR
    start = timer_time_us();
    for(j = 0; j < w/(1+inpadd); j++)
      for(i = 0; i < w/(1+inpadd); i++)
      #ifdef SIM
        if(output[k*(w/(1+inpadd)+2)*(w/(1+inpadd)+2) + (j+1)*(w/(1+inpadd)+2) + i+1] != stage[0].memA[3].read(j*(w/(1+inpadd)) + i)) num_err++;
      #else
        output[k*(w/(1+inpadd)+2)*(w/(1+inpadd)+2) + (j+1)*(w/(1+inpadd)+2) + i+1] = stage[0].memA[3].read(j*(w/(1+inpadd)) + i);
      #endif
    end = timer_time_us();
    data_time += (end - start);
  }
#ifdef SIM
  printf("Layer done with %d errors\n", num_err);
#endif
  printf("FU config time = %d us\n", config_time);
  printf("FU run time = %d us\n", run_time);
  printf("FU load/store data time = %d us\n", data_time);
  printf("Total time = %d ms\n", (config_time + run_time + data_time)/1000);
  total_config_time += config_time;
  total_run_time += run_time;
  total_data_time += data_time;
  total_time += (config_time + run_time + data_time)/1000;
}

//perform upsample layer
void upsample() {

  //clear configuration of all stages
  globalClearConf();

  //update pointers
  int16_t * input = (int16_t *) fp_data + data_pos;
  data_pos += DATA_LAYER_19;
  int16_t * output = (int16_t *) fp_data + data_pos;

  //local variables
  int i, j, k, l;
#ifdef SIM
  int num_err = 0;
#endif
  unsigned int data_time = 0, config_time, run_time = 0;

  //configure mem0 to read same pixel 4 times
  start = timer_time_us();
  stage[0].memA[0].setDuty(4);
  stage[0].memA[0].setPer(4);
  stage[0].memA[0].setIter(1);
  stage[0].memA[0].setPer2(13);
  stage[0].memA[0].setIncr2(1);
  stage[0].memA[0].setIter2(13);

  //configure yolo0 to perform bypass
  stage[0].yolo[0].setSelA(sMEMA[0]);
  stage[0].yolo[0].setDelay(MEMP_LAT);
  stage[0].yolo[0].setBypass(1);

  //configure mem3 to write maxpool results
  stage[0].memA[3].setDuty(2);
  stage[0].memA[3].setPer(2);
  stage[0].memA[3].setIncr(1);
  stage[0].memA[3].setIter(2);
  stage[0].memA[3].setShift(26-2);
  stage[0].memA[3].setPer2(13);
  stage[0].memA[3].setIncr2(2);
  stage[0].memA[3].setIter2(13);
  stage[0].memA[3].setShift2(26);
  stage[0].memA[3].setDelay(MEMP_LAT+YOLO_BYPASS_LAT);
  stage[0].memA[3].setSel(sYOLO[0]);
  stage[0].memA[3].setInWr(1);
  end = timer_time_us();
  config_time = end - start;

#ifdef SIM
  for(k = 0; k < 1; k++) {
#else
  for(k = 0; k < 128; k++) {
#endif

    //Send input FM tile (x-y-z format)
    start = timer_time_us();
    for(j = 0; j < 13; j++)
      for(i = 0; i < 13; i++)
        stage[0].memA[0].write(j*13 + i, input[k*13*13 + j*13 + i]);
    end = timer_time_us();
    data_time += (end - start);

    //run
    start = timer_time_us();
    run();

    //Wait until done
    while(done() == 0);
    end = timer_time_us();
    run_time += (end - start);

    //store result in DDR
    start = timer_time_us();
    for(j = 0; j < 26; j++)
      for(i = 0; i < 26; i++)
      #ifdef SIM
        if(output[k*28*28 + (j+1)*28 + i+1] != stage[0].memA[3].read(j*26 + i)) num_err++;
      #else
        output[k*28*28 + (j+1)*28 + i+1] = stage[0].memA[3].read(j*26 + i);
      #endif
    end = timer_time_us();
    data_time += (end - start);
  }
#ifdef SIM
  printf("Layer done with %d errors\n", num_err);
#endif
  printf("FU config time = %d us\n", config_time);
  printf("FU run time = %d us\n", run_time);
  printf("FU load/store data time = %d us\n", data_time);
  printf("Total time = %d ms\n", (config_time + run_time + data_time)/1000);
  total_config_time += config_time;
  total_run_time += run_time;
  total_data_time += data_time;
  total_time += (config_time + run_time + data_time)/1000;
}

//linear approximation of sigmoid function
int16_t sigmoid(int16_t val) {
  int16_t fp2375 = 0x260, fp084375 = 0xD8, fp0625 = 0xA0, fp05 = 0x80; //Q8.8
  int16_t fp5 = 0x500, fp1 = 0x100; //Q8.8
  int16_t val_out;
  if(val < 0.) val_out = ~val + 1; //emulates multiplying by -1
  else val_out = val;
  if(val_out >= fp5) val_out = fp1;
  else if(val_out >= fp2375) val_out = fp084375 + (val_out >> 5); //emulates multiplying by 0.03125 = 2^(-5)
  else if(val_out >= fp1) val_out = fp0625 + (val_out >> 3); //emulates multiplying by 0.125 = 2^(-3)
  else val_out = fp05 + (val_out >> 2); //emulates multiplying by 0.25 = 2^(-2);
  if(val < 0.) val_out = fp1 - val_out;
  return val_out;
}

//polynomial approximation of exponential function
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

//apply sigmoid to input FM
void yolo_layer(int w, int16_t xy_div, int first_yolo, unsigned int in_pos, unsigned int out_pos) {

  //locate data pointers
  int16_t * input = (int16_t *) fp_data + in_pos;
  int16_t * output = (int16_t *) fp_data + out_pos;

  //local variables
  int16_t i, j, k, m;
  int16_t obj_score, pred_score;
  int32_t val_32;
  int16_t yolo_bias[12] = {0x0280, 0x0380, 0x05C0, 0x06C0, 0x0940, 0x0E80, 0x1440, 0x1480, 0x21C0, 0x2A40, 0x5600, 0x4FC0}; //Q10.6
#ifdef SIM
  int num_err = 0;
#endif

  //loops to go through yolo layer output
  for(i = 0; i < 3; i++) {
    for(j = 0; j < w; j++) {
      for(k = 0; k < w; k++) {

        //sigmoid of objectness score
        obj_score = sigmoid(input[(4+85*i)*w*w + j*w + k]);

        //check if objectness score is above threshold
        if(obj_score > threshold) {

          //Calculate x
          val_32 = (int32_t)((int32_t)(sigmoid(input[85*i*w*w + j*w + k]) + (k<<8))*(int32_t)xy_div); //Q8.8 *Q1.15 = Q9.23
        #ifdef SIM
          if(output[84*nboxes] != (int16_t)(val_32 >> 9)) num_err++;
        #else
          output[84*nboxes] = (int16_t)(val_32 >> 9); //Q9.23 to Q2.14
        #endif

          //Calculate y
          val_32 = (int32_t)((int32_t)(sigmoid(input[(1+85*i)*w*w + j*w + k]) + (j<<8))*(int32_t)xy_div); //Q8.8 *Q1.15 = Q9.23
          val_32 = (int32_t)((int32_t)(int16_t)(val_32 >> 9)*(int32_t)y_scales); //Q2.14 * Q2.14 = Q4.28
        #ifdef SIM
          if(output[84*nboxes+1] != (int16_t)(val_32 >> 14) - (int16_t)y_bias) num_err++;
        #else
          output[84*nboxes+1] = (int16_t)(val_32 >> 14) - (int16_t)y_bias; //Q4.28 to Q2.14
        #endif

          //Calculate w
          val_32 = (int32_t)((int32_t)exp_fnc(input[(2+85*i)*w*w + j*w + k])*(int32_t)w_scales); //Q2.14 * Q2.14 = Q4.28
          val_32 = (int32_t)((int32_t)(int16_t)(val_32 >> 14)*(int32_t)yolo_bias[2*(i+(1-first_yolo)+3*first_yolo)]); //Q2.14 * Q10.6 = Q12.20
        #ifdef SIM
          if(output[84*nboxes+2] != (int16_t)(val_32 >> 6)) num_err++;
        #else
          output[84*nboxes+2] = (int16_t)(val_32 >> 6); //Q12.20 to Q2.14
        #endif

          //Calculate h
          val_32 = (int32_t)((int32_t)exp_fnc(input[(3+85*i)*w*w + j*w + k])*(int32_t)h_scales); //Q2.14 * Q2.14 = Q4.28
          val_32 = (int32_t)((int32_t)(int16_t)(val_32 >> 14)*(int32_t)yolo_bias[2*(i+(1-first_yolo)+3*first_yolo)+1]); //Q2.14 * Q10.6 = Q12.20
        #ifdef SIM
          if(output[84*nboxes+3] != (int16_t)(val_32 >> 6)) num_err++;
        #else
          output[84*nboxes+3] = (int16_t)(val_32 >> 6); //Q12.20 to Q2.14
        #endif

          //Calculate probability scores
          for(m = 0; m < 80; m++) {
            val_32 = (int32_t)((int32_t)sigmoid(input[(5+m+85*i)*w*w + j*w + k])*(int32_t)obj_score<<6); //Q8.8 * Q2.14 = Q10.22
            pred_score = (int16_t)(val_32 >> 8); //Q10.22 to Q2.14
            if(pred_score <= (threshold << 6)) pred_score = 0; //Q2.14
        #ifdef SIM
            if(output[84*nboxes+4+m] != pred_score) num_err++;
        #else
            output[84*nboxes+4+m] = pred_score; // prediction scores
        #endif
          }

          //Update number of candidate boxes
          nboxes++;
        }
      }
    }
  }
#ifdef SIM
  printf("Layer done with %d errors\n", num_err);
#endif
}

//send detection results back
void send_data(unsigned int pos) {

  //Loop to send output of yolo layer
  int i, j;
  count_bytes = 0;
  char * fp_data_char = (char *) fp_data + pos*2;
  for(j = 0; j < NUM_OUTPUT_FRAMES+1; j++) {

     // start timer
     if(j == 0) start = timer_time_us();

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
  end = timer_time_us();
  printf("\noutput layer transferred in %d ms\n\n", (end-start)/1000);
}

void run() {

  //send init message
  printf("\nYOLO HW FULL\n\n");

#ifndef SIM
  //init ETHERNET
  eth_init(ETHERNET_BASE);
  eth_set_rx_payload_size(ETH_NBYTES);
#endif

  //init VERSAT
  versat_init(VERSAT_BASE);

  //define memory regions
  define_memory_regions();

  //Stores initial address of layers 9/10/14/19 output
  unsigned int data_pos_layer10 = data_pos + NETWORK_INPUT + DATA_LAYER_2 + DATA_LAYER_4 + DATA_LAYER_6 + DATA_LAYER_8;
  unsigned int data_pos_layer14 = data_pos_layer10 + DATA_LAYER_10 + DATA_LAYER_11 + DATA_LAYER_12 + DATA_LAYER_13;
  unsigned int data_pos_layer19 = data_pos_layer14 + DATA_LAYER_14 + DATA_LAYER_15 + DATA_LAYER_16;
  unsigned int data_pos_layer9 = data_pos_layer19 + DATA_LAYER_19 + DATA_LAYER_20;
  unsigned int box_pos = data_pos_layer9 + DATA_LAYER_9 + DATA_LAYER_22 + DATA_LAYER_23;

  //load data and reset DDR to zero
#ifndef SIM
  reset_DDR();
  receive_data();
  fill_grey();

  //initialize ix, iy, dx and dy arrays
  prepare_resize();

  //resize input image
  printf("\nResizing input image...\n");
  resize_image();
#endif

  //layer1,2 (conv + maxpool)
  printf("\nRunning layers 1 and 2...\n");
  conv_layer(YOLO_INPUT, IMG_C, LAYER_1_NUM_KER, LAYER_1_KER_SIZE, LAYER_1_TILING_W, LAYER_1_TILING_H, LAYER_1_MAXPOOL, LAYER_1_OUTPADD, LAYER_1_STRIDE, 1, LAYER_1_IGNOREPAD, 0);

  //layer3,4 (conv + maxpool)
  printf("\nRunning layers 3 and 4...\n");
  conv_layer(LAYER_3_W, LAYER_1_NUM_KER, LAYER_3_NUM_KER, LAYER_3_KER_SIZE, LAYER_3_TILING_W, LAYER_3_TILING_H, LAYER_3_MAXPOOL, LAYER_3_OUTPADD, LAYER_3_STRIDE, LAYER_1_OUTPADD, LAYER_3_IGNOREPAD, 0);

  //layer5,6 (conv + maxpool)
  printf("\nRunning layers 5 and 6...\n");
  conv_layer(LAYER_5_W, LAYER_3_NUM_KER, LAYER_5_NUM_KER, LAYER_5_KER_SIZE, LAYER_5_TILING_W, LAYER_5_TILING_H, LAYER_5_MAXPOOL, LAYER_5_OUTPADD, LAYER_5_STRIDE, LAYER_3_OUTPADD, LAYER_5_IGNOREPAD, 0);

  //layer7,8 (conv + maxpool)
  printf("\nRunning layers 7 and 8...\n");
  conv_layer(LAYER_7_W, LAYER_5_NUM_KER, LAYER_7_NUM_KER, LAYER_7_KER_SIZE, LAYER_7_TILING_W, LAYER_7_TILING_H, LAYER_7_MAXPOOL, LAYER_7_OUTPADD, LAYER_7_STRIDE, LAYER_5_OUTPADD, LAYER_7_IGNOREPAD, 0);

  //layer9 (conv)
  printf("\nRunning layer 9...\n");
  conv_layer(LAYER_9_W, LAYER_7_NUM_KER, LAYER_9_NUM_KER, LAYER_9_KER_SIZE, LAYER_9_TILING_W, LAYER_9_TILING_H, LAYER_9_MAXPOOL, LAYER_9_OUTPADD, LAYER_9_STRIDE, LAYER_7_OUTPADD, LAYER_9_IGNOREPAD, data_pos_layer9);

  //layer10 (maxpool)
  printf("\nRunning layer 10...\n");
  maxpool_layer(LAYER_9_W, LAYER_9_NUM_KER, LAYER_10_INPADD, LAYER_10_STRIDE, data_pos_layer10);

  //layer11 (conv)
  printf("\nRunning layer 11...\n");
  conv_layer(LAYER_11_W, LAYER_9_NUM_KER, LAYER_11_NUM_KER, LAYER_11_KER_SIZE, LAYER_11_TILING_W, LAYER_11_TILING_H, LAYER_11_MAXPOOL, LAYER_11_OUTPADD, LAYER_11_STRIDE, 1, LAYER_11_IGNOREPAD, 0);

  //layer12 (maxpool)
  printf("\nRunning layer 12...\n");
  maxpool_layer(LAYER_11_W, LAYER_11_NUM_KER, LAYER_12_INPADD, LAYER_12_STRIDE, 0);

  //layer13 (conv)
  printf("\nRunning layer 13...\n");
  conv_layer(LAYER_13_W, LAYER_11_NUM_KER, LAYER_13_NUM_KER, LAYER_13_KER_SIZE, LAYER_11_TILING_W, LAYER_11_TILING_H, LAYER_13_MAXPOOL, LAYER_13_OUTPADD, LAYER_13_STRIDE, 1, LAYER_13_IGNOREPAD, 0);

  //layer14 (conv)
  printf("\nRunning layer 14...\n");
  conv_layer(LAYER_14_W, LAYER_13_NUM_KER, LAYER_14_NUM_KER, LAYER_14_KER_SIZE, LAYER_11_TILING_W, LAYER_11_TILING_H, LAYER_14_MAXPOOL, LAYER_14_OUTPADD, LAYER_14_STRIDE, LAYER_13_OUTPADD, LAYER_14_IGNOREPAD, 0);

  //layer15 (conv)
  printf("\nRunning layer 15...\n");
  conv_layer(LAYER_15_W, LAYER_14_NUM_KER, LAYER_15_NUM_KER, LAYER_15_KER_SIZE, LAYER_11_TILING_W, LAYER_11_TILING_H, LAYER_15_MAXPOOL, LAYER_15_OUTPADD, LAYER_15_STRIDE, LAYER_14_OUTPADD, LAYER_15_IGNOREPAD, 0);

  //layer16 (conv)
  printf("\nRunning layer 16...\n");
  conv_layer(LAYER_16_W, LAYER_15_NUM_KER, LAYER_16_NUM_KER, LAYER_16_KER_SIZE, LAYER_16_TILING_W, LAYER_16_TILING_H, LAYER_16_MAXPOOL, LAYER_16_OUTPADD, LAYER_16_STRIDE, LAYER_15_OUTPADD, LAYER_16_IGNOREPAD, 0);

  //layer17 (yolo)
  start = timer_time_us();
  printf("\nRunning layer 17...\n");
  yolo_layer(LAYER_16_W, yolo1_div, 1, data_pos, box_pos);
  end = timer_time_us();
  printf("Total time = %d us\n", end-start);

  //layer19 (conv)
  data_pos = data_pos_layer14;
  printf("\nRunning layer 19...\n");
  conv_layer(LAYER_19_W, LAYER_14_NUM_KER, LAYER_19_NUM_KER, LAYER_19_KER_SIZE, LAYER_16_TILING_W, LAYER_16_TILING_H, LAYER_19_MAXPOOL, LAYER_19_OUTPADD, LAYER_19_STRIDE, 0, LAYER_19_IGNOREPAD, data_pos_layer19);

  //layer20 (upsample)
  printf("\nRunning layer 20...\n");
  upsample();

  //layer22 (conv)
  printf("\nRunning layer 22...\n");
  conv_layer(LAYER_22_W, LAYER_9_NUM_KER+LAYER_19_NUM_KER, LAYER_22_NUM_KER, LAYER_22_KER_SIZE, LAYER_22_TILING_W, LAYER_22_TILING_H, LAYER_22_MAXPOOL, LAYER_22_OUTPADD, LAYER_22_STRIDE, 1, LAYER_22_IGNOREPAD, 0);

  //layer23 (conv)
  printf("\nRunning layer 23...\n");
  conv_layer(LAYER_23_W, LAYER_22_NUM_KER, LAYER_23_NUM_KER, LAYER_23_KER_SIZE, LAYER_23_TILING_W, LAYER_23_TILING_H, LAYER_23_MAXPOOL, LAYER_23_OUTPADD, LAYER_23_STRIDE, LAYER_22_OUTPADD, LAYER_23_IGNOREPAD, 0);

  //layer24 (yolo)
  start = timer_time_us();
  printf("\nRunning layer 24...\n");
  yolo_layer(LAYER_23_W, yolo2_div, 0, data_pos, box_pos);
  end = timer_time_us();
  printf("Total time = %d us\n", end-start);

  //return data
#ifndef SIM
  printf("\nTotal config time = %d us\n", total_config_time);
  printf("Total run time = %d ms\n", total_run_time/1000);
  printf("Total data time = %d ms\n", total_data_time/1000);
  printf("Total execution time = %d ms\n", total_time);
  send_data(box_pos);
#endif

  ///////////////////////////////////////////////////////////////////////////////////////
  // test xyolo accumulator function by performing 3x3x3 convolution in 3 configs
  ///////////////////////////////////////////////////////////////////////////////////////
  globalClearConf();

  //transfer pixels and weights to versat
  int16_t i, pixels[27], weights[27], exp_res = 0;
  for(i = 0; i < 3*3*3; i++) {
    pixels[i] = rand()%50-25;
    weights[i] = rand()%10-5;
    stage[0].memA[0].write(i, pixels[i]);
    stage[0].memA[1].write(i, weights[i]);
    exp_res += pixels[i] * weights[i];
  }

  //first configuration
  stage[0].memA[0].setDuty(9);
  stage[0].memA[0].setPer(9);
  stage[0].memA[0].setIncr(1);
  stage[0].memA[0].setIter(1);
  stage[0].memA[1].setDuty(9);
  stage[0].memA[1].setPer(9);
  stage[0].memA[1].setIncr(1);
  stage[0].memA[1].setIter(1);
  stage[0].yolo[0].setSelA(sMEMA[0]);
  stage[0].yolo[0].setSelB(sMEMA[1]);
  stage[0].yolo[0].setPer(9);
  stage[0].yolo[0].setIter(1);
  stage[0].yolo[0].setDelay(MEMP_LAT);
  run();
  while(done()==0);

  //second configuration
  stage[0].memA[0].setStart(9);
  stage[0].memA[1].setStart(9);
  stage[0].yolo[0].setAcc(1);
  run();
  while(done()==0);

  //third configuration
  stage[0].memA[0].setStart(18);
  stage[0].memA[1].setStart(18);
  stage[0].memA[3].setDuty(1);
  stage[0].memA[3].setDelay(MEMP_LAT+YOLO_LAT-1);
  stage[0].memA[3].setPer(9);
  stage[0].memA[3].setIter(1);
  stage[0].memA[3].setSel(sYOLO[0]);
  stage[0].memA[3].setInWr(1);
  run();
  while(done()==0);

  //check result
  if(stage[0].memA[3].read(0) == exp_res) printf("xyolo acc feature tested successfully!\n");
  else printf("xyolo acc feature failed!\nExpected: %d, got %d\n", exp_res, stage[0].memA[3].read(0));

  return;
}
