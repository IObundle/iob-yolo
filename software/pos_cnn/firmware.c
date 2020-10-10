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

//yolo constants
#define LAYER_16_W 13
#define LAYER_23_W 26
#define YOLO_INPUT 416

//constants for bounding boxes
#define threshold ((int16_t)(((float)0.5)*((int32_t)1<<13))) //Q3.13
#define nms_threshold ((int16_t)(((float)0.45)*((int32_t)1<<13))) //Q3.13
#define yolo1_div ((int16_t)(((float)1/LAYER_16_W)*((int32_t)1<<15))) //Q1.15
#define yolo2_div ((int16_t)(((float)1/LAYER_23_W)*((int32_t)1<<15))) //Q1.15
#define x_scales ((int16_t)(((float)YOLO_INPUT/NEW_W)*((int32_t)1<<14))) //Q2.14
#define x_bias ((int16_t)(((float)(YOLO_INPUT-NEW_W)/(NEW_W*2))*((int32_t)1<<13))) //Q3.13
#define y_scales ((int16_t)(((float)YOLO_INPUT/NEW_H)*((int32_t)1<<14))) //Q2.14
#define y_bias ((int16_t)(((float)(YOLO_INPUT-NEW_H)/(NEW_H*2))*((int32_t)1<<13))) //Q3.13
#define w_scales ((int16_t)(((float)1/NEW_W)*((int32_t)1<<15))) //Q1.15
#define h_scales ((int16_t)(((float)1/NEW_H)*((int32_t)1<<15))) //Q1.15
#define c3 ((int16_t)0x0AAA) // pow(2,-3)+pow(2,-5)+pow(2,-7)+pow(2,-9)+pow(2,-11)+pow(2,-13) in Q2.14
#define c4 ((int16_t)0x02C0) // pow(2,-5)+pow(2,-7)+pow(2,-8) in Q2.14
#define MAX_NUM_BOXES 10
#define box_width 3
#define label_height 20

//rgb values constants - 16bit values
// one line for each class: 00000000000 B[j] G[j] R[j]
// 81st line with:          00000000000 1    1    1
#define RGB_LINE 16
#define RGB_VALUES_SIZE (RGB_LINE*81)

//label constants
#define MAX_LABEL_SIZE 2340
#define LABEL_LINES 20
#define LABEL_W_OFF (81+15)
//Original MAX_LABEL_SIZE/20*4 + 12 values of padding 
//each 4 bytes have 3x label_px_value and 0, for padding channel 
#define LABEL_LINE_SIZE (117*4 + 12)
#define LABEL_SIZE (9600) // LABEL_LINE_SIZE * LABEL_LINES
#define LABELS_FILE_SIZE (LABEL_W_OFF + LABEL_SIZE*81) //label_w + labels for versat 16bit values

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

//define ethernet constants
#define ETH_NBYTES (1024-18) //minimum ethernet payload excluding FCS
#define INPUT_FILE_SIZE ((RGB_VALUES_SIZE + LABELS_FILE_SIZE + IMAGE_INPUT + 13*13*256 + 26*26*256)*2) //16 bits
#define NUM_INPUT_FRAMES (INPUT_FILE_SIZE/ETH_NBYTES)
#define OUTPUT_FILE_SIZE (IMAGE_INPUT) //8 bits
#define NUM_OUTPUT_FRAMES (OUTPUT_FILE_SIZE/ETH_NBYTES)

//define DDR mapping
#define RGB_BASE_ADDRESS (DDR_MEM + (1 << (FIRM_ADDR_W))) //after main mem
#define LABEL_BASE_ADDRESS (RGB_BASE_ADDRESS + 2*RGB_VALUES_SIZE)
#define INPUT_IMAGE_BASE_ADDRESS (LABEL_BASE_ADDRESS + 2*LABELS_FILE_SIZE)
#define DATA_BASE_ADDRESS (INPUT_IMAGE_BASE_ADDRESS + 2*IMAGE_INPUT)

//ETHERNET variables
int rcv_timeout = 5000;
char data_to_send[ETH_NBYTES];
char data_rcv[ETH_NBYTES+18];
unsigned int bytes_to_receive, bytes_to_send, count_bytes;

//TIMER variables
unsigned int start, end;

//input image pointer
int16_t * fp_rgb = (int16_t *) RGB_BASE_ADDRESS;
int16_t * fp_labels = (int16_t *) LABEL_BASE_ADDRESS;
int16_t * fp_image = (int16_t *) INPUT_IMAGE_BASE_ADDRESS;

//receive weigths and resized padded image
void rcv_data() {

  //Local variables
  int i, j;
  count_bytes = 0;
  char * data_p = (char *) RGB_BASE_ADDRESS;
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
  exp_val_fixed = val + 0x2000; //1+w -> Q3.13
  val_32 = (int32_t)((int32_t)val*(int32_t)val); //w^2 -> Q3.13*Q3.13 = Q6.26
  val_16 = (int16_t)(val_32 >> 13); //w^2 -> Q6.26 to Q3.13
  val_32 = (int32_t)((int32_t)0x2000*(int32_t)val_16); //0.5*w^2 -> Q2.14*Q3.13 = Q5.27
  exp_val_fixed += (int16_t)(val_32 >> 14); //1+w+0.5*w^2 -> Q5.27 to Q3.13
  val_32 = (int32_t)((int32_t)val_16*(int32_t)val); //w^3 -> Q3.13*Q3.13 = Q6.26
  val_16 = (int16_t)(val_32 >> 13); //w^3 -> Q6.26 to Q3.13
  val_32 = (int32_t)((int32_t)c3*(int32_t)val_16); //c3*w^3 -> Q2.14*Q3.13 = Q5.27
  exp_val_fixed += (int16_t)(val_32 >> 14); //1+w+0.5*w^2+c3*w^3 -> Q5.27 to Q3.13
  val_32 = (int32_t)((int32_t)val_16*(int32_t)val); //w^4 -> Q3.13*Q3.13 = Q6.26
  val_16 = (int16_t)(val_32 >> 13); //w^4 -> Q6.26 to Q3.13
  val_32 = (int32_t)((int32_t)c4*(int32_t)val_16); //c4*w^4 -> Q2.14*Q3.13 = Q5.27
  exp_val_fixed += (int16_t)(val_32 >> 14); //1+w+0.5*w^2+c3*w^3+c4*w^4 -> Q5.27 to Q3.13
  return exp_val_fixed; //Q3.13
}

// create boxes
void create_boxes(int w, unsigned int pos, int16_t xy_div, int first_yolo) {

  //local variable
  int16_t i, j, k, m, n, n_start, n_end;
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
	  val_32 = (int32_t)((int32_t)(fp_data[i*w*256 + j*16 + 5*k*w*16 + 5*k] + (j<<13))*(int32_t)xy_div); //Q3.13 *Q1.15 = Q4.28
	  val_16 = (int16_t)(val_32 >> 15); //Q4.28 to Q3.13
	  val_32 = (int32_t)((int32_t)val_16*(int32_t)x_scales); //Q3.13 * Q2.14 = Q5.27
	  val_16 = (int16_t)(val_32 >> 14); //Q5.27 to Q3.13
	  val_16 -= (int16_t)x_bias; //Q3.13
	  boxes[84*nboxes] = val_16; //x
          
	  //Calculate y
	  val_32 = (int32_t)((int32_t)(fp_data[i*w*256 + j*16 + 5*k*w*16 + 5*k + 1] + (i<<13))*(int32_t)xy_div); //Q3.13 *Q1.15 = Q4.28
	  val_16 = (int16_t)(val_32 >> 15); //Q4.28 to Q3.13
	  val_32 = (int32_t)((int32_t)val_16*(int32_t)y_scales); //Q3.13 * Q2.14 = Q5.27
	  val_16 = (int16_t)(val_32 >> 14); //Q5.27 to Q3.13
	  val_16 -= (int16_t)y_bias; //Q3.13
	  boxes[84*nboxes+1] = val_16; //y

	  //Calculate w
	  val_32 = (int32_t)((int32_t)exp_fnc(fp_data[i*w*256 + j*16 + 5*k*w*16 + 5*k + 2])*(int32_t)w_scales); //Q3.13 * Q1.15 = Q4.28
	  val_16 = (int16_t)(val_32 >> 15); //Q4.28 to Q3.13
	  val_32 = (int32_t)((int32_t)val_16*(int32_t)yolo_bias[2*(k+(1-first_yolo)+3*first_yolo)]); //Q3.13 * Q10.6 = Q13.19 -> mask 1,2,3
	  boxes[84*nboxes+2] = (int16_t)(val_32 >> 6); //Q13.19 to Q3.13

	  //Calculate h
	  val_32 = (int32_t)((int32_t)exp_fnc(fp_data[i*w*256 + j*16 + 5*k*w*16 + 5*k + 3])*(int32_t)h_scales); //Q2.14 * Q2.14 = Q4.28
	  val_16 = (int16_t)(val_32 >> 15); //Q4.28 to Q3.13
	  val_32 = (int32_t)((int32_t)val_16*(int32_t)yolo_bias[2*(k+(1-first_yolo)+3*first_yolo)+1]); //Q3.13 * Q10.6 = Q13.19 -> mask 1,2,3
	  boxes[84*nboxes+3] = (int16_t)(val_32 >> 6); //Q13.19 to Q3.13

          //check if prob score is higher than threshold
          n_start = 5*k + 5;
	  n_end = 16;
          for(m = 0; m < 6; m++) {
            for(n = n_start; n < n_end; n++) {
              val_32 = (int32_t)((int32_t)fp_data[i*w*256 + j*16 + 5*k*w*16 + m*w*16 + n]*(int32_t)obj_score); //Q3.13 * Q3.13 = Q6.26
              val_16 = (int16_t)(val_32 >> 13); //Q6.26 to Q3.13
	      if(val_16 > threshold) boxes[84*nboxes+4+m*16+n-(5*k+5)] = val_16;
 	      n_start = 0;
	    }
	    if(m == 4) n_end = 5*k + 5;
	  }

	  //Update number of candidate boxes
	  nboxes++;
	  uart_printf("nboxes: %d\n", nboxes);
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
	  w = overlap(x1, w1, x2, w2); //Q3.13
	  h = overlap(y1, h1, y2, h2); //Q3.13
	  if(w > 0 && h > 0) {
	    b_inter = (int32_t)((int32_t)w*(int32_t)h); //Q3.13 * Q3.13 = Q6.26
	    mul_32 = (int32_t)((int32_t)w1*(int32_t)h1); //Q3.13 * Q3.13 = Q6.26
	    b_union = (int16_t)(mul_32 >> 13); //w1*h1 -> Q6.26 to Q3.13
	    mul_32 = (int32_t)((int32_t)w2*(int32_t)h2); //Q3.13 * Q3.13 = Q6.26
	    b_union += (int16_t)(mul_32 >> 13); //w1*h1+w2*h2 -> Q6.26 to Q3.13
	    b_union -= (int16_t)(b_inter >> 13); //w1*h1+w2*h2-inter -> Q6.26 to Q3.13
	    b_iou = (int16_t)((int32_t)b_inter/(int32_t)b_union); //Q6.26 / Q3.13 = Q3.13
	    if(b_iou > nms_threshold) boxes[84*box_IDs[k]+4+i] = 0;
	  }
	}
      }
    }
  }					
}

//Draw bounding box in input image using versat
void draw_box_versat(int left, int top, int right, int bot, int rgb, int box_w) {

  int l=0;
  int line_w = 0, h_right=0;
  
  //Limit box coordinates
  if(left < 0) left = 0; else if(left >= IMG_W) left = IMG_W-1;
  if(right < 0) right = 0; else if(right >= IMG_W) right = IMG_W-1;
  if(top < 0) top = 0; else if(top >= IMG_H) top = IMG_H-1;
  if(bot < 0) bot = 0; else if(bot >= IMG_H) bot = IMG_H-1;

  line_w = right - left + 1;
  h_right = right - box_w +1;

  //clear configurations
  versat.clear();
  // VERSAT CONFIGURATIONS

  // yread ext: read 1 rgb line with 1's
  versat.yread.setExtAddr((int) (&(fp_rgb[16*80]))); //16x2Bytes to jump a 32Byte line
  versat.yread.setOffset(0); // read same line to all yread memories
  versat.yread.setPingPong(0);
  versat.yread.setIntAddr(0);
  versat.yread.setExtIter(1);
  versat.yread.setExtPer(1); // only one MIG_BUS_W transfer
  versat.yread.setExtShift(0);
  versat.yread.setExtIncr(16); // does this really matter?
  versat.dma.yread_setLen(0);

  // yread int: send first line to xyolo
  versat.yread.setIntIter((line_w*IMG_C)/nYOLOmacs);
  versat.yread.setIntPer(2);
  versat.yread.setIntShift(0);
  versat.yread.setIntStart(0);
  versat.yread.setIntIncr(0);
  versat.yread.setIntDelay(0);

  // yread ext: read 1 rgb line with 1's
  versat.ywrite.read.setExtAddr((int) (&(fp_rgb[16*rgb]))); //16x2Bytes to jump a 32Byte line
  versat.ywrite.read.setOffset(0); // read same line to all yread memories
  versat.ywrite.read.setPingPong(0);
  versat.ywrite.read.setIntAddr(0);
  versat.ywrite.read.setExtIter(1);
  versat.ywrite.read.setExtPer(1); // only one MIG_BUS_W transfer
  versat.ywrite.read.setExtShift(0);
  versat.ywrite.read.setExtIncr(16); // does this really matter?
  versat.dma.ywrite_read_setLen(0);

  // ywrite read int: send 1 line every 2 cycles
  versat.ywrite.read.setIntStart(0);
  versat.ywrite.read.setIntIter((line_w*IMG_C)/nYOLOmacs);
  versat.ywrite.read.setIntPer(2);
  versat.ywrite.read.setIntShift(0);
  versat.ywrite.read.setIntIncr(0);
  versat.ywrite.read.setIntIter2(0);
  versat.ywrite.read.setIntPer2(0);
  versat.ywrite.read.setIntShift2(0);
  versat.ywrite.read.setIntIncr2(0);
  versat.ywrite.read.setIntIter3(0);
  versat.ywrite.read.setIntPer3(0);
  versat.ywrite.read.setIntShift3(0);
  versat.ywrite.read.setIntIncr3(0);

  // xyolo: multiply and bypass adder
  versat.ywrite.yolo.setIter((line_w*IMG_C)/nYOLOmacs);
  versat.ywrite.yolo.setPer(2);
  versat.ywrite.yolo.setShift(0);
  versat.ywrite.yolo.setBias(0);
  versat.ywrite.yolo.setLeaky(0);
  versat.ywrite.yolo.setSigmoid(0);
  versat.ywrite.yolo.setSigMask(0);
  versat.ywrite.yolo.setMaxpool(0);
  versat.ywrite.yolo.setBypass(0);
  versat.ywrite.yolo.setBypassAdder(1);

  // ywrite int: write results from xyolo
  versat.ywrite.write.setIntStart(0);
  versat.ywrite.write.setIntDuty(2*(nYOLOvect/IMG_C));
  versat.ywrite.write.setIntDelay(XYOLO_READ_LAT + XYOLO_WRITE_LAT - 2);
  versat.ywrite.write.setIntIter((line_w*IMG_C)/nYOLOmacs);
  versat.ywrite.write.setIntPer(2*(nYOLOvect/IMG_C));
  versat.ywrite.write.setIntShift(1);
  versat.ywrite.write.setIntIncr(0);

  // ywrite ext: write result burst
  versat.ywrite.write.setOffset(0);
  versat.ywrite.write.setIntAddr(0);
  versat.ywrite.write.setExtIter(1);
  versat.ywrite.write.setExtPer((line_w*IMG_C+15)/16); //ceil(x/y) = (x+y-1)/y
  versat.ywrite.write.setExtShift(0);
  versat.ywrite.write.setExtIncr(16);
  versat.dma.ywrite_write_setNBytesW(2*line_w*IMG_C);

  // draw horizontal lines
  for(l = 0; l < box_w; l++) {
    //top line
    versat.ywrite.write.setExtAddr((int)(&(fp_image[(top+l)*IMG_W*IMG_C+left*IMG_C]))); //fp_image position
    
    //wait until done
    while(versat.done() == 0);
    /* uart_printf("line %d\n", l); */
    //run configuration
    versat.run();
    
    //stop yread transfers
    versat.yread.setExtIter(0);
    versat.ywrite.read.setExtIter(0);

    //bottom line
    versat.ywrite.write.setExtAddr((int) (&(fp_image[(bot-l)*IMG_W*IMG_C+left*IMG_C])));

    //wait until done
    while(versat.done() == 0);
    /* uart_printf("line %d\n", l); */
    //run configuration
    versat.run();

  }

  //vertical lines
  //VERSAT CONFIGURATIONS
  // yread int: send first line to xyolo
  versat.yread.setIntIter((box_w*IMG_C)/nYOLOmacs);
  versat.yread.setIntPer(2);
  versat.yread.setIntShift(0);
  versat.yread.setIntStart(0);
  versat.yread.setIntIncr(0);
  versat.yread.setIntDelay(0);

  // ywrite read int: send 1 line every 2 cycles
  versat.ywrite.read.setIntStart(0);
  versat.ywrite.read.setIntIter((box_w*IMG_C)/nYOLOmacs);
  versat.ywrite.read.setIntPer(2);
  versat.ywrite.read.setIntShift(0);
  versat.ywrite.read.setIntIncr(0);
  versat.ywrite.read.setIntIter2(0);
  versat.ywrite.read.setIntPer2(0);
  versat.ywrite.read.setIntShift2(0);
  versat.ywrite.read.setIntIncr2(0);
  versat.ywrite.read.setIntIter3(0);
  versat.ywrite.read.setIntPer3(0);
  versat.ywrite.read.setIntShift3(0);
  versat.ywrite.read.setIntIncr3(0);

  // xyolo: multiply and bypass adder
  versat.ywrite.yolo.setIter((box_w*IMG_C)/nYOLOmacs);
  versat.ywrite.yolo.setPer(2);
  versat.ywrite.yolo.setShift(0);
  versat.ywrite.yolo.setBias(0);
  versat.ywrite.yolo.setLeaky(0);
  versat.ywrite.yolo.setSigmoid(0);
  versat.ywrite.yolo.setSigMask(0);
  versat.ywrite.yolo.setMaxpool(0);
  versat.ywrite.yolo.setBypass(0);
  versat.ywrite.yolo.setBypassAdder(1);

  // ywrite int: write results from xyolo
  versat.ywrite.write.setIntStart(0);
  versat.ywrite.write.setIntDuty(2*(nYOLOvect/IMG_C));
  versat.ywrite.write.setIntDelay(XYOLO_READ_LAT + XYOLO_WRITE_LAT - 2);
  versat.ywrite.write.setIntIter((box_w*IMG_C)/nYOLOmacs);
  versat.ywrite.write.setIntPer(2*(nYOLOvect/IMG_C));
  versat.ywrite.write.setIntShift(1);
  versat.ywrite.write.setIntIncr(0);

  // ywrite ext: write result burst
  versat.ywrite.write.setOffset(0);
  versat.ywrite.write.setIntAddr(0);
  versat.ywrite.write.setExtIter(1);
  versat.ywrite.write.setExtPer((box_w*IMG_C+15)/16); //ceil(x/y) = (x+y-1)/y
  versat.ywrite.write.setExtShift(0);
  versat.ywrite.write.setExtIncr(16);
  versat.dma.ywrite_write_setNBytesW(2*box_w*IMG_C);

  // all vertical lines
  for(l = top+box_w; l <= bot-box_w; l++) {
    //left line
    versat.ywrite.write.setExtAddr((int)(&(fp_image[l*IMG_W*IMG_C+left*IMG_C]))); //fp_image position
    
    //wait until done
    while(versat.done() == 0);
    /* uart_printf("line %d\n", l); */
    //run configuration
    versat.run();
    
    //bottom line
    versat.ywrite.write.setExtAddr((int) (&(fp_image[l*IMG_W*IMG_C+h_right*IMG_C])));

    //wait until done
    while(versat.done() == 0);
    /* uart_printf("line %d\n", l); */
    //run configuration
    versat.run();

  }

  

  //clear configurations
  versat.clear();

  //final runs
  versat_end();

}

//Draw bounding box in input image
void draw_box(int left, int top, int right, int bot, int16_t red, int16_t green, int16_t blue) {

  //Limit box coordinates
  if(left < 0) left = 0; else if(left >= IMG_W) left = IMG_W-1;
  if(right < 0) right = 0; else if(right >= IMG_W) right = IMG_W-1;
  if(top < 0) top = 0; else if(top >= IMG_H) top = IMG_H-1;
  if(bot < 0) bot = 0; else if(bot >= IMG_H) bot = IMG_H-1;

  //Draw horizontally
  int i;
  for(i = left; i <= right; i++) {
    fp_image[top*IMG_W*IMG_C + i*IMG_C] = red;
    fp_image[top*IMG_W*IMG_C + i*IMG_C + 1] = green;
    fp_image[top*IMG_W*IMG_C + i*IMG_C + 2] = blue;
    fp_image[bot*IMG_W*IMG_C + i*IMG_C] = red;
    fp_image[bot*IMG_W*IMG_C + i*IMG_C + 1] = green;
    fp_image[bot*IMG_W*IMG_C + i*IMG_C + 2] = blue;
  }

  //Draw vertically
  for(i = top; i <= bot; i++) {
    fp_image[i*IMG_W*IMG_C + left*IMG_C] = red;
    fp_image[i*IMG_W*IMG_C + left*IMG_C + 1] = green;
    fp_image[i*IMG_W*IMG_C + left*IMG_C + 2] = blue;
    fp_image[i*IMG_W*IMG_C + right*IMG_C] = red;
    fp_image[i*IMG_W*IMG_C + right*IMG_C + 1] = green;
    fp_image[i*IMG_W*IMG_C + right*IMG_C + 2] = blue;
  }
}

// draw class using versat
void draw_class_versat(int label_w, int j, int top_width, int left, int previous_w, int rgb){
  int l;
  
  /* uart_printf("draw_class_versat %d\n", j); */

  //clear configurations
  versat.clear();
  // VERSAT CONFIGURATIONS

  // yread ext: read 1 rgb line for the j class
  versat.yread.setExtAddr((int) (&(fp_rgb[16*rgb]))); //16x2Bytes to jump a 32Byte line
  versat.yread.setOffset(0); // read same line to all yread memories
  versat.yread.setPingPong(0);
  versat.yread.setIntAddr(0);
  versat.yread.setExtIter(1);
  versat.yread.setExtPer(1); // only one MIG_BUS_W transfer
  versat.yread.setExtShift(0);
  versat.yread.setExtIncr(16); // does this really matter?
  versat.dma.yread_setLen(0);

  // yread int: send first line to xyolo
  versat.yread.setIntIter((label_w*IMG_C)/nYOLOmacs);
  versat.yread.setIntPer(2);
  versat.yread.setIntShift(0);
  versat.yread.setIntStart(0);
  versat.yread.setIntIncr(0);
  versat.yread.setIntDelay(0);

  // ywrite read ext: read label line
  versat.ywrite.read.setOffset(0); // only use 1st stage
  versat.ywrite.read.setPingPong(1);
  versat.ywrite.read.setIntAddr(0);
  versat.ywrite.read.setExtIter(1);
  versat.ywrite.read.setExtPer((label_w*IMG_C+15)/16); // ceil(x/y) = (x+y-1)/y
  versat.ywrite.read.setExtShift(0);
  versat.ywrite.read.setExtIncr(16);
  versat.dma.ywrite_read_setLen((label_w*IMG_C+15)/16-1); //ceil(x/y)-1

  // ywrite read int: send 1 line every 2 cycles
  versat.ywrite.read.setIntStart(0);
  versat.ywrite.read.setIntIter((label_w*IMG_C)/nYOLOmacs);
  versat.ywrite.read.setIntPer(2);
  versat.ywrite.read.setIntShift(1);
  versat.ywrite.read.setIntIncr(0);
  versat.ywrite.read.setIntIter2(0);
  versat.ywrite.read.setIntPer2(0);
  versat.ywrite.read.setIntShift2(0);
  versat.ywrite.read.setIntIncr2(0);
  versat.ywrite.read.setIntIter3(0);
  versat.ywrite.read.setIntPer3(0);
  versat.ywrite.read.setIntShift3(0);
  versat.ywrite.read.setIntIncr3(0);

  // xyolo: multiply and bypass adder
  versat.ywrite.yolo.setIter((label_w*IMG_C)/nYOLOmacs);
  versat.ywrite.yolo.setPer(2);
  versat.ywrite.yolo.setShift(8);
  versat.ywrite.yolo.setBias(0);
  versat.ywrite.yolo.setLeaky(0);
  versat.ywrite.yolo.setSigmoid(0);
  versat.ywrite.yolo.setSigMask(0);
  versat.ywrite.yolo.setMaxpool(0);
  versat.ywrite.yolo.setBypass(0);
  versat.ywrite.yolo.setBypassAdder(1);

  // ywrite int: write results from xyolo
  versat.ywrite.write.setIntStart(0);
  versat.ywrite.write.setIntDuty(2*(nYOLOvect/IMG_C));
  versat.ywrite.write.setIntDelay(XYOLO_READ_LAT + XYOLO_WRITE_LAT -2 - 2);
  versat.ywrite.write.setIntIter((label_w*IMG_C)/nYOLOmacs);
  versat.ywrite.write.setIntPer(2*(nYOLOvect/IMG_C));
  versat.ywrite.write.setIntShift(1);
  versat.ywrite.write.setIntIncr(0);

  // ywrite ext: write result burst
  versat.ywrite.write.setOffset(0);
  versat.ywrite.write.setIntAddr(0);
  versat.ywrite.write.setExtIter(1);
  versat.ywrite.write.setExtPer((label_w*IMG_C+15)/16); //ceil(x/y) = (x+y-1)/y
  versat.ywrite.write.setExtShift(0);
  versat.ywrite.write.setExtIncr(16);
  versat.dma.ywrite_write_setNBytesW(2*label_w*IMG_C);

  // each run writes a label line to fp_image
  for(l = 0; l < label_height && (l+top_width) < IMG_H; l++) {
    versat.ywrite.read.setExtAddr((int) (&(fp_labels[LABEL_W_OFF + LABEL_SIZE*j + LABEL_LINE_SIZE*l]))); // start of each label line
    versat.ywrite.write.setExtAddr((int)(&(fp_image[(l+top_width)*IMG_W*IMG_C+(left+previous_w)*IMG_C]))); //fp_image position
    
    //wait until done
    while(versat.done() == 0);
    /* uart_printf("line %d\n", l); */
    //run configuration
    versat.run();
    
    //stop yread transfers
    versat.yread.setExtIter(0);
  }

  //clear configurations
  versat.clear();

  //final runs
  versat_end();

}

//DEBUG: directly copy resulting boxes from memory
void copy_boxes(){
  int NBOXES = 8;
  int16_t * box_sol = (int16_t *) DATA_BASE_ADDRESS;
  box_sol += 256*13*13 + 256*26*26;

  nboxes = NBOXES;
  int i;
  for(i = 0; i < 84*nboxes; i++)
    boxes[i] = box_sol[i];

}


//Verify class label in input image
void verify_class(int label_w, int j, int top_width, int left, int previous_w, int16_t r, int16_t g, int16_t b) {
  uart_printf("Verifying class: %d\n", j);
  int l, k, err_cnt=0;
  uint16_t label;
  uint16_t red_val, green_val, blue_val;
  uint16_t red_im, green_im, blue_im;
  for(l = 0; l < label_height && (l+top_width) < IMG_H; l++) {
    uart_printf("Label line %d\n", l);
    for(k = 0; k < label_w && (k+left+previous_w) < IMG_W; k++) {
      /* uart_printf("Label col %d\n", k); */
      label = (uint16_t) fp_labels[LABEL_W_OFF+LABEL_SIZE*j+l*LABEL_LINE_SIZE+4*k];
      
      /* uart_printf("\tlabel\n"); */
      //Q8.0*Q8.0=Q16.0 to Q8.0 -> red
      red_im = fp_image[(l+top_width)*IMG_W*IMG_C+(k+left+previous_w)*IMG_C];
      /* uart_printf("\tred_im\n"); */
      
      //green
      green_im = fp_image[(l+top_width)*IMG_W*IMG_C+(k+left+previous_w)*IMG_C + 1];
      /* uart_printf("\tblue_im\n"); */

      //blue
      blue_im = fp_image[(l+top_width)*IMG_W*IMG_C+(k+left+previous_w)*IMG_C + 2];
      /* uart_printf("\tgreen_im\n"); */

      //Q8.0*Q8.0=Q16.0 to Q8.0 -> red
      red_val = ((uint16_t)((uint16_t)r*(uint16_t)label)) >> 8;
      //green
      green_val = ((uint16_t)((uint16_t)g*(uint16_t)label)) >> 8;
      //blue
      blue_val = ((uint16_t)((uint16_t)b*(uint16_t)label)) >> 8;

      /* uart_printf("\tval\n"); */
      //check fp_image for correct values
      if(red_val != red_im){
	err_cnt++;
	/* uart_printf("Exp: %d | Act: %d\n", red_val, red_im); */
      }
      /* uart_printf("\tred\n"); */
      if(green_val != green_im){
	err_cnt++;
	/* uart_printf("Exp: %d | Act: %d\n", green_val, green_im); */
      }
      /* uart_printf("\tgreen\n"); */
      if(blue_val != blue_im){
	err_cnt++;
	/* uart_printf("Exp: %d | Act: %d\n", blue_val, blue_im); */
      }

      /* uart_printf("\tchecks\n"); */
    }
  }
  /* uart_printf("Label with %d errors\n", err_cnt); */

  return;
}

//Draw class label in input image
void draw_class(int label_w, int j, int top_width, int left, int previous_w, int16_t r, int16_t g, int16_t b) {
  int l, k;
  uint16_t label;
  uint16_t val=0;
  for(l = 0; l < label_height && (l+top_width) < IMG_H; l++) {
    /* uart_printf("line: %x\n", l); */
    for(k = 0; k < label_w && (k+left+previous_w) < IMG_W; k++) {
      label = fp_labels[LABEL_W_OFF+LABEL_SIZE*j+l*LABEL_LINE_SIZE+4*k];
      //Q8.0*Q8.0=Q16.0 to Q8.0 -> red
      val = ((uint16_t)((uint16_t)r*(uint16_t)label)) >> 8;
      /* uart_printf("\t%x", val); */
      fp_image[(l+top_width)*IMG_W*IMG_C+(k+left+previous_w)*IMG_C] = val;
      /* fp_image[(l+top_width)*IMG_W*IMG_C+(k+left+previous_w)*IMG_C] = ((uint16_t)((uint16_t)r*(uint16_t)label)) >> 8; */
      //green
      val = ((uint16_t)((uint16_t)g*(uint16_t)label)) >> 8;
      /* uart_printf("\t%x", val); */
      fp_image[(l+top_width)*IMG_W*IMG_C+(k+left+previous_w)*IMG_C + 1] = val;
      fp_image[(l+top_width)*IMG_W*IMG_C+(k+left+previous_w)*IMG_C + 1] = ((uint16_t)((uint16_t)g*(uint16_t)label)) >> 8;
      //blue
      val = ((uint16_t)((uint16_t)b*(uint16_t)label)) >> 8;
      /* uart_printf("\t%x", val); */
      fp_image[(l+top_width)*IMG_W*IMG_C+(k+left+previous_w)*IMG_C + 2] = val;
      /* fp_image[(l+top_width)*IMG_W*IMG_C+(k+left+previous_w)*IMG_C + 2] = ((uint16_t)((uint16_t)b*(uint16_t)label)) >> 8; */
      /* uart_printf("\t%x\n", 0); */
    }
  }
}

//Draw detections (bounding boxes and class labels) in input image
void draw_detections() {

  //local variables
  int i, j, k;
  /* uint8_t colors[6][3] = { {255,0,255}, {0,0,255},{0,255,255},{0,255,0},{255,255,0},{255,0,0} }; //Q8.0 */
  /* uint8_t ratio, red, green, blue, label_w; */
  int16_t label_w, red, green, blue;
  uint16_t mul_16;
  int32_t mul_32;
  int offset, ratio_min, ratio_max;
  int left, right, top, bot, top_width, previous_w;


  /* //write random labels */
  /* top_width = 0; */
  /* left = 0; */
  /* previous_w = 0; */

  /* for(i=0; i<8; i++) { */
  /*   previous_w = 0; */

  /*   for(j=i*10; j<10+(i*10); j++) { */
  /*     // pick generated colors based on class value */
  /*     red = fp_rgb[RGB_LINE*j + 0]; */
  /*     green = fp_rgb[RGB_LINE*j + 1]; */
  /*     blue = fp_rgb[RGB_LINE*j + 2]; */
      
  /*     if(previous_w == 0) { */
  /*     /\* top_width = 30; *\/ */
  /*     } else { */
  /* 	label_w = fp_labels[80]; */
  /* 	/\* draw_class(label_w, 80, top_width, left, previous_w, red, green, blue); *\/ */
  /* 	draw_class_versat(label_w, 80, top_width+25, left, previous_w, j); */
  /* 	/\* verify_class(label_w, 80, top_width, left, previous_w, red, green, blue); *\/ */
  /* 	previous_w += label_w; */
  /*     } */
      
  /*     //Draw class labels */
  /*     label_w = fp_labels[j]; */
  /*     /\* draw_class(label_w, j, top_width, left, previous_w, red, green, blue); *\/ */
  /*     draw_class_versat(label_w, j, top_width+25, left, previous_w, j); */
  /*     /\* verify_class(label_w, j, top_width, left, previous_w, red, green, blue); *\/ */
  /*     previous_w += label_w; */
      
  /*   } */
  /*   top_width += 50; */
  /* } */
  //Check valid detections
  for(i = 0; i < nboxes; i++) {

    //Find detected classes
    previous_w = 0;
    for(j = 0; j < 80; j++) {
      if(boxes[84*i+4+j] != 0) {

        //Check if this was the first class detected for given box
        if(previous_w == 0) {

          //Randomly pick rgb colors for the box
          /* offset = j*123457 % 80; */
          /* mul_16 = (uint16_t)((uint16_t)offset*(uint16_t)((uint8_t)0x10)); //Q8.0 *Q0.8 = Q8.8 */
          /* ratio = (uint8_t)(mul_16>>2); //Q8.8 to Q2.6 */
          /* ratio_min = (ratio >> 6); */
          /* ratio_max = ratio_min + 1; */
          /* ratio = ratio & 0x3F; //Q2.6 */
          /* mul_16 = (uint16_t)((uint16_t)(0x40-ratio)*(uint16_t)colors[ratio_min][2]); //Q2.6 *Q8.0 = Q10.6 */
          /* mul_16 += (uint16_t)((uint16_t)ratio*(uint16_t)colors[ratio_max][2]); //Q2.6 *Q8.0 = Q10.6 */
          /* red = (mul_16 >> 6); //Q10.6 to Q8.0 */
          /* mul_16 = (uint16_t)((uint16_t)(0x40-ratio)*(uint16_t)colors[ratio_min][1]); //Q2.6 *Q8.0 = Q10.6 */
          /* mul_16 += (uint16_t)((uint16_t)ratio*(uint16_t)colors[ratio_max][1]); //Q2.6 *Q8.0 = Q10.6 */
          /* green = (mul_16 >> 6); //Q10.6 to Q8.0 */
          /* mul_16 = (uint16_t)((uint16_t)(0x40-ratio)*(uint16_t)colors[ratio_min][0]); //Q2.6 *Q8.0 = Q10.6 */
          /* mul_16 += (uint16_t)((uint16_t)ratio*(uint16_t)colors[ratio_max][0]); //Q2.6 *Q8.0 = Q10.6 */
          /* blue = (mul_16 >> 6); //Q10.6 to Q8.0 */

  	  // pick generated colors based on class value
  	  /* red = fp_rgb[RGB_LINE*j + 0]; */
  	  /* green = fp_rgb[RGB_LINE*j + 1]; */
  	  /* blue = fp_rgb[RGB_LINE*j + 2]; */

  	  /* uart_printf("Class %d | R: %d | G: %d | B: %d\n", j, red, green, blue); */

          //Calculate box coordinates in image frame
          mul_16 = boxes[84*i] - (boxes[84*i+2]>>1); //Q3.13
          mul_32 = (int32_t)((int32_t)mul_16 * (int32_t)IMG_W); //Q3.13 * Q16.0 = Q19.13
          left = (mul_32 >> 13);
          mul_16 = boxes[84*i] + (boxes[84*i+2]>>1); //Q3.13
          mul_32 = (int32_t)((int32_t)mul_16 * (int32_t)IMG_W); //Q3.13 * Q16.0 = Q19.13
          right = (mul_32 >> 13);
          mul_16 = boxes[84*i+1] - (boxes[84*i+3]>>1); //Q3.13
          mul_32 = (int32_t)((int32_t)mul_16 * (int32_t)IMG_H); //Q3.13 * Q16.0 = Q19.13
          top = (mul_32 >> 13);
          mul_16 = boxes[84*i+1] + (boxes[84*i+3]>>1); //Q3.13
          mul_32 = (int32_t)((int32_t)mul_16 * (int32_t)IMG_H); //Q3.13 * Q16.0 = Q19.13
          bot = (mul_32 >> 13);

          //Draw box
          /* for(k = 0; k < box_width; k++) draw_box(left+k, top+k, right-k, bot-k, red, green, blue); */
	  draw_box_versat(left, top, right, bot, j, box_width);

          //Limit top and left box coordinates
          if(top < 0) top = 0;
          if(left < 0) left = 0;
          top_width = top + box_width;
          if(top_width - label_height >= 0) top_width -= label_height;

  	  //DEBUG: print first label values
  	  /* int x, y; */
  	  /* uint16_t label; */
  	  /* uart_printf("Some label values\n"); */
  	  /* for(x = 0; x < 6; x++) { */
  	  /*   for(y = 0; y < 12; y++) { */
  	  /*     label = fp_labels[LABEL_W_OFF+LABEL_SIZE*j+x*LABEL_LINE_SIZE+y]; */
  	  /*     uart_printf("\tlabel[%d][%d][%d]: %d\n", j, x, y, label); */
  	  /*   } */
  	  /* } */

        //Otherwise, add comma and space
        } else {
          label_w = fp_labels[80];
          /* draw_class(label_w, 80, top_width, left, previous_w, red, green, blue); */
          draw_class_versat(label_w, 80, top_width, left, previous_w, j);
          /* verify_class(label_w, 80, top_width, left, previous_w, red, green, blue); */
          previous_w += label_w;
        }

        //Draw class labels
        label_w = fp_labels[j];
        /* draw_class(label_w, j, top_width, left, previous_w, red, green, blue); */
        draw_class_versat(label_w, j, top_width, left, previous_w, j);
        /* verify_class(label_w, j, top_width, left, previous_w, red, green, blue); */
        previous_w += label_w;
      }
    }
  }
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
  uart_printf("\nPOS CNN\n\n");

  //init versat
  versat_init(VERSAT_BASE);

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

  uart_printf("NUMBER OF BOXES: %d\n", nboxes);

  //filter boxes
  uart_printf("\nFiltering boxes...\n");
  start = timer_time_us(TIMER_BASE);
  filter_boxes();
  end = timer_time_us(TIMER_BASE);
  uart_printf("Done in %d us\n\n", end-start);

  /* //DEBUG: copy solution to boxes */
  /* copy_boxes(); */

/* #ifndef SIM */
  //draw boxes and labels
  uart_printf("\nDrawing boxes and labels...\n");
  start = timer_time_us(TIMER_BASE);
  draw_detections();
  end = timer_time_us(TIMER_BASE);
  uart_printf("Done in %d us\n\n", end-start);
/* #else */
#ifdef SIM
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
  send_data();
#endif

  //finish
  uart_putc(ETX);
  uart_txwait();
  return 0;
}
