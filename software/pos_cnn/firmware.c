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
#define IMAGE_INPUT (IMG_W*IMG_H*IMG_C) //already 32-byte aligned

//yolo constants
#define LAYER_16_W 13
#define LAYER_23_W 26

//constants for bounding boxes
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

//define ethernet constants
#define ETH_NBYTES (1024-18) //minimum ethernet payload excluding FCS
#define INPUT_FILE_SIZE (LABELS_FILE_SIZE + (IMAGE_INPUT + 13*13*256 + 26*26*256)*2) //16 bits
#define NUM_INPUT_FRAMES (INPUT_FILE_SIZE/ETH_NBYTES)
#define OUTPUT_FILE_SIZE (IMAGE_INPUT) //8 bits
#define NUM_OUTPUT_FRAMES (OUTPUT_FILE_SIZE/ETH_NBYTES)

//define DDR mapping
#define LABEL_BASE_ADDRESS (DDR_MEM + (1 << (FIRM_ADDR_W))) //after main mem
#define INPUT_IMAGE_BASE_ADDRESS (LABEL_BASE_ADDRESS + LABELS_FILE_SIZE)
#define DATA_BASE_ADDRESS (INPUT_IMAGE_BASE_ADDRESS + 2*IMAGE_INPUT)

//ETHERNET variables
int rcv_timeout = 5000;
char data_to_send[ETH_NBYTES];
char data_rcv[ETH_NBYTES+18];
unsigned int bytes_to_receive, bytes_to_send, count_bytes;

//TIMER variables
unsigned int start, end;

//input image pointer
uint8_t * fp_labels = (uint8_t *) LABEL_BASE_ADDRESS;
int16_t * fp_image = (int16_t *) INPUT_IMAGE_BASE_ADDRESS;

//receive weigths and resized padded image
void rcv_data() {

  //Local variables
  int i, j;
  count_bytes = 0;
  char * data_p = (char *) LABEL_BASE_ADDRESS;
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
    fp_image[top*IMG_W + i] = red;
    fp_image[bot*IMG_W + i] = red;
    fp_image[IMG_W*IMG_H + top*IMG_W + i] = green;
    fp_image[IMG_W*IMG_H + bot*IMG_W + i] = green;
    fp_image[IMG_W*IMG_H*2 + top*IMG_W + i] = blue;
    fp_image[IMG_W*IMG_H*2 + bot*IMG_W + i] = blue;
  }

  //Draw vertically
  for(i = top; i <= bot; i++) {
    fp_image[i*IMG_W + left] = red;
    fp_image[i*IMG_W + right] = red;
    fp_image[IMG_W*IMG_H + i*IMG_W + left] = green;
    fp_image[IMG_W*IMG_H + i*IMG_W + right] = green;
    fp_image[IMG_W*IMG_H*2 + i*IMG_W + left] = blue;
    fp_image[IMG_W*IMG_H*2 + i*IMG_W + right] = blue;
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
      fp_image[(l+top_width)*IMG_W+(k+left+previous_w)] = ((uint16_t)((uint16_t)r*(uint16_t)label)) >> 8;
      //green
      fp_image[IMG_W*IMG_H+(l+top_width)*IMG_W+(k+left+previous_w)] = ((uint16_t)((uint16_t)g*(uint16_t)label)) >> 8;
      //blue
      fp_image[2*IMG_W*IMG_H+(l+top_width)*IMG_W+(k+left+previous_w)] = ((uint16_t)((uint16_t)b*(uint16_t)label)) >> 8;
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

  //filter boxes
  uart_printf("\nFiltering boxes...\n");
  start = timer_time_us(TIMER_BASE);
  filter_boxes();
  end = timer_time_us(TIMER_BASE);
  uart_printf("Done in %d us\n\n", end-start);

#ifndef SIM
  //draw boxes and labels
  uart_printf("\nDrawing boxes and labels...\n");
  start = timer_time_us(TIMER_BASE);
  draw_detections();
  end = timer_time_us(TIMER_BASE);
  uart_printf("Done in %d us\n\n", end-start);
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
  send_data();
#endif

  //finish
  uart_putc(ETX);
  uart_txwait();
  return 0;
}
