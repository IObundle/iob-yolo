//import custom libraries
#include "system.h"
#include "iob-uart.h"
#include "iob-eth.h"
#include "iob_timer.h"
#include "firmware.h"
#include "iob-cache.h"

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
#define CACHE_CTRL (CACHE_CTRL_BASE<<(ADDR_W-N_SLAVES_W))

void print_cache_status() {
  uart_printf("ctrl_data_read_hit = %d\n", ctrl_data_read_hit(CACHE_CTRL));
  uart_printf("ctrl_data_read_miss = %d\n", ctrl_data_read_miss(CACHE_CTRL));
  uart_printf("ctrl_data_write_hit = %d\n", ctrl_data_write_hit(CACHE_CTRL));
  uart_printf("ctrl_data_write_miss = %d\n\n", ctrl_data_write_miss(CACHE_CTRL));
  ctrl_counter_reset(CACHE_CTRL);
}

#ifdef FIXED

 //Constants for image resize
 #define w_scale ((uint32_t)(((float)(IMG_W-1)/(NEW_W-1))*((uint32_t)1<<22))) // Q1.22
 #define h_scale ((uint32_t)(((float)(IMG_H-1)/(NEW_H-1))*((uint32_t)1<<20))) // Q1.20

 //Constants for bounding boxes
 #define threshold ((int16_t)(((float)0.5)*((int32_t)1<<8))) //Q8.8
 #define nms_threshold ((int16_t)(((float)0.45)*((int32_t)1<<14))) //Q2.14
 #define yolo1_div ((int16_t)(((float)1/LAYER_17_W)*((int32_t)1<<15))) //Q1.15
 #define yolo2_div ((int16_t)(((float)1/LAYER_24_W)*((int32_t)1<<15))) //Q1.15
 #define y_scales ((int16_t)(((float)NEW_W/NEW_H)*((int32_t)1<<14))) //Q2.14
 #define y_bias ((int16_t)(((float)(NEW_W-NEW_H)/(NEW_H*2))*((int32_t)1<<14))) //Q2.14
 #define w_scales ((int16_t)(((float)1/NEW_W)*((int32_t)1<<14))) //Q2.14
 #define h_scales ((int16_t)(((float)1/NEW_H)*((int32_t)1<<14))) //Q2.14
 #define c3 ((int16_t)0x0AAA) // pow(2,-3)+pow(2,-5)+pow(2,-7)+pow(2,-9)+pow(2,-11)+pow(2,-13) in Q2.14
 #define c4 ((int16_t)0x02C0) // pow(2,-5)+pow(2,-7)+pow(2,-8) in Q2.14

 //define general constants
 #define ETH_NBYTES (1024-18) //minimum ethernet payload excluding FCS
 #define INPUT_FILE_SIZE IMAGE_INPUT //8 bits per point
 #define NUM_INPUT_FRAMES (INPUT_FILE_SIZE/ETH_NBYTES)
 #define WEIGHTS_FILE_SIZE (17704732) //16 bits per input
 #define NUM_WEIGHT_FRAMES (WEIGHTS_FILE_SIZE/ETH_NBYTES)
 #define LABELS_FILE_SIZE (82+MAX_LABEL_SIZE*81) //8 bits per input (82 to keep it align)
 #define GEMM_IN_SIZE (NTW_IN_W*NTW_IN_H*NTW_IN_NUM_KER*NTW_IN_KER_SIZE*NTW_IN_KER_SIZE*2) //16 bits per input
 #define GEMM_OUT_SIZE (NTW_IN_W*NTW_IN_H*NTW_IN_NUM_KER*4) //32 bits per input

 //define DDR mapping
 #define WEIGTHS_BASE_ADDRESS (DDR_MEM + 0x00008000) //16kb for program + 16kb for stack
 #define LABEL_BASE_ADDRESS (WEIGTHS_BASE_ADDRESS + WEIGHTS_FILE_SIZE)
 #define GEMM_IN_BASE_ADDRESS (LABEL_BASE_ADDRESS + LABELS_FILE_SIZE)
 #define GEMM_OUT_BASE_ADDRESS (GEMM_IN_BASE_ADDRESS + GEMM_IN_SIZE + 2) //+2 to be aligned
 #define DATA_BASE_ADDRESS (GEMM_OUT_BASE_ADDRESS + GEMM_OUT_SIZE)

 //define intermediate data constants
 #define INTERM_LAYER1_SIZE (NTW_IN_W*2) //16 bits per input
 #define NUM_INTERM_LAYER1_FRAMES (INTERM_LAYER1_SIZE/ETH_NBYTES)
 #define INTERM_LAYER2_SIZE ((LAYER_3_W+2)*2*2) //16 bits per input
 #define NUM_INTERM_LAYER2_FRAMES (INTERM_LAYER2_SIZE/ETH_NBYTES)
 #define INTERM_LAYER4_SIZE ((LAYER_5_W+2)*2*2) //16 bits per input
 #define NUM_INTERM_LAYER4_FRAMES (INTERM_LAYER4_SIZE/ETH_NBYTES)
 #define INTERM_LAYER5_SIZE (LAYER_5_W*2) //16 bits per input
 #define NUM_INTERM_LAYER5_FRAMES (INTERM_LAYER5_SIZE/ETH_NBYTES)
 #define INTERM_LAYER6_SIZE ((LAYER_7_W+2)*2*2) //16 bits per input
 #define NUM_INTERM_LAYER6_FRAMES (INTERM_LAYER6_SIZE/ETH_NBYTES)
 #define INTERM_LAYER8_SIZE ((LAYER_9_W+2)*2*2) //16 bits per input
 #define NUM_INTERM_LAYER8_FRAMES (INTERM_LAYER8_SIZE/ETH_NBYTES)
 #define INTERM_LAYER9_SIZE ((LAYER_9_W+2)*2) //16 bits per input
 #define NUM_INTERM_LAYER9_FRAMES (INTERM_LAYER9_SIZE/ETH_NBYTES)

 //Constants for bounding boxes
 #define box_width 3
 #define label_height 20
 uint8_t nboxes = 0;

 //weights and data updatable pointers
 unsigned int weight_pos = 0, data_pos = 0;

 //ETHERNET variables
 int rcv_timeout = 5000;
 char data_to_send[ETH_NBYTES];
 char data_rcv[ETH_NBYTES+18];
 unsigned int bytes_to_receive, bytes_to_send, count_bytes;

 //TIMER variables
 unsigned int start, end;

 //weights and data base address pointers
 int16_t *fp_weights;
 int16_t *fp_data;
 uint8_t * fp_image;
 uint8_t * fp_labels;
#ifdef GEMM
 int16_t * fp_gemm_in;
 int32_t * fp_gemm_out;
#endif


 //define base address of weights and data pointers
 void define_memory_regions() {

  //image
  fp_image = (uint8_t *) DATA_BASE_ADDRESS;

  //data
  fp_data = (int16_t *) (DATA_BASE_ADDRESS + IMAGE_INPUT);

  //weights
  fp_weights = (int16_t *) WEIGTHS_BASE_ADDRESS;

  //labels
  fp_labels = (uint8_t *) LABEL_BASE_ADDRESS;

 #ifdef GEMM
  fp_gemm_in = (int16_t *) GEMM_IN_BASE_ADDRESS;
  fp_gemm_out = (int32_t *) GEMM_OUT_BASE_ADDRESS;
 #endif
 }

 //fill part of 416x316 region of resized image with grey (0.5 = 0x0080 in Q8.8)
 void fill_grey() {
  int i, j, k;
  for(i = 0; i < NTW_IN_C; i++) {
    for(j = 0; j < EXTRA_H; j++) {
      for(k = 0; k < NTW_IN_W; k++) {
	fp_data[i*(NTW_IN_W+2)*(NTW_IN_H+2) + (j+GREY_PADD)*(NTW_IN_W+2) + (k+1)] = 0x0080; //1st region
	fp_data[i*(NTW_IN_W+2)*(NTW_IN_H+2) + (j+GREY_PADD)*(NTW_IN_W+2) + (k+1) + ((NTW_IN_W+2)*(EXTRA_H+NEW_H))] = 0x0080; //2nd region
      }   
    }
  }
 }

 //resize input image to 416x312
 void resize_image() {

  //local variables
  uint16_t r, c, k;
  uint16_t iy, ix, new_r;
  uint32_t sx, sy, mul, dy, dx;
  uint16_t val_h, val_w, next_val_w;
			
  for(k = 0; k < NTW_IN_C; k++) { 			// 3
    for(c = 0; c < NTW_IN_W; c++) {			// 416
      new_r = 1;
			
      //Width index calculation
      if(c == NTW_IN_W-1) val_w = (uint16_t)(fp_image[k*IMG_W*IMG_H + (IMG_W-1)]); //Q0.8 to Q0.12
      else {
      	sx = (uint32_t)((uint32_t)c*w_scale); //9.0 * Q1.22 = Q10.22			
	ix = (sx >> 22);
	dx = (sx & 0x3FFFFF); //Q0.22
	mul = (uint32_t)((uint32_t)((1<<22)- dx)*(uint32_t)(fp_image[k*IMG_W*IMG_H + ix])); //Q0.22 * Q0.8 = Q0.30
	mul += (uint32_t)((uint32_t)(dx)*(uint32_t)(fp_image[k*IMG_W*IMG_H + (ix+1)])); //Q0.30
	val_w = (uint16_t) (mul >> 18); //Q0.30 to Q0.12
      }
			
      for(r = 0; r < NEW_H; r++) {			// 312

        //Width reduction (to 416)	
	if(c == NTW_IN_W-1) next_val_w = (uint16_t)(fp_image[k*IMG_W*IMG_H + new_r*IMG_W + (IMG_W-1)]); //Q0.8 to Q0.12
	else {
	  mul = (uint32_t)((uint32_t)((1<<22)- dx)*(uint32_t)(fp_image[k*IMG_W*IMG_H + new_r*IMG_W + ix])); //Q0.22 * Q0.8 = Q0.30
	  mul += (uint32_t)((uint32_t)(dx)*(uint32_t)(fp_image[k*IMG_W*IMG_H + new_r*IMG_W + (ix+1)])); //Q0.30
	  next_val_w = (uint16_t) (mul >> 18); //Q0.30 to Q0.12
	}
				
	//Height index calculation
	if(r != NEW_H-1) {
	  sy = (uint32_t)((uint32_t)r*h_scale); //Q9.0 * Q1.20 = Q10.20
	  iy = (sy >> 20);
	  dy = (sy & 0xFFFFF); //Q0.20
					
	  //Check if to calculate next width
	  if( (iy+1) != new_r) {
	    new_r++;
	    val_w = next_val_w;					
	    if(c == NTW_IN_W-1) next_val_w = (uint16_t)(fp_image[k*IMG_W*IMG_H + new_r*IMG_W + (IMG_W-1)]); //Q0.8 to Q0.12
	    else {
	      mul = (uint32_t)((uint32_t)((1<<22)- dx)*(uint32_t)(fp_image[k*IMG_W*IMG_H + new_r*IMG_W + ix])); //Q0.22 * Q0.8 = Q0.30
	      mul += (uint32_t)((uint32_t)(dx)*(uint32_t)(fp_image[k*IMG_W*IMG_H + new_r*IMG_W + (ix+1)])); //Q0.30
	      next_val_w = (uint16_t) (mul >> 18); //Q0.30 to Q0.12
 	    }
          }
					
	  //Height reduction (to 312)
	  mul = (uint32_t)((uint32_t)((1<<20)- dy)*(uint32_t)val_w); //Q0.20 * Q0.12 = Q0.32
	  mul += (uint32_t)((uint32_t)(dy)*(uint32_t)next_val_w); //Q0.32
	  val_h = (uint16_t) (mul >> 24); //Q0.32 to Q8.8
	} else val_h = (next_val_w >> 4); //Q0.12 to Q8.8
				
	//Save new value
	fp_data[k*(NTW_IN_W+2)*(NTW_IN_H+2) + (r+GREY_PADD)*(NTW_IN_W+2) + (c+1) + ((NTW_IN_W+2)*EXTRA_H)] = val_h;
				
	//Update variables
	new_r++;
	val_w = next_val_w;
      }     
    }
  }
 }

 //perform convolutional layer
 void conv_layer(int w, int h, int c, int num_ker, int ker_size, int pad, int batch_norm, int nextPadding, int nextStride, int ignorePadding, unsigned int new_output_pos, unsigned int offset) {

  //locate weight and data pointers
  unsigned int new_h, new_h_output, out_offset, new_w;
  if(nextStride) new_w = w+1; else if(nextPadding) new_w = w+2; else new_w = w;
  if(w == h) { 
    new_h = w+2*pad;
    out_offset = 0;
    new_h_output = new_w;
  } else { 
    new_h = h+2;
    out_offset = offset;
    if(offset == 0) new_h_output = h; else new_h_output = new_h;
  }
  unsigned int pos_delta = (w+2*pad)*new_h*c;
  int16_t * w_pos;
  w_pos = (int16_t *) fp_weights + weight_pos;
  int16_t * bias_pos = (int16_t *) fp_weights + weight_pos + num_ker*ker_size*ker_size*c;
  int16_t * in_d_pos = (int16_t *) fp_data + data_pos;
  int16_t * out_d_pos;
  if(new_output_pos != 0) out_d_pos = (int16_t *) fp_data + new_output_pos; else out_d_pos = (int16_t *) in_d_pos + pos_delta;

  //local variables
  int i, j, k, l, m, n;
  unsigned int output_pos;
  int16_t output_conv, leaky = 3276; //0.1 in Q1.15;
  int32_t mul;

 #ifdef GEMM

  //Transformation
  for(l = 0; l < c; l++)  		//Number of channels
    for(m = 0; m < ker_size; m++) 	//Kernel size
      for(n = 0; n < ker_size; n++) 
	for(j = 0; j < h; j++)   	//Output map size
	  for(k = 0; k < w; k++) 
	    fp_gemm_in[l*ker_size*ker_size*h*w + m*ker_size*h*w + n*h*w + j*w + k] = in_d_pos[(j+ignorePadding)*(w+2*pad) + (k+ignorePadding) + l*(w+2*pad)*new_h + m*(w+2*pad) + n];

  //Convolution
  for(i = 0; i < num_ker; i++) {
    for(j = 0; j < c*ker_size*ker_size; j++) {
      for(k = 0; k < w*h; k++) { 
	//Q4.12 * Q8.8 = Q12.20
	if(j == 0) fp_gemm_out[i*w*h + k] = (int32_t)((int32_t)w_pos[i*c*ker_size*ker_size + j]*(int32_t)fp_gemm_in[j*w*h + k]);
	else fp_gemm_out[i*w*h + k] += (int32_t)((int32_t)w_pos[i*c*ker_size*ker_size + j]*(int32_t)fp_gemm_in[j*w*h + k]);
      }
    }
  }  

  //Result
  for(i = 0; i < num_ker; i++) {
    for(j = 0; j < h; j++) { 
      for(k = 0; k < w; k++) {

      //Calculate output position
      if(nextPadding) output_pos = i*new_w*new_w + (j+1)*new_w + (k+1) + (out_offset*new_w);
      else output_pos = i*new_w*new_h_output + j*new_w + k + (out_offset*new_w);

      //perform batch normalize
      if(batch_norm) {
	output_conv = (int16_t)((int32_t) (fp_gemm_out[i*w*h + j*w + k] << 3) >> 16); //Q12.20 to Q9.7
	mul = (int32_t) output_conv * (int32_t) bias_pos[i]; //Q9.7 * Q8.8 = Q17.15
	output_conv = (int16_t)((int32_t) (mul << 9) >> 16) +  bias_pos[num_ker+i]; //Q17.15 to Q8.8
	if(output_conv < 0) {
	  mul = (int32_t) output_conv * (int32_t) leaky; //Q8.8 * Q1.15 = Q9.23
	  output_conv = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
	}
	out_d_pos[output_pos] = output_conv;

	//Copy last column and last row if needed
	if(nextStride) {
	  if(k == w-1) out_d_pos[output_pos + 1] = output_conv; 
	  if(j == w-1) out_d_pos[output_pos + new_w] = output_conv;
	  if(k == w-1 && j == w-1) out_d_pos[output_pos + 1 + new_w] = output_conv;
	}
      } 
 
      //otherwise, only add bias
      else {
	output_conv = (int16_t)((int32_t)(fp_gemm_out[i*w*h + j*w + k] << 4) >> 16);//Q12.20 to Q8.8
	out_d_pos[output_pos] = output_conv + bias_pos[i];
      }
    }
  }
 }

 #else

  //local variables
  unsigned int output_pos2, output_pos3, output_pos4;
  int16_t op1, op2, op2_2, op2_3, op2_4;
  int16_t output_conv2, output_conv3, output_conv4;
  int16_t mul_16, mul_16_2, mul_16_3, mul_16_4; //0.1 in Q1.15;
  int32_t acc_final, acc_final2, acc_final3, acc_final4; 
  int32_t mul2, mul3, mul4;

  //perform convolution
  for(i = 0; i < num_ker; i+=4) {               //Number of kernels
    for(j = 0; j < h; j++) {   	        	//Output map size
      for(k = 0; k < w; k++) {
        if(nextPadding) {
	  output_pos = i*new_w*new_w + (j+1)*new_w + (k+1) + (out_offset*new_w);
	  output_pos2 = (i+1)*new_w*new_w + (j+1)*new_w + (k+1) + (out_offset*new_w);
	  output_pos3 = (i+2)*new_w*new_w + (j+1)*new_w + (k+1) + (out_offset*new_w);
	  output_pos4 = (i+3)*new_w*new_w + (j+1)*new_w + (k+1) + (out_offset*new_w);
	} else { 
	  output_pos = i*new_w*new_h_output + j*new_w + k + (out_offset*new_w);
	  output_pos2 = (i+1)*new_w*new_h_output + j*new_w + k + (out_offset*new_w);
	  output_pos3 = (i+2)*new_w*new_h_output + j*new_w + k + (out_offset*new_w);
	  output_pos4 = (i+3)*new_w*new_h_output + j*new_w + k + (out_offset*new_w);
	}
	acc_final = 0;
	acc_final2 = 0;
	acc_final3 = 0;
	acc_final4 = 0;
	for(l = 0; l < c; l++) { 		//Number of channels
	  for(m = 0; m < ker_size; m++) {	//Kernel size
	    for(n = 0; n < ker_size; n++) {
	      op1 = in_d_pos[(j+ignorePadding)*(w+2*pad) + (k+ignorePadding) + l*(w+2*pad)*new_h + m*(w+2*pad) + n]; //Q8.8
	      op2 = w_pos[i*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul = (int32_t)((int32_t)op1*(int32_t)op2); //Q12.20
	      acc_final += mul; //Q12.20
	      op2_2 = w_pos[(i+1)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul2 = (int32_t)((int32_t)op1*(int32_t)op2_2); //Q12.20
	      acc_final2 += mul2; //Q12.20
	      op2_3 = w_pos[(i+2)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul3 = (int32_t)((int32_t)op1*(int32_t)op2_3); //Q12.20
	      acc_final3 += mul3; //Q12.20
	      op2_4 = w_pos[(i+3)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul4 = (int32_t)((int32_t)op1*(int32_t)op2_4); //Q12.20
	      acc_final4 += mul4; //Q12.20
	    }
	  }
	}

	//perform batch normalize
	if(batch_norm) {
	  output_conv = (int16_t) ((int32_t) (acc_final << 3) >> 16);//Q12.20 to Q9.7
  	  mul = (int32_t) output_conv * (int32_t) bias_pos[i];//Q9.7 * Q8.8 = Q17.15
	  mul_16 = (int16_t) ((int32_t) (mul << 9) >> 16);//Q17.15 to Q8.8
	  mul_16 += bias_pos[num_ker+i]; //Q8.8
	  output_conv2 = (int16_t) ((int32_t) (acc_final2 << 3) >> 16);//Q12.20 to Q9.7
  	  mul2 = (int32_t) output_conv2 * (int32_t) bias_pos[i+1];//Q9.7 * Q8.8 = Q17.15
	  mul_16_2 = (int16_t) ((int32_t) (mul2 << 9) >> 16);//Q17.15 to Q8.8
	  mul_16_2 += bias_pos[num_ker+i+1]; //Q8.8
	  output_conv3 = (int16_t) ((int32_t) (acc_final3 << 3) >> 16);//Q12.20 to Q9.7
  	  mul3 = (int32_t) output_conv3 * (int32_t) bias_pos[i+2];//Q9.7 * Q8.8 = Q17.15
	  mul_16_3 = (int16_t) ((int32_t) (mul3 << 9) >> 16);//Q17.15 to Q8.8
	  mul_16_3 += bias_pos[num_ker+i+2]; //Q8.8
	  output_conv4 = (int16_t) ((int32_t) (acc_final4 << 3) >> 16);//Q12.20 to Q9.7
  	  mul4 = (int32_t) output_conv4 * (int32_t) bias_pos[i+3];//Q9.7 * Q8.8 = Q17.15
	  mul_16_4 = (int16_t) ((int32_t) (mul4 << 9) >> 16);//Q17.15 to Q8.8
	  mul_16_4 += bias_pos[num_ker+i+3]; //Q8.8

	  //perform leaky activation
	  if(mul_16 < 0) {
	    mul = (int32_t) mul_16 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	    mul_16 = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
	  }
	  out_d_pos[output_pos] = mul_16;
	  if(mul_16_2 < 0) {
	    mul2 = (int32_t) mul_16_2 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	    mul_16_2 = (int16_t) ((int32_t)(mul2 << 1 ) >> 16); //Convert to Q8.8
	  }
	  out_d_pos[output_pos2] = mul_16_2;
	  if(mul_16_3 < 0) {
	    mul3 = (int32_t) mul_16_3 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	    mul_16_3 = (int16_t) ((int32_t)(mul3 << 1 ) >> 16); //Convert to Q8.8
	  }
	  out_d_pos[output_pos3] = mul_16_3;
	  if(mul_16_4 < 0) {
	    mul4 = (int32_t) mul_16_4 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	    mul_16_4 = (int16_t) ((int32_t)(mul4 << 1 ) >> 16); //Convert to Q8.8
	  }
	  out_d_pos[output_pos4] = mul_16_4;

	  //Copy last column and last row if needed
	  if(nextStride) {
	    if(k == w-1) { 
	      out_d_pos[output_pos + 1] = mul_16;
	      out_d_pos[output_pos2 + 1] = mul_16_2;
	      out_d_pos[output_pos3 + 1] = mul_16_3;
	      out_d_pos[output_pos4 + 1] = mul_16_4;
	    }
	    if(j == w-1) {
	      out_d_pos[output_pos + new_w] = mul_16;
	      out_d_pos[output_pos2 + new_w] = mul_16_2;
	      out_d_pos[output_pos3 + new_w] = mul_16_3;
	      out_d_pos[output_pos4 + new_w] = mul_16_4;
	    }
	    if(k == w-1 && j == w-1) {
	      out_d_pos[output_pos + 1 + new_w] = mul_16;
	      out_d_pos[output_pos2 + 1 + new_w] = mul_16_2;
	      out_d_pos[output_pos3 + 1 + new_w] = mul_16_3;
	      out_d_pos[output_pos4 + 1 + new_w] = mul_16_4;
	    }
	  }
	}

	//otherwise, only add bias
	else {
	  output_conv = (int16_t) ((int32_t) (acc_final << 4) >> 16);//Q12.20 to Q8.8
	  out_d_pos[output_pos] = output_conv + bias_pos[i];
	  output_conv2 = (int16_t) ((int32_t) (acc_final2 << 4) >> 16);//Q12.20 to Q8.8
	  out_d_pos[output_pos2] = output_conv2 + bias_pos[i+1];
	  output_conv3 = (int16_t) ((int32_t) (acc_final3 << 4) >> 16);//Q12.20 to Q8.8
	  out_d_pos[output_pos3] = output_conv3 + bias_pos[i+2];
	  output_conv4 = (int16_t) ((int32_t) (acc_final4 << 4) >> 16);//Q12.20 to Q8.8
	  out_d_pos[output_pos4] = output_conv4 + bias_pos[i+3];
	}
      }
    }
  }
 #endif

  //update weights and data pointers
  if(batch_norm) weight_pos += num_ker*2 + num_ker*c*ker_size*ker_size;
  else weight_pos += num_ker + num_ker*c*ker_size*ker_size;
  if(new_output_pos != 0) data_pos = new_output_pos; else data_pos += pos_delta;
 }

 //perform maxpool layer
 void maxpool_layer(int w, int h, int num_ker, int downsample, int ignorePadding, unsigned int new_output_pos) {

  //locate data pointers
  unsigned int new_h, out_offset, output_w = (w/(1+downsample))+2, new_h_output;
  if(w == h) { 
    new_h = w+1-downsample+2*ignorePadding;
    out_offset = 0;
    new_h_output = output_w;
  } else { 
    new_h = h;
    out_offset = 1;
    new_h_output = h/2+4;;
  }
  int16_t * in_d_pos = (int16_t *) fp_data + data_pos;
  int16_t * out_d_pos;
  if(new_output_pos != 0) out_d_pos = (int16_t *) fp_data + new_output_pos;
  else out_d_pos = (int16_t *) in_d_pos + (w+1-downsample+2*ignorePadding)*new_h*num_ker;

  //local variables
  int i, j, k, l, m, new_w = w+1-downsample+2*ignorePadding;
  int16_t max, max2, max3, max4, val, val2, val3, val4;

  //perform max pooling
  for(i = 0; i < num_ker; i+=4) { 		//Number of kernels
    for(j = 0; j < h/(1+downsample); j++) {	//Output map size
      for(k = 0; k < w/(1+downsample); k++) {
	for(l = 0; l < 2; l++) {		//2x2 block
 	  for(m = 0; m < 2; m++) {
	    val = in_d_pos[i*new_w*new_h + j*new_w*(1+downsample) + k*(1+downsample) + l*new_w + m + (1 + new_w)*ignorePadding];
	    val2 = in_d_pos[(i+1)*new_w*new_h + j*new_w*(1+downsample) + k*(1+downsample) + l*new_w + m + (1 + new_w)*ignorePadding];
	    val3 = in_d_pos[(i+2)*new_w*new_h + j*new_w*(1+downsample) + k*(1+downsample) + l*new_w + m + (1 + new_w)*ignorePadding];
	    val4 = in_d_pos[(i+3)*new_w*new_h + j*new_w*(1+downsample) + k*(1+downsample) + l*new_w + m + (1 + new_w)*ignorePadding];
	    if(l == 0 && m == 0) { max = val; max2 = val2; max3 = val3; max4 = val4; }
	    else { if(max < val) max = val; if(max2 < val2) max2 = val2; if(max3 < val3) max3 = val3; if(max4 < val4) max4 = val4; }
	  }
	}
	out_d_pos[i*output_w*new_h_output + (j+1)*output_w + (k+1) + out_offset*output_w] = max;
	out_d_pos[(i+1)*output_w*new_h_output + (j+1)*output_w + (k+1) + out_offset*output_w] = max2;
	out_d_pos[(i+2)*output_w*new_h_output + (j+1)*output_w + (k+1) + out_offset*output_w] = max3;
	out_d_pos[(i+3)*output_w*new_h_output + (j+1)*output_w + (k+1) + out_offset*output_w] = max4;
      }
    }
  }

  //update data pointer
  if(new_output_pos != 0) data_pos = new_output_pos;
  else data_pos += (w+1-downsample+2*ignorePadding)*new_h*num_ker;
 }

 //perform yolo layer
 void yolo_layer(int w) {

  //locate data pointers
  int16_t * in_d_pos = (int16_t *) fp_data + data_pos;
  int16_t * out_d_pos = (int16_t *) in_d_pos + w*w*255;

  //local variables
  int i, j;
  unsigned int output_pos;
  int16_t fp2375 = 0x260, fp084375 = 0xD8, fp0625 = 0xA0, fp05 = 0x80; //Q8.8
  int16_t val_in, val_out;
  int16_t fp5 = 0x500, fp1 = 0x100; //Q8.8

  //perform yolo layer
  for(i = 0; i < 255; i++) {            //Number of kernels
    for(j = 0; j < w*w; j++) {   	//Output map size
      output_pos = i*w*w + j;
      val_in = in_d_pos[output_pos]; //Q8.8
      if(i != 2 && i != 3 && i != 87 && i != 88 && i != 172 && i != 173) {

        //Sigmoid linear approximation
	if(val_in < 0.) val_out = ~val_in + 1; //emulates multiplying by -1
	else val_out = val_in;

	if(val_out >= fp5) val_out = fp1;
	else if(val_out >= fp2375) val_out = fp084375 + (val_out >> 5); //emulates multiplying by 0.03125 = 2^(-5)
	else if(val_out >= fp1) val_out = fp0625 + (val_out >> 3); //emulates multiplying by 0.125 = 2^(-3)
	else val_out = fp05 + (val_out >> 2); //emulates multiplying by 0.25 = 2^(-2); 
	  
	if(val_in > 0.) out_d_pos[output_pos] = val_out;
	else out_d_pos[output_pos] = fp1 - val_out;
      } else out_d_pos[output_pos] = val_in;
    }
  }

  //update data pointer
  data_pos += w*w*255;
 }

 //perform upsample layer
 void upsample_layer(int w, int num_ker) {

  //locate data pointers
  int16_t * in_d_pos = (int16_t *) fp_data + data_pos;
  int16_t * out_d_pos = (int16_t *) in_d_pos + w*w*num_ker;

  //local variables
  int i, j, k, l, m, output_w = w*2+2;
  int16_t val, val2;

  //perform upsampling
  for(i = 0; i < num_ker; i+=2) { 		//Number of kernels
    for(j = 0; j < w; j++) {   			//Output map size
      for(k = 0; k < w; k++) {
	val = in_d_pos[i*w*w + j*w + k];
	val2 = in_d_pos[(i+1)*w*w + j*w + k];
	for(l = 0; l < 2; l++) {		//2x2 block
	  for(m = 0; m < 2; m++) {
	    out_d_pos[i*output_w*output_w + j*output_w*2 + k*2 + l*output_w + m + (1+output_w)] = val;
	    out_d_pos[(i+1)*output_w*output_w + j*output_w*2 + k*2 + l*output_w + m + (1+output_w)] = val2;
	  }
	}
      }
    }
  }

  //update data pointer
  data_pos += w*w*num_ker;
 }

 //Polynomial approximation of exponential function
 int16_t exp_poly_appr(int16_t val) {
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

 //Get candidate boxes from yolo layers output
 void get_boxes(int w, int16_t xy_div, int first_yolo, unsigned int in_pos, unsigned int out_pos) {

  //locate data pointers
  int16_t * in_d_pos = (int16_t *) fp_data + in_pos;
  int16_t * out_d_pos = (int16_t *) fp_data + out_pos;

  //local variables
  int16_t i, j, k, m;
  int16_t val_16, obj_score, pred_score;
  int32_t val_32;
  int16_t yolo_bias[12] = {0x0280, 0x0380, 0x05C0, 0x06C0, 0x0940, 0x0E80, 0x1440, 0x1480, 0x21C0, 0x2A40, 0x5600, 0x4FC0}; //Q10.6

  //loops to go through yolo layer output
  for(i = 0; i < 3; i++) {
    for(j = 0; j < w; j++) {
      for(k = 0; k < w; k++) {
	if(in_d_pos[(85*i+4)*w*w + j*w + k] > threshold) {

  	  //Calculate x
	  val_16 = in_d_pos[85*i*w*w + j*w + k]; //Q8.8
	  val_32 = (int32_t)((int32_t)(val_16 + (k<<8))*(int32_t)xy_div); //Q8.8 *Q1.15 = Q9.23
	  val_16 = (int16_t)(val_32 >> 9); //Q9.23 to Q2.14
	  out_d_pos[85*nboxes] = val_16; //x

	  //Calculate y
	  val_16 = in_d_pos[(85*i+1)*w*w + j*w + k]; //Q8.8
	  val_32 = (int32_t)((int32_t)(val_16 + (j<<8))*(int32_t)xy_div); //Q8.8 *Q1.15 = Q9.23
	  val_16 = (int16_t)(val_32 >> 9); //Q9.23 to Q2.14
	  val_32 = (int32_t)((int32_t)val_16*(int32_t)y_scales); //Q2.14 * Q2.14 = Q4.28
	  val_16 = (int16_t)(val_32 >> 14); //Q4.28 to Q2.14
	  val_16 -= (int16_t)y_bias; //Q2.14
	  out_d_pos[85*nboxes+1] = val_16; //y

	  //Calculate w
	  val_16 = exp_poly_appr(in_d_pos[(85*i+2)*w*w + j*w + k]); //Q2.14
	  val_32 = (int32_t)((int32_t)val_16*(int32_t)w_scales); //Q2.14 * Q2.14 = Q4.28
	  val_16 = (int16_t)(val_32 >> 14); //Q4.28 to Q2.14
	  val_32 = (int32_t)((int32_t)val_16*(int32_t)yolo_bias[2*(i+3*first_yolo)]); //Q2.14 * Q10.6 = Q12.20
	  val_16 = (int16_t)(val_32 >> 6); //Q12.20 to Q2.14
	  out_d_pos[85*nboxes+2] = val_16; //w

	  //Calculate h
	  val_16 = exp_poly_appr(in_d_pos[(85*i+3)*w*w + j*w + k]); //Q2.14
	  val_32 = (int32_t)((int32_t)val_16*(int32_t)h_scales); //Q2.14 * Q2.14 = Q4.28
	  val_16 = (int16_t)(val_32 >> 14); //Q4.28 to Q2.14
	  val_32 = (int32_t)((int32_t)val_16*(int32_t)yolo_bias[2*(i+3*first_yolo)+1]); //Q2.14 * Q10.6 = Q12.20
	  val_16 = (int16_t)(val_32 >> 6); //Q12.20 to Q2.14
	  out_d_pos[85*nboxes+3] = val_16; //h

	  //Objectness score
	  obj_score = in_d_pos[(85*i+4)*w*w + j*w + k]; //Q8.8
	  obj_score = obj_score << 6; //Q8.8 to Q2.14
	  out_d_pos[85*nboxes+4] = obj_score;

	  //Calculate probability scores
	  for(m = 0; m < 80; m++) {
 	    val_16 = in_d_pos[(85*i+5+m)*w*w + j*w + k]; //Q8.8
	    val_32 = (int32_t)((int32_t)val_16*(int32_t)obj_score); //Q8.8 * Q2.14 = Q10.22
	    pred_score = (int16_t)(val_32 >> 8); //Q10.22 to Q2.14
	    if(pred_score <= (threshold << 6)) pred_score = 0; //Q2.14
	    out_d_pos[85*nboxes+5+m] = pred_score; // prediction scores
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
 void filter_boxes(unsigned int pos) {

  //locate data pointers
  int16_t * in_d_pos = (int16_t *) fp_data + pos;
  uint8_t * out_d_pos = (uint8_t *) fp_data + (pos + 85*nboxes)*2;

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
      if(in_d_pos[85*j+5+i] != 0) {

        //Store box ID in descending order of prob score
	if(obj_cnt == 0) out_d_pos[0] = j;
	else {

	  //Search for position of new box ID
	  for(k = 0; k < obj_cnt; k++) if(in_d_pos[85*j+5+i] > in_d_pos[85*out_d_pos[k]+5+i]) break;

	  //Store box ID
	  if(k < obj_cnt) for(l = obj_cnt; l > k; l--) out_d_pos[l] = out_d_pos[l-1];
	  out_d_pos[k] = j; //min prob score
	}

	//Update object counter
	obj_cnt++;
      }
    }

    //Apply NMS if more than 1 object from same class was detected
    if(obj_cnt > 1) {
      for(j = 0; j < obj_cnt; j++) {
 	if(in_d_pos[85*out_d_pos[j]+5+i] == 0) continue;
	for(k = j+1; k < obj_cnt; k++) {

	  //Get boxes coordinates
	  x1 = in_d_pos[85*out_d_pos[j]];
	  y1 = in_d_pos[85*out_d_pos[j]+1];
	  w1 = in_d_pos[85*out_d_pos[j]+2];
	  h1 = in_d_pos[85*out_d_pos[j]+3];
	  x2 = in_d_pos[85*out_d_pos[k]];
	  y2 = in_d_pos[85*out_d_pos[k]+1];
	  w2 = in_d_pos[85*out_d_pos[k]+2];
	  h2 = in_d_pos[85*out_d_pos[k]+3];

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
	    if(b_iou > nms_threshold) in_d_pos[85*out_d_pos[k]+5+i] = 0;
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
  uint16_t mul_16;
  int l, k;
  for(l = 0; l < label_height && (l+top_width) < IMG_H; l++){
    for(k = 0; k < label_w && (k+left+previous_w) < IMG_W; k++){
      fp_image[(l+top_width)*IMG_W+(k+left+previous_w)] = ((uint16_t)((uint16_t)r*(uint16_t)fp_labels[(81+MAX_LABEL_SIZE*j)+l*label_w+k])) >> 8; //Q8.0*Q8.0 to Q8.0 -> red
      fp_image[IMG_W*IMG_H+(l+top_width)*IMG_W+(k+left+previous_w)] = ((uint16_t)((uint16_t)g *(uint16_t)fp_labels[(81+MAX_LABEL_SIZE*j)+l*label_w+k])) >> 8; //green
      fp_image[2*IMG_W*IMG_H+(l+top_width)*IMG_W+(k+left+previous_w)] = ((uint16_t)((uint16_t)b *(uint16_t)fp_labels[(81+MAX_LABEL_SIZE*j)+l*label_w+k])) >> 8; //blue
    }
  }	
 }

 //Draw detections (bounding boxes and class labels) in input image
 void draw_detections(unsigned int pos) {

  //locate data pointers
  int16_t * in_d_pos = (int16_t *) fp_data + pos;

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
      if(in_d_pos[85*i+5+j] != 0) {

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
	  mul_16 = in_d_pos[85*i] - (in_d_pos[85*i+2]>>1); //Q2.14
	  mul_32 = (int32_t)((int32_t)mul_16 * (int32_t)IMG_W); //Q2.14 * Q16.0 = Q18.14
	  left = (mul_32 >> 14);					
	  mul_16 = in_d_pos[85*i] + (in_d_pos[85*i+2]>>1); //Q2.14
	  mul_32 = (int32_t)((int32_t)mul_16 * (int32_t)IMG_W); //Q2.14 * Q16.0 = Q18.14
	  right = (mul_32 >> 14);
	  mul_16 = in_d_pos[85*i+1] - (in_d_pos[85*i+3]>>1); //Q2.14
	  mul_32 = (int32_t)((int32_t)mul_16 * (int32_t)IMG_H); //Q2.14 * Q16.0 = Q18.14
	  top = (mul_32 >> 14);
	  mul_16 = in_d_pos[85*i+1] + (in_d_pos[85*i+3]>>1); //Q2.14
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
 void print_results(unsigned int box_pos) {
   int i, j;
   uint32_t pred_32;
   const char *class_names[80] = {"person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed", "dining table", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};
  for(i = 0; i < nboxes; i++) {
    for(j = 0; j < 80; j++) {
      if(fp_data[box_pos+85*i+5+j] != 0) {
	pred_32 = (uint32_t)((uint32_t)fp_data[box_pos+85*i+5+j]*(uint32_t)100); //Q2.14 * Q16.0 = Q18.14
	if( (pred_32&0x3FFF) > 0x2000) uart_printf("\n%s: %d%%", class_names[j], (pred_32>>14)+1);
	else uart_printf("\n%s: %d%%", class_names[j], (pred_32>>14));
      }
    }
  }
  uart_printf("\n");	
 }

#else

 #include <math.h>

 //Constants for image resize
 #define w_scale ((float)(IMG_W-1)/(NEW_W-1))
 #define h_scale ((float)(IMG_H-1)/(NEW_H-1))

 //Constants for bounding boxes
 #define threshold ((float)0.5)
 #define nms_threshold ((float)0.45)
 #define yolo1_div ((float)1/LAYER_17_W)
 #define yolo2_div ((float)1/LAYER_24_W)
 #define y_scales ((float)NEW_W/NEW_H)
 #define y_bias ((float)(NEW_W-NEW_H)/(NEW_H*2))
 #define w_scales ((float)1/NEW_W)
 #define h_scales ((float)1/NEW_H)
 #define c3 (pow(2,-3)+pow(2,-5)+pow(2,-7)+pow(2,-9)+pow(2,-11)+pow(2,-13))
 #define c4 (pow(2,-5)+pow(2,-7)+pow(2,-8))

 //define general constants
 #define ETH_NBYTES (1022-18) //1024-2 to be divisible by 4
 #define INPUT_FILE_SIZE (IMAGE_INPUT*4) //32 bits per point
 #define NUM_INPUT_FRAMES (INPUT_FILE_SIZE/ETH_NBYTES)
 #define WEIGHTS_FILE_SIZE (35409464) //32 bits per input
 #define NUM_WEIGHT_FRAMES (WEIGHTS_FILE_SIZE/ETH_NBYTES)
 #define LABELS_FILE_SIZE ((82+MAX_LABEL_SIZE*81)*4) //32 bits per input (82 to keep it align)
 #define GEMM_IN_SIZE (NTW_IN_W*NTW_IN_H*NTW_IN_NUM_KER*NTW_IN_KER_SIZE*NTW_IN_KER_SIZE*4) //32 bits per input
 #define GEMM_OUT_SIZE (NTW_IN_W*NTW_IN_H*NTW_IN_NUM_KER*4) //32 bits per input

 //define DDR mapping
 #define WEIGTHS_BASE_ADDRESS (DDR_MEM + 0x00008000) //16kb for program + 16kb for stack
 #define LABEL_BASE_ADDRESS (WEIGTHS_BASE_ADDRESS + WEIGHTS_FILE_SIZE)
 #define GEMM_IN_BASE_ADDRESS (LABEL_BASE_ADDRESS + LABELS_FILE_SIZE)
 #define GEMM_OUT_BASE_ADDRESS (GEMM_IN_BASE_ADDRESS + GEMM_IN_SIZE)
 #define DATA_BASE_ADDRESS (GEMM_OUT_BASE_ADDRESS + GEMM_OUT_SIZE)

 //define intermediate data constants
 #define INTERM_LAYER1_SIZE (NTW_IN_W*4) //32 bits per input
 #define NUM_INTERM_LAYER1_FRAMES (INTERM_LAYER1_SIZE/ETH_NBYTES)
 #define INTERM_LAYER2_SIZE ((LAYER_3_W+2)*2*4) //32 bits per input
 #define NUM_INTERM_LAYER2_FRAMES (INTERM_LAYER2_SIZE/ETH_NBYTES)
 #define INTERM_LAYER4_SIZE ((LAYER_5_W+2)*2*4) //32 bits per input
 #define NUM_INTERM_LAYER4_FRAMES (INTERM_LAYER4_SIZE/ETH_NBYTES)
 #define INTERM_LAYER5_SIZE (LAYER_5_W*4) //32 bits per input
 #define NUM_INTERM_LAYER5_FRAMES (INTERM_LAYER5_SIZE/ETH_NBYTES)
 #define INTERM_LAYER6_SIZE ((LAYER_7_W+2)*2*4) //32 bits per input
 #define NUM_INTERM_LAYER6_FRAMES (INTERM_LAYER6_SIZE/ETH_NBYTES)
 #define INTERM_LAYER8_SIZE ((LAYER_9_W+2)*2*4) //32 bits per input
 #define NUM_INTERM_LAYER8_FRAMES (INTERM_LAYER8_SIZE/ETH_NBYTES)
 #define INTERM_LAYER9_SIZE ((LAYER_9_W+2)*4) //32 bits per input
 #define NUM_INTERM_LAYER9_FRAMES (INTERM_LAYER9_SIZE/ETH_NBYTES)

 //Constants for bounding boxes
 #define box_width 3
 #define label_height 20
 uint8_t nboxes = 0;

 //weights and data updatable pointers
 unsigned int weight_pos = 0, data_pos = 0;

 //ETHERNET variables
 int rcv_timeout = 5000;
 char data_to_send[ETH_NBYTES];
 char data_rcv[ETH_NBYTES+18];
 unsigned int bytes_to_receive, bytes_to_send, count_bytes;

 //TIMER variables
 unsigned int start, end;

 //weights and data base address pointers
 float *fp_weights;
 float *fp_data;
 float * fp_image;
 float * fp_labels;
#ifdef GEMM
 float * fp_gemm_in;
 float * fp_gemm_out;
#endif

 //define base adress of weights and data pointers
 void define_memory_regions() {
 			
   //image
   fp_image = (float *) DATA_BASE_ADDRESS;
 									
   //data
   fp_data = (float *) (DATA_BASE_ADDRESS + IMAGE_INPUT);
 															
   //weights
   fp_weights = (float *) WEIGTHS_BASE_ADDRESS;

   //labels
   fp_labels = (float *) LABEL_BASE_ADDRESS;
 
 #ifdef GEMM
   fp_gemm_in = (float *) GEMM_IN_BASE_ADDRESS;
   fp_gemm_out = (float *) GEMM_OUT_BASE_ADDRESS;
 #endif
 }

 //fill part of 416x316 region of resized image with grey (0.5)
 void fill_grey() {
  int i, j, k;
  for(i = 0; i < NTW_IN_C; i++) {
    for(j = 0; j < EXTRA_H; j++) {
      for(k = 0; k < NTW_IN_W; k++) {
	fp_data[i*(NTW_IN_W+2)*(NTW_IN_H+2) + (j+GREY_PADD)*(NTW_IN_W+2) + (k+1)] = 0.5; //1st region
	fp_data[i*(NTW_IN_W+2)*(NTW_IN_H+2) + (j+GREY_PADD)*(NTW_IN_W+2) + (k+1) + ((NTW_IN_W+2)*(EXTRA_H+NEW_H))] = 0.5; //2nd region
      }
    }
  }
 }

 //resize input image to 416x312
 void resize_image() {

  //local variables
  int r, c, k;
  int iy, ix, new_r;
  float sx, sy, dy, dx;
  float val_h, val_w, next_val_w;
				
  for(k = 0; k < NTW_IN_C; k++) { 				// 3
    for(c = 0; c < NTW_IN_W; c++) {				// 416
      new_r = 1;
				
      //Width index calculation
      if(c == NTW_IN_W-1) val_w = fp_image[k*IMG_W*IMG_H + (IMG_W-1)];
      else {
	sx = c*w_scale;		
	ix = (int) sx;
	dx = sx - ix;
	val_w = (1.-dx)*fp_image[k*IMG_W*IMG_H + ix] + dx*fp_image[k*IMG_W*IMG_H + (ix+1)];
      }
				
      for(r = 0; r < NEW_H; r++) {				// 312

	//Width reduction (to 416)
	if(c == NTW_IN_W-1) next_val_w = fp_image[k*IMG_W*IMG_H + new_r*IMG_W + (IMG_W-1)];
	else next_val_w = (1.-dx)*fp_image[k*IMG_W*IMG_H + new_r*IMG_W + ix] + dx*fp_image[k*IMG_W*IMG_H + new_r*IMG_W + (ix+1)];
					
	//Height index calculation
	if(r != NEW_H-1) {
  	  sy = r*h_scale;
	  iy = (int) sy;
	  dy = sy - iy;
						
	  //Check if to calculate next width
	  if( (iy+1) != new_r) {
	    new_r++;
	    val_w = next_val_w;					
	    if(c == NTW_IN_W-1) next_val_w = fp_image[k*IMG_W*IMG_H + new_r*IMG_W + (IMG_W-1)];
	    else next_val_w = (1.-dx)*fp_image[k*IMG_W*IMG_H + new_r*IMG_W + ix] + dx*fp_image[k*IMG_W*IMG_H + new_r*IMG_W + (ix+1)];
	  }
						
	  //Height reduction (to 312)
	  val_h = (1.-dy)*val_w + dy*next_val_w;
	} else val_h = next_val_w;
					
	//Save new value
	fp_data[k*(NTW_IN_W+2)*(NTW_IN_H+2) + (r+GREY_PADD)*(NTW_IN_W+2) + (c+1) + ((NTW_IN_W+2)*EXTRA_H)] = val_h;
					
	//Update variables
	new_r++;
	val_w = next_val_w;
      }
    }
  }
 }

 //perform convolutional layer
 void conv_layer(int w, int h, int c, int num_ker, int ker_size, int pad, int batch_norm, int nextPadding, int nextStride, int ignorePadding, unsigned int new_output_pos, unsigned int offset) {

  //locate weight and data pointers
  unsigned int new_h, new_h_output, out_offset, new_w;
  if(nextStride) new_w = w+1; else if(nextPadding) new_w = w+2; else new_w = w;
  if(w == h) { 
    new_h = w+2*pad;
    out_offset = 0;
    new_h_output = new_w;
  } else { 
    new_h = h+2;
    out_offset = offset;
    if(offset == 0) new_h_output = h; else new_h_output = new_h;
  }
  unsigned int pos_delta = (w+2*pad)*new_h*c;
  float * w_pos;
  w_pos = (float *) fp_weights + weight_pos;
  float * bias_pos = (float *) fp_weights + weight_pos + num_ker*ker_size*ker_size*c;
  float * in_d_pos = (float *) fp_data + data_pos;
  float * out_d_pos;
  if(new_output_pos != 0) out_d_pos = (float *) fp_data + new_output_pos; else out_d_pos = (float *) in_d_pos + pos_delta;
		
  //local variables
  int i, j, k, l, m, n;
  unsigned int output_pos;
  float output_conv, leaky = 0.1;

 #ifdef GEMM

  //Transformation
  for(l = 0; l < c; l++)  		//Number of channels
    for(m = 0; m < ker_size; m++) 	//Kernel size
      for(n = 0; n < ker_size; n++) 
	for(j = 0; j < h; j++)   	//Output map size
	  for(k = 0; k < w; k++) 
	    fp_gemm_in[l*ker_size*ker_size*h*w + m*ker_size*h*w + n*h*w + j*w + k] = in_d_pos[(j+ignorePadding)*(w+2*pad) + (k+ignorePadding) + l*(w+2*pad)*new_h + m*(w+2*pad) + n];

  //Convolution
  for(i = 0; i < num_ker; i++) {
    for(j = 0; j < c*ker_size*ker_size; j++) {
      for(k = 0; k < w*h; k++) {
	if(j == 0) fp_gemm_out[i*w*h + k] = w_pos[i*c*ker_size*ker_size + j]*fp_gemm_in[j*w*h + k];
	else fp_gemm_out[i*w*h + k] += w_pos[i*c*ker_size*ker_size + j]*fp_gemm_in[j*w*h + k];
      }
    }
  }

  //Result
  for(i = 0; i < num_ker; i++) {
    for(j = 0; j < h; j++) { 
      for(k = 0; k < w; k++) {

	//Calculate output position
	if(nextPadding) output_pos = i*new_w*new_w + (j+1)*new_w + (k+1) + (out_offset*new_w);
	else output_pos = i*new_w*new_h_output + j*new_w + k + (out_offset*new_w);

	//perform batch normalize
	if(batch_norm) {
	  output_conv = fp_gemm_out[i*w*h + j*w + k]*bias_pos[i] + bias_pos[num_ker+i];
	  if(output_conv < 0) output_conv *= leaky;
	  out_d_pos[output_pos] = output_conv;

	  //Copy last column and last row if needed
	  if(nextStride) {
	    if(k == w-1) out_d_pos[output_pos + 1] = output_conv; 
	    if(j == w-1) out_d_pos[output_pos + new_w] = output_conv;
	    if(k == w-1 && j == w-1) out_d_pos[output_pos + 1 + new_w] = output_conv;
	  }
	} else out_d_pos[output_pos] = fp_gemm_out[i*w*h + j*w + k] + bias_pos[i];
      }
    }
  }

 #else
	
  //local variables
  unsigned int output_pos2, output_pos3, output_pos4;
  float op1, op2, op2_2, op2_3, op2_4;
  float output_conv2, output_conv3, output_conv4;
  float acc_final, acc_final2, acc_final3, acc_final4;

  //perform convolution
  for(i = 0; i < num_ker; i+=4) {	//Number of kernels
    for(j = 0; j < h; j++) {   		//Output map size
      for(k = 0; k < w; k++) {
  	if(nextPadding) {
	  output_pos = i*new_w*new_w + (j+1)*new_w + (k+1) + (out_offset*new_w); 
	  output_pos2 = (i+1)*new_w*new_w + (j+1)*new_w + (k+1) + (out_offset*new_w);
	  output_pos3 = (i+2)*new_w*new_w + (j+1)*new_w + (k+1) + (out_offset*new_w); 
	  output_pos4 = (i+3)*new_w*new_w + (j+1)*new_w + (k+1) + (out_offset*new_w); 
	} else {
	  output_pos = i*new_w*new_h_output + j*new_w + k + (out_offset*new_w);
	  output_pos2 = (i+1)*new_w*new_h_output + j*new_w + k + (out_offset*new_w);
	  output_pos3 = (i+2)*new_w*new_h_output + j*new_w + k + (out_offset*new_w);
	  output_pos4 = (i+3)*new_w*new_h_output + j*new_w + k + (out_offset*new_w);
	}
	acc_final = 0;
	acc_final2 = 0;
	acc_final3 = 0;
	acc_final4 = 0;
	for(l = 0; l < c; l++) { 		//Number of channels
	  for(m = 0; m < ker_size; m++) {	//Kernel size
	    for(n = 0; n < ker_size; n++) {
	      op1 = in_d_pos[(j+ignorePadding)*(w+2*pad) + (k+ignorePadding) + l*(w+2*pad)*new_h + m*(w+2*pad) + n];
	      op2 = w_pos[i*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n];
	      acc_final += op1*op2;
	      op2_2 = w_pos[(i+1)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n];
	      acc_final2 += op1*op2_2;
	      op2_3 = w_pos[(i+2)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n];
	      acc_final3 += op1*op2_3;
	      op2_4 = w_pos[(i+3)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n];
	      acc_final4 += op1*op2_4;
	    }
	  }
	}
					
	//perform batch normalize
	if(batch_norm) {
	  output_conv = acc_final*bias_pos[i] + bias_pos[num_ker+i];
	  output_conv2 = acc_final2*bias_pos[i+1] + bias_pos[num_ker+i+1];
	  output_conv3 = acc_final3*bias_pos[i+2] + bias_pos[num_ker+i+2];
	  output_conv4 = acc_final4*bias_pos[i+3] + bias_pos[num_ker+i+3];
														
	  //perform leaky activation
	  if(output_conv < 0) output_conv *= leaky;
	  if(output_conv2 < 0) output_conv2 *= leaky;
	  if(output_conv3 < 0) output_conv3 *= leaky;
	  if(output_conv4 < 0) output_conv4 *= leaky;
	  out_d_pos[output_pos] = output_conv;
	  out_d_pos[output_pos2] = output_conv2;
	  out_d_pos[output_pos3] = output_conv3;
	  out_d_pos[output_pos4] = output_conv4;
						
	  //Copy last column and last row if needed
	  if(nextStride) {
	    if(k == w-1) { 
	      out_d_pos[output_pos + 1] = output_conv; 
	      out_d_pos[output_pos2 + 1] = output_conv2; 
	      out_d_pos[output_pos3 + 1] = output_conv3;
	      out_d_pos[output_pos4 + 1] = output_conv4;
	    }
	    if(j == w-1) { 
	      out_d_pos[output_pos + new_w] = output_conv; 
	      out_d_pos[output_pos2 + new_w] = output_conv2;
	      out_d_pos[output_pos3 + new_w] = output_conv3;
	      out_d_pos[output_pos4 + new_w] = output_conv4;							
	    }
	    if(k == w-1 && j == w-1) { 
	      out_d_pos[output_pos + 1 + new_w] = output_conv; 
	      out_d_pos[output_pos2 + 1 + new_w] = output_conv2; 
	      out_d_pos[output_pos3 + 1 + new_w] = output_conv3; 
	      out_d_pos[output_pos4 + 1 + new_w] = output_conv4; 
	    }
	  }
	}
					
	//otherwise, only add bias
	else {
	  out_d_pos[output_pos] = acc_final + bias_pos[i];
	  out_d_pos[output_pos2] = acc_final2 + bias_pos[i+1];
	  out_d_pos[output_pos3] = acc_final3 + bias_pos[i+2];
	  out_d_pos[output_pos4] = acc_final4 + bias_pos[i+3];
	}
      }
    }
  }

 #endif
		
  //update weights and data pointers
  if(batch_norm) weight_pos += num_ker*2 + num_ker*c*ker_size*ker_size;
  else weight_pos += num_ker + num_ker*c*ker_size*ker_size;
  if(new_output_pos != 0) data_pos = new_output_pos; else data_pos += pos_delta;
 }	
	
 //perform maxpool layer
 void maxpool_layer(int w, int h, int num_ker, int downsample, int ignorePadding, unsigned int new_output_pos) {
		
  //locate data pointers
  unsigned int new_h, out_offset, new_h_output, output_w = (w/(1+downsample))+2;
  if(w == h) { 
    new_h = w+1-downsample+2*ignorePadding;
    out_offset = 0;
    new_h_output = output_w;
  } else { 
    new_h = h;
    out_offset = 1;
    new_h_output = h/2+4;
  }
  float * in_d_pos = (float *) fp_data + data_pos;
  float * out_d_pos;
  if(new_output_pos != 0) out_d_pos = (float *) fp_data + new_output_pos; 
  else out_d_pos = (float *) in_d_pos + (w+1-downsample+2*ignorePadding)*new_h*num_ker;
		
  //local variables
  int i, j, k, l, m, new_w = w+1-downsample+2*ignorePadding;
  float max, max2, max3, max4, val, val2, val3, val4;
		
  //perform max pooling
  for(i = 0; i < num_ker; i+=4) { 							//Number of kernels
    for(j = 0; j < h/(1+downsample); j++) {   				//Output map size
      for(k = 0; k < w/(1+downsample); k++) { 
    	for(l = 0; l < 2; l++) {						//2x2 block
	  for(m = 0; m < 2; m++) {
	    val = in_d_pos[i*new_w*new_h + j*new_w*(1+downsample) + k*(1+downsample) + l*new_w + m + (1 + new_w)*ignorePadding];
	    val2 = in_d_pos[(i+1)*new_w*new_h + j*new_w*(1+downsample) + k*(1+downsample) + l*new_w + m + (1 + new_w)*ignorePadding];
	    val3 = in_d_pos[(i+2)*new_w*new_h + j*new_w*(1+downsample) + k*(1+downsample) + l*new_w + m + (1 + new_w)*ignorePadding];
	    val4 = in_d_pos[(i+3)*new_w*new_h + j*new_w*(1+downsample) + k*(1+downsample) + l*new_w + m + (1 + new_w)*ignorePadding];
	    if(l == 0 && m == 0) { max = val; max2 = val2; max3 = val3; max4 = val4; }
	    else { if(max < val) max = val; if(max2 < val2) max2 = val2; if(max3 < val3) max3 = val3; if(max4 < val4) max4 = val4; }
	  }
	}
	out_d_pos[i*output_w*new_h_output + (j+1)*output_w + (k+1) + out_offset*output_w] = max;
	out_d_pos[(i+1)*output_w*new_h_output + (j+1)*output_w + (k+1) + out_offset*output_w] = max2;
	out_d_pos[(i+2)*output_w*new_h_output + (j+1)*output_w + (k+1) + out_offset*output_w] = max3;
	out_d_pos[(i+3)*output_w*new_h_output + (j+1)*output_w + (k+1) + out_offset*output_w] = max4;
      }
    }
  }
		
  //update data pointer
  if(new_output_pos != 0) data_pos = new_output_pos;
  else data_pos += (w+1-downsample+2*ignorePadding)*new_h*num_ker;
 }

 //perform yolo layer
 void yolo_layer(int w) {
		
  //locate data pointers
  float * in_d_pos = (float *) fp_data + data_pos;
  float * out_d_pos = (float *) in_d_pos + w*w*255;
		
  //local variables
  int i, j;
  unsigned int output_pos;
  float val_in, val_out;
		
  //perform yolo layer
  for(i = 0; i < 255; i++) { 				//Number of kernels
    for(j = 0; j < w*w; j++) {   		//Output map size
      output_pos = i*w*w + j;	
      val_in = in_d_pos[output_pos]; 
      if(i != 2 && i != 3 && i != 87 && i != 88 && i != 172 && i != 173) {

	//Sigmoid linear approximation
	if(val_in < 0.) val_out = -val_in;
	else val_out = val_in;

   	if(val_out >= 5.) val_out = 1;
	else if(val_out >= 2.375) val_out = 0.03125*val_out+0.84375;
	else if(val_out >= 1.) val_out = 0.125*val_out+0.625;
	else val_out = 0.25*val_out+0.5;
					
	if(val_in > 0.) out_d_pos[output_pos] = val_out;
	else out_d_pos[output_pos] = 1. - val_out;
      }  else out_d_pos[output_pos] = val_in;	
    }
  }
		
  //update data pointer
  data_pos += w*w*255;
 }
		
 //perform upsample layer
 void upsample_layer(int w, int num_ker) {

  //locate data pointers
  float * in_d_pos = (float *) fp_data + data_pos;
  float * out_d_pos = (float *) in_d_pos + w*w*num_ker;
		
  //local variables
  int i, j, k, l, m, output_w = w*2+2;
  float val, val2;
		
  //perform upsampling
  for(i = 0; i < num_ker; i+=2) { 								//Number of kernels
    for(j = 0; j < w; j++) {   									//Output map size
      for(k = 0; k < w; k++) {
	val = in_d_pos[i*w*w + j*w + k];
	val2 = in_d_pos[(i+1)*w*w + j*w + k];
	for(l = 0; l < 2; l++) {							//2x2 block
	  for(m = 0; m < 2; m++) {
	    out_d_pos[i*output_w*output_w + j*output_w*2 + k*2 + l*output_w + m + (1+output_w)] = val;
	    out_d_pos[(i+1)*output_w*output_w + j*output_w*2 + k*2 + l*output_w + m + (1+output_w)] = val2;
	  }
	}
      }
    }
  }
		
  //update data pointer
  data_pos += w*w*num_ker;
 }

 //Polynomial approximation of exponential function
 float exp_poly_appr(float val) {
  float val_aux, exp_val_fixed;
  exp_val_fixed = val + 1;
  val_aux = val*val; //w^2
  exp_val_fixed += 0.5*val_aux; //1+w+0.5*w^2
  val_aux *= val; //w^3
  exp_val_fixed += c3*val; //1+w+0.5*w^2+c3*w^3
  val_aux *= val; //w^4
  exp_val_fixed += c4*val_aux; //1+w+0.5*w^2+c3*w^3+c4*w^4
  return exp_val_fixed;
 }

 //Get candidate boxes from yolo layers output
 void get_boxes(int w, float xy_div, int first_yolo, unsigned int in_pos, unsigned int out_pos) {
		
  //locate data pointers
  float * in_d_pos = (float *) fp_data + in_pos;
  float * out_d_pos = (float *) fp_data + out_pos;
  						
  //local variables
  int16_t i, j, k, m;
  float pred_score;
  int yolo_bias[12] = {10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319};
  																
  //loops to go through yolo layer output
  for(i = 0; i < 3; i++) {
    for(j = 0; j < w; j++) {
      	for(k = 0; k < w; k++) {
	  if(in_d_pos[(85*i+4)*w*w + j*w + k] > threshold) {
	    
	    //Calculate x
	    out_d_pos[85*nboxes] = (in_d_pos[85*i*w*w + j*w + k]+k)*xy_div;
		
	    //Calculate y
	    out_d_pos[85*nboxes+1] = (in_d_pos[(85*i+1)*w*w + j*w + k]+j)*xy_div*y_scales - y_bias;
	
	    //Calculate w
	    out_d_pos[85*nboxes+2] = exp_poly_appr(in_d_pos[(85*i+2)*w*w + j*w + k])*w_scales*yolo_bias[2*(i+3*first_yolo)];
	    
            //Calculate h
	    out_d_pos[85*nboxes+3] = exp_poly_appr(in_d_pos[(85*i+3)*w*w + j*w + k])*h_scales*yolo_bias[2*(i+3*first_yolo)+1];
	
	    //Objectness score
	    out_d_pos[85*nboxes+4] = in_d_pos[(85*i+4)*w*w + j*w + k];
	
            //Calculate probability scores
            for(m = 0; m < 80; m++) {
	      pred_score = in_d_pos[(85*i+5+m)*w*w + j*w + k]*in_d_pos[(85*i+4)*w*w + j*w + k];
	      if(pred_score <= threshold) pred_score = 0;
	      out_d_pos[85*nboxes+5+m] = pred_score;
	    }
	  
	  //Update number of candidate boxes
	  nboxes++;
        }
      }
    }
  }
 }

 //Calculate overlapp between 2 boxes
 float overlap(float x1, float w1, float x2, float w2) {
  float l1, l2, left, r1, r2, right;
  l1 = x1 - (w1/2);
  l2 = x2 - (w2/2);
  left = l1 > l2 ? l1 : l2;
  r1 = x1 + (w1/2);
  r2 = x2 + (w2/2);
  right = r1 < r2 ? r1 : r2;
  return right - left;
 }

 //Apply non-maximum-suppresion to filter repeated boxes 
 void filter_boxes(unsigned int pos) {

  //locate data pointers
  float * in_d_pos = (float *) fp_data + pos;
  uint8_t * out_d_pos = (uint8_t *) fp_data + (pos + 85*nboxes)*4;
		
  //Local variables
  int i, j, k, l;
  int obj_cnt;
  float w, h, b_union, b_iou, b_inter;
  float x1, y1, w1, h1, x2, y2, w2, h2;
		
  //Loop to go through classes from candidate boxes
  for(i = 0; i < 80; i++) {
			
    //Count number of candidate boxes for given class
    obj_cnt = 0;
    for(j = 0; j < nboxes; j++) {
      if(in_d_pos[85*j+5+i] != 0) {
				
	//Store box ID in descending order of prob score
	if(obj_cnt == 0) out_d_pos[0] = j;
	else {
						
	  //Search for position of new box ID
	  for(k = 0; k < obj_cnt; k++)
	    if(in_d_pos[85*j+5+i] > in_d_pos[85*out_d_pos[k]+5+i])
	      break;
							
	  //Store box ID
	  if(k < obj_cnt) 
 	    for(l = obj_cnt; l > k; l--)
	      out_d_pos[l] = out_d_pos[l-1];
	  out_d_pos[k] = j; //min prob score
	}
			
	//Update object counter
	obj_cnt++;
      }
    }
			
    //Apply NMS if more than 1 object from same class was detected
    if(obj_cnt > 1) {
      for(j = 0; j < obj_cnt; j++) {
	if(in_d_pos[85*out_d_pos[j]+5+i] == 0) continue;
	for(k = j+1; k < obj_cnt; k++) {
					
	  //Get boxes coordinates
	  x1 = in_d_pos[85*out_d_pos[j]];
	  y1 = in_d_pos[85*out_d_pos[j]+1];
	  w1 = in_d_pos[85*out_d_pos[j]+2];
	  h1 = in_d_pos[85*out_d_pos[j]+3];
	  x2 = in_d_pos[85*out_d_pos[k]];
	  y2 = in_d_pos[85*out_d_pos[k]+1];
	  w2 = in_d_pos[85*out_d_pos[k]+2];
	  h2 = in_d_pos[85*out_d_pos[k]+3];
						
	  //Calculate IoU (intersection over union)
	  w = overlap(x1, w1, x2, w2);
  	  h = overlap(y1, h1, y2, h2);
	  if(w > 0 && h > 0) {
	    b_inter = w*h;
	    b_union = w1*h1 + w2*h2 - b_inter;
	    b_iou = b_inter/b_union;						
	    if(b_iou > nms_threshold) in_d_pos[85*out_d_pos[k]+5+i] = 0;
   	  }
	}
      }
    }
  }					
 }

 //Draw bounding box in input image
 void draw_box(int left, int top, int right, int bot, float red, float green, float blue) {

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
 void draw_class(int label_w, int j, int top_width, int left, int previous_w, float r, float g, float b) {
  int l, k;
  for(l = 0; l < label_height && (l+top_width) < IMG_H; l++){
    for(k = 0; k < label_w && (k+left+previous_w) < IMG_W; k++){
      fp_image[(l+top_width)*IMG_W+(k+left+previous_w)] = r*fp_labels[(81+MAX_LABEL_SIZE*j)+l*label_w+k]; //red
      fp_image[IMG_W*IMG_H+(l+top_width)*IMG_W+(k+left+previous_w)] = g*fp_labels[(81+MAX_LABEL_SIZE*j)+l*label_w+k]; //green
      fp_image[2*IMG_W*IMG_H+(l+top_width)*IMG_W+(k+left+previous_w)] = b*fp_labels[(81+MAX_LABEL_SIZE*j)+l*label_w+k]; //blue
    }
  }	
 }

 //Draw detections (bounding boxes and class labels) in input image
 void draw_detections(unsigned int pos) {
		
  //locate data pointers
  float * in_d_pos = (float *) fp_data + pos;
		
  //local variables
  int i, j, k;
  float colors[6][3] = { {1,0,1}, {0,0,1},{0,1,1},{0,1,0},{1,1,0},{1,0,0} };
  float ratio, red, green, blue, label_w;
  int offset, ratio_min, ratio_max;
  int left, right, top, bot, top_width, previous_w;
		
  //Check valid detections
  for(i = 0; i < nboxes; i++) {
			
    //Find detected classes
    previous_w = 0;
    for(j = 0; j < 80; j++) {
      if(in_d_pos[85*i+5+j] != 0) {
        							
	//Check if this was the first class detected for given box
	if(previous_w == 0) {
			
 	  //Randomly pick rgb colors for the box
 	  offset = j*123457 % 80;
	  ratio = offset*0.0625;
	  ratio_min = (int) ratio;
	  ratio_max = ratio_min + 1;
	  ratio -= ratio_min;
	  red = (1.-ratio)*colors[ratio_min][2] + ratio*colors[ratio_max][2];
	  green = (1.-ratio)*colors[ratio_min][1] + ratio*colors[ratio_max][1];
	  blue = (1.-ratio)*colors[ratio_min][0] + ratio*colors[ratio_max][0];
	   					
	  //Calculate box coordinates in image frame
	  left = (in_d_pos[85*i] - (in_d_pos[85*i+2]/2))*IMG_W;				
	  right = (in_d_pos[85*i] + (in_d_pos[85*i+2]/2))*IMG_W;
	  top = (in_d_pos[85*i+1] - (in_d_pos[85*i+3]/2))*IMG_H;
	  bot = (in_d_pos[85*i+1] + (in_d_pos[85*i+3]/2))*IMG_H;				
 
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
 void print_results(unsigned int box_pos) {
  int i, j;
  const char *class_names[80] = {"person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed", "dining table", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};
  for(i = 0; i < nboxes; i++)
    for(j = 0; j < 80; j++)
      if(fp_data[box_pos+85*i+5+j] != 0) 
	uart_printf("\n%s: %.2f%%", class_names[j], fp_data[box_pos+85*i+5+j]*100);
  uart_printf("\n");	
 }

#endif

//reset certain DDR positions to zero due to padding
void reset_DDR() {
  
  //local variables
  unsigned int i, pos;

  //measure initial time
  uart_printf("\nSetting DDR positions to zero\n");
  start = timer_get_count_us(TIMER);

  //input network
  for(i = 0; i < NETWORK_INPUT; i++) fp_data[i] = 0;

  //layer2
  pos = NETWORK_INPUT + DATA_LAYER_1;
  for(i = 0; i < DATA_LAYER_2; i++) fp_data[pos + i] = 0;

  //layer4
  pos += DATA_LAYER_2 + DATA_LAYER_3;
  for(i = 0; i < DATA_LAYER_4; i++) fp_data[pos + i] = 0;

  //layer6
  pos += DATA_LAYER_4 + DATA_LAYER_5;
  for(i = 0; i < DATA_LAYER_6; i++) fp_data[pos + i] = 0;

  //layer8
  pos += DATA_LAYER_6 + DATA_LAYER_7;
  for(i = 0; i < DATA_LAYER_8; i++) fp_data[pos + i] = 0;

  //layer10
  pos += DATA_LAYER_8;
  for(i = 0; i < DATA_LAYER_10; i++) fp_data[pos + i] = 0;

  //layer12
  pos += DATA_LAYER_10 + DATA_LAYER_11;
  for(i = 0; i < DATA_LAYER_12; i++) fp_data[pos + i] = 0;

  //layer14
  pos += DATA_LAYER_12 + DATA_LAYER_13;
  for(i = 0; i < DATA_LAYER_14; i++) fp_data[pos + i] = 0;

  //layer20
  pos += DATA_LAYER_14 + DATA_LAYER_15 + DATA_LAYER_16 + DATA_LAYER_17 + DATA_LAYER_19;
  for(i = 0; i < DATA_LAYER_20; i++) fp_data[pos + i] = 0;
 
  //layer9
  pos += DATA_LAYER_20;
  for(i = 0; i < DATA_LAYER_9; i++) fp_data[pos + i] = 0;

  //measure final time
  end = timer_get_count_us(TIMER);
  uart_printf("DDR reset to zero done in %d ms\n", (end-start)/1000);
}

void rcv_frame(unsigned int pos, unsigned int NUM_DATA_FRAMES, unsigned int DATA_SIZE, int interm_flag, char * data_p, int label_flag) {

  //Local variables
  int i, j;
  char * fp_data_char;
 #ifdef FIXED
  if(interm_flag) fp_data_char = (char *) (DATA_BASE_ADDRESS + IMAGE_INPUT + 2*NETWORK_INPUT + 2*pos);
 #else
  if(interm_flag) fp_data_char = (char *) (DATA_BASE_ADDRESS + 4*IMAGE_INPUT + 4*NETWORK_INPUT + 4*pos);
 #endif
  else fp_data_char = (char *) data_p;
  count_bytes = 0;

  //Loop to receive intermediate data frames
  for(j = 0; j < NUM_DATA_FRAMES+1; j++) {

     //wait to receive frame
     while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout) !=0);

     // start timer
     if(interm_flag == 0 && j == 0 && label_flag == 0) {
       start = timer_get_count_us(TIMER);
     }   

     //check if it is last packet (has less data that full payload size)
     if(j == NUM_DATA_FRAMES) bytes_to_receive = DATA_SIZE - count_bytes;
     else bytes_to_receive = ETH_NBYTES;

     //save in DDR
     for(i = 0; i < bytes_to_receive; i++) {
       fp_data_char[j*ETH_NBYTES + i] = data_rcv[14+i];
       data_to_send[i] = data_rcv[14+i];
     }

     //send data back as ack
     eth_send_frame(data_to_send, ETH_NBYTES);

     //update byte counter
     count_bytes += ETH_NBYTES;
  }
}

//receive weights and data
void receive_data() {

  //local variables
  unsigned int pos = 0, i, label_size;
  uart_printf("\nReady to receive input image, weights, labels and intermediate data\n");

  //Receive input image
  char * fp_image_char = (char *) DATA_BASE_ADDRESS;
  rcv_frame(0, NUM_INPUT_FRAMES, INPUT_FILE_SIZE, 0, fp_image_char, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("image transferred in %d ms\n", (end-start)/1000);
  
  //Receive weights
  char * fp_weights_char = (char *) WEIGTHS_BASE_ADDRESS;
  rcv_frame(0, NUM_WEIGHT_FRAMES, WEIGHTS_FILE_SIZE, 0, fp_weights_char, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("weights transferred in %d ms\n", (end-start)/1000);

  //Receive labels
  start = timer_get_count_us(TIMER);
 #ifdef FIXED
  rcv_frame(0, 0, 81, 0, fp_labels, 1);
  for(i = 0; i < 81; i++) {
    label_size = fp_labels[i]*label_height;
    rcv_frame(0, label_size/ETH_NBYTES, label_size, 0, fp_labels+81+MAX_LABEL_SIZE*i, 1);
  }
 #else
  char * fp_labels_char = (char *) LABEL_BASE_ADDRESS;
  rcv_frame(0, 0, 81*4, 0, fp_labels_char, 1);
  for(i = 0; i < 81; i++) {
    label_size = ((unsigned int)fp_labels[i])*label_height*4;
    fp_labels_char = (char *) LABEL_BASE_ADDRESS + (81 + MAX_LABEL_SIZE*i)*4;
    rcv_frame(0, label_size/ETH_NBYTES, label_size, 0, fp_labels_char, 1);
  }
 #endif
  end = timer_get_count_us(TIMER);
  uart_printf("labels transferred in %d ms\n", (end-start)/1000);

#ifdef INTERM_DATA
  //restart timer
  start = timer_get_count_us(TIMER);

  //loop to receive intermediate layer 1 data
  for(i = 0; i < NTW_IN_NUM_KER; i++) {
    rcv_frame(pos, NUM_INTERM_LAYER1_FRAMES, INTERM_LAYER1_SIZE, 1, 0, 0); //1st line
    pos += (NTW_IN_W*(NTW_IN_H+1));
    rcv_frame(pos, NUM_INTERM_LAYER1_FRAMES, INTERM_LAYER1_SIZE, 1, 0, 0); //2nd line
    pos += NTW_IN_W;
  }

  //loop to receive intermediate layer 2 data
  for(i = 0; i < LAYER_2_NUM_KER; i++) {
    rcv_frame(pos, NUM_INTERM_LAYER2_FRAMES, INTERM_LAYER2_SIZE, 1, 0, 0); //1st line
    pos += (LAYER_3_W+2)*LAYER_3_H;
    rcv_frame(pos, NUM_INTERM_LAYER2_FRAMES, INTERM_LAYER2_SIZE, 1, 0, 0); //2nd line
    pos += (LAYER_3_W+2)*2;
  }

  //loop to receive intermediate layer 4 data
  pos += DATA_LAYER_3;
  for(i = 0; i < LAYER_4_NUM_KER; i++) {
    rcv_frame(pos, NUM_INTERM_LAYER4_FRAMES, INTERM_LAYER4_SIZE, 1, 0, 0); //1st line
    pos += (LAYER_5_W+2)*LAYER_5_H;
    rcv_frame(pos, NUM_INTERM_LAYER4_FRAMES, INTERM_LAYER4_SIZE, 1, 0, 0); //2nd line
    pos += (LAYER_5_W+2)*2; 
  }

  //loop to receive intermediate layer 5 data
  for(i = 0; i < LAYER_5_NUM_KER; i++) {
    rcv_frame(pos, NUM_INTERM_LAYER5_FRAMES, INTERM_LAYER5_SIZE, 1, 0, 0); //1st line
    pos += LAYER_5_W*(1+LAYER_5_H);
    rcv_frame(pos, NUM_INTERM_LAYER5_FRAMES, INTERM_LAYER5_SIZE, 1, 0, 0); //2nd line
    pos += LAYER_5_W;
  }

  //loop to receive intermediate layer 6 data
  for(i = 0; i < LAYER_6_NUM_KER; i++) {
    rcv_frame(pos, NUM_INTERM_LAYER6_FRAMES, INTERM_LAYER6_SIZE, 1, 0, 0); //1st line
    pos += (LAYER_7_W+2)*LAYER_7_H;
    rcv_frame(pos, NUM_INTERM_LAYER6_FRAMES, INTERM_LAYER6_SIZE, 1, 0, 0); //2nd line
    pos += (LAYER_7_W+2)*2;
  }

  //loop to receive intermediate layer 8 data
  pos += DATA_LAYER_7;
  for(i = 0; i < LAYER_8_NUM_KER; i++) {
    rcv_frame(pos, NUM_INTERM_LAYER8_FRAMES, INTERM_LAYER8_SIZE, 1, 0, 0); //1st line
    pos += (LAYER_9_W+2)*LAYER_9_H;
    rcv_frame(pos, NUM_INTERM_LAYER8_FRAMES, INTERM_LAYER8_SIZE, 1, 0, 0); //2nd line
    pos += (LAYER_9_W+2)*2;
  }

  //loop to receive intermediate layer 9 data
  pos += DATA_LAYER_10 + DATA_LAYER_11 + DATA_LAYER_12 + DATA_LAYER_13 + DATA_LAYER_14 + DATA_LAYER_15 + DATA_LAYER_16 + DATA_LAYER_17 + DATA_LAYER_19 + DATA_LAYER_20;
  for(i = 0; i < LAYER_9_NUM_KER; i++) {
    pos += LAYER_9_W+2;
    rcv_frame(pos, NUM_INTERM_LAYER9_FRAMES, INTERM_LAYER9_SIZE, 1, 0, 0); //1st line
    pos += (LAYER_9_W+2)*(1+LAYER_9_H);
    rcv_frame(pos, NUM_INTERM_LAYER9_FRAMES, INTERM_LAYER9_SIZE, 1, 0, 0); //2nd line
    pos += (LAYER_9_W+2)*2;
  }

  //measure transference time
  end = timer_get_count_us(TIMER);
  uart_printf("intermediate data transferred in %d ms\n", (end-start)/1000);
#endif
}

//send detection results back
void send_data() {

  //Loop to send output of yolo layer
  int i, j;
  count_bytes = 0;
  for(j = 0; j < NUM_INPUT_FRAMES+1; j++) {

     // start timer
     if(j == 0) start = timer_get_count_us(TIMER);

     //check if it is last packet (has less data that full payload size)
     if(j == NUM_INPUT_FRAMES) bytes_to_send = INPUT_FILE_SIZE - count_bytes;
     else bytes_to_send = ETH_NBYTES;

     //prepare variable to be sent
     for(i = 0; i < bytes_to_send; i++) data_to_send[i] = fp_image[j*ETH_NBYTES + i];

     //send frame
     eth_send_frame(data_to_send, ETH_NBYTES);

     //wait to receive frame as ack
     if(j != NUM_INPUT_FRAMES) while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout) !=0);

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
  uart_printf("\nYOLO SW FULL\n\n");
  uart_txwait();

  //init ETHERNET
  eth_init(ETHERNET);
  eth_set_rx_payload_size(ETH_NBYTES);

  //define memory regions
  define_memory_regions();
  unsigned int total_time;

  //load data and reset DDR to zero
#ifndef SIM
  reset_DDR();
  receive_data();
#endif

  //resize input image to 418x316x3
  fill_grey();
  uart_printf("\nResizing input image to 416x416\n");
  start = timer_get_count_us(TIMER);
  resize_image();
  end = timer_get_count_us(TIMER);
  uart_printf("Resize image done in %d ms\n", (end-start)/1000);
  total_time = (end-start)/1000;
  print_cache_status();

  //layer1 (418x316x3 -> 416x316x16)
  start = timer_get_count_us(TIMER);
  conv_layer(NTW_IN_W, NTW_IN_H, NTW_IN_C, NTW_IN_NUM_KER, NTW_IN_KER_SIZE, NTW_IN_PAD, NTW_IN_BATCH_NORM, NTW_IN_NEXT_PADD, NTW_IN_NEXT_STRIDE, NTW_IN_IGNORE_PADD, 0, NTW_IN_OFFSET);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer1 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //layer2 (416x316x16 -> 210x162x16)
  start = timer_get_count_us(TIMER);
  maxpool_layer(LAYER_2_W, LAYER_2_H, LAYER_2_NUM_KER, LAYER_2_DOWNSAMPLE, LAYER_2_IGNORE_PADD, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer2 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //layer3 (210x162x16 -> 208x160x32)
  start = timer_get_count_us(TIMER);
  conv_layer(LAYER_3_W, LAYER_3_H, LAYER_3_C, LAYER_3_NUM_KER, LAYER_3_KER_SIZE, LAYER_3_PAD, LAYER_3_BATCH_NORM, LAYER_3_NEXT_PADD, LAYER_3_NEXT_STRIDE, LAYER_3_IGNORE_PADD, 0, LAYER_3_OFFSET);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer3 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //layer4 (208x160x32 -> 106x84x32)
  start = timer_get_count_us(TIMER);
  maxpool_layer(LAYER_4_W, LAYER_4_H, LAYER_4_NUM_KER, LAYER_4_DOWNSAMPLE, LAYER_4_IGNORE_PADD, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer4 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //layer5 (106x84x32 -> 104x84x64)
  start = timer_get_count_us(TIMER);
  conv_layer(LAYER_5_W, LAYER_5_H, LAYER_5_C, LAYER_5_NUM_KER, LAYER_5_KER_SIZE, LAYER_5_PAD, LAYER_5_BATCH_NORM, LAYER_5_NEXT_PADD, LAYER_5_NEXT_STRIDE, LAYER_5_IGNORE_PADD, 0, LAYER_5_OFFSET);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer5 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //layer6 (104x84x64 -> 54x46x64)
  start = timer_get_count_us(TIMER);
  maxpool_layer(LAYER_6_W, LAYER_6_H, LAYER_6_NUM_KER, LAYER_6_DOWNSAMPLE, LAYER_6_IGNORE_PADD, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer6 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //layer7 (54x46x64 -> 52x44x128)
  start = timer_get_count_us(TIMER);
  conv_layer(LAYER_7_W, LAYER_7_H, LAYER_7_C, LAYER_7_NUM_KER, LAYER_7_KER_SIZE, LAYER_7_PAD, LAYER_7_BATCH_NORM, LAYER_7_NEXT_PADD, LAYER_7_NEXT_STRIDE, LAYER_7_IGNORE_PADD, 0, LAYER_7_OFFSET);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer7 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //layer8 (52x44x128 -> 28x26x128)
  start = timer_get_count_us(TIMER);
  maxpool_layer(LAYER_8_W, LAYER_8_H, LAYER_8_NUM_KER, LAYER_8_DOWNSAMPLE, LAYER_8_IGNORE_PADD, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer8 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //Initial address of layer 10 output
  unsigned int data_pos_layer8 = data_pos + DATA_LAYER_8;

  //layer9 (28x26x128 -> 28x28x256) -> Zero-padding
  //Result of layer 9 goes after result of layer 20
  start = timer_get_count_us(TIMER);
  conv_layer(LAYER_9_W, LAYER_9_H, LAYER_9_C, LAYER_9_NUM_KER, LAYER_9_KER_SIZE, LAYER_9_PAD, LAYER_9_BATCH_NORM, LAYER_9_NEXT_PADD, LAYER_9_NEXT_STRIDE, LAYER_9_IGNORE_PADD, data_pos + DATA_LAYER_8 + DATA_LAYER_10 + DATA_LAYER_11 + DATA_LAYER_12 + DATA_LAYER_13 + DATA_LAYER_14 + DATA_LAYER_15 + DATA_LAYER_16 + DATA_LAYER_17 + DATA_LAYER_19 + DATA_LAYER_20, LAYER_9_OFFSET);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer9 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //layer10 (28x28x256 -> 15x15x256) -> Ignores padding from layer 9
  start = timer_get_count_us(TIMER);
  maxpool_layer(LAYER_10_W, LAYER_10_W, LAYER_10_NUM_KER, LAYER_10_DOWNSAMPLE, LAYER_10_IGNORE_PADD, data_pos_layer8);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer10 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //layer11 (15x15x256 -> 14x14x512)
  //Repeats last line and column of each feature map
  start = timer_get_count_us(TIMER);
  conv_layer(LAYER_11_W, LAYER_11_W, LAYER_11_C, LAYER_11_NUM_KER, LAYER_11_KER_SIZE, LAYER_11_PAD, LAYER_11_BATCH_NORM, LAYER_11_NEXT_PADD, LAYER_11_NEXT_STRIDE, LAYER_11_IGNORE_PADD, 0, LAYER_11_OFFSET);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer11 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //layer12 (14x14x512 -> 15x15x512)
  start = timer_get_count_us(TIMER);
  maxpool_layer(LAYER_12_W, LAYER_12_W, LAYER_12_NUM_KER, LAYER_12_DOWNSAMPLE, LAYER_12_IGNORE_PADD, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer12 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //layer13 (15x15x512 -> 13x13x1024)
  start = timer_get_count_us(TIMER);
  conv_layer(LAYER_13_W, LAYER_13_W, LAYER_13_C, LAYER_13_NUM_KER, LAYER_13_KER_SIZE, LAYER_13_PAD, LAYER_13_BATCH_NORM, LAYER_13_NEXT_PADD, LAYER_13_NEXT_STRIDE, LAYER_13_IGNORE_PADD, 0, LAYER_13_OFFSET);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer13 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //layer14 (13x13x1024 -> 15x15x256)
  start = timer_get_count_us(TIMER);
  conv_layer(LAYER_14_W, LAYER_14_W, LAYER_14_C, LAYER_14_NUM_KER, LAYER_14_KER_SIZE, LAYER_14_PAD, LAYER_14_BATCH_NORM, LAYER_14_NEXT_PADD, LAYER_14_NEXT_STRIDE, LAYER_14_IGNORE_PADD, 0, LAYER_14_OFFSET);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer14 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //Stores initial address of layer 14 output for first route layer
  unsigned int data_pos_layer14 = data_pos;

  //layer15 (15x15x256 -> 13x13x512)
  start = timer_get_count_us(TIMER);
  conv_layer(LAYER_15_W, LAYER_15_W, LAYER_15_C, LAYER_15_NUM_KER, LAYER_15_KER_SIZE, LAYER_15_PAD, LAYER_15_BATCH_NORM, LAYER_15_NEXT_PADD, LAYER_15_NEXT_STRIDE, LAYER_15_IGNORE_PADD, 0, LAYER_15_OFFSET);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer15 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //layer16 (13x13x512 -> 13x13x255)
  start = timer_get_count_us(TIMER);
  conv_layer(LAYER_16_W, LAYER_16_W, LAYER_16_C, LAYER_16_NUM_KER, LAYER_16_KER_SIZE, LAYER_16_PAD, LAYER_16_BATCH_NORM, LAYER_16_NEXT_PADD, LAYER_16_NEXT_STRIDE, LAYER_16_IGNORE_PADD, 0, LAYER_16_OFFSET);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer16 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //layer17 (13x13x255 -> 13x13x255)
  start = timer_get_count_us(TIMER);
  yolo_layer(LAYER_17_W);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer17 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //Stores initial address of first yolo layer for sending
  unsigned int data_pos_layer17 = data_pos;

  //Stores initial address of the output of layer 19
  unsigned int previous_data_pos = data_pos + DATA_LAYER_17;

  //layer18 (points to the initial address of layer 14 output)
  data_pos = data_pos_layer14;

  //layer19 (15x15x256 -> 13x13x128)
  start = timer_get_count_us(TIMER);
  conv_layer(LAYER_19_W, LAYER_19_W, LAYER_19_C, LAYER_19_NUM_KER, LAYER_19_KER_SIZE, LAYER_19_PAD, LAYER_19_BATCH_NORM, LAYER_19_NEXT_PADD, LAYER_19_NEXT_STRIDE, LAYER_19_IGNORE_PADD, previous_data_pos, LAYER_19_OFFSET);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer19 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //layer20 (13x13x128 -> 28x28x128)
  start = timer_get_count_us(TIMER);
  upsample_layer(LAYER_20_W, LAYER_20_NUM_KER);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer20 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //layer22 (28x28x128 -> 26x26x256)
  //layer 21 (second route layer) is not needed as output of layer 9 is already after output of layer 20
  start = timer_get_count_us(TIMER);
  conv_layer(LAYER_22_W, LAYER_22_W, LAYER_22_C, LAYER_22_NUM_KER, LAYER_22_KER_SIZE, LAYER_22_PAD, LAYER_22_BATCH_NORM, LAYER_22_NEXT_PADD, LAYER_22_NEXT_STRIDE, LAYER_22_IGNORE_PADD, data_pos + DATA_LAYER_20 + DATA_LAYER_9, LAYER_22_OFFSET);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer22 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //layer23 (26x26x256 -> 26x26x255)
  start = timer_get_count_us(TIMER);
  conv_layer(LAYER_23_W, LAYER_23_W, LAYER_23_C, LAYER_23_NUM_KER, LAYER_23_KER_SIZE, LAYER_23_PAD, LAYER_23_BATCH_NORM, LAYER_23_NEXT_PADD, LAYER_23_NEXT_STRIDE, LAYER_23_IGNORE_PADD, 0, LAYER_23_OFFSET);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer23 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //layer24 (26x26x255 -> 26x26x255)
  start = timer_get_count_us(TIMER);
  yolo_layer(LAYER_24_W);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer24 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();

  //get candidate boxes
  uart_printf("\nGetting candidate boxes...\n");
  unsigned int box_pos = data_pos+DATA_LAYER_24;
  start = timer_get_count_us(TIMER);
  get_boxes(LAYER_17_W, yolo1_div, 1, data_pos_layer17, box_pos);
  get_boxes(LAYER_24_W, yolo2_div, 0, data_pos, box_pos);
  filter_boxes(box_pos);
  end = timer_get_count_us(TIMER);
  uart_printf("Candidate boxes selected in %d us\n", (end-start));
  total_time += (end-start)/1000;
  print_cache_status();

  //draw detections
  uart_printf("\nDrawing detections...\n");
  start = timer_get_count_us(TIMER);
  draw_detections(box_pos);
  end = timer_get_count_us(TIMER);
  uart_printf("Detection drawn in %d us\n", (end-start));
  total_time += (end-start)/1000;
  print_cache_status();

  //print results
  print_results(box_pos);

  //return data
  uart_printf("\ntotal_time = %d seconds (%d minutes) \n", total_time/1000, (total_time/1000)/60);
  send_data();
  uart_putc(4);
  return 0;
}
