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

//define constants
#define ETH_NBYTES (1024-18) //minimum ethernet payload excluding FCS
#define INPUT_FILE_SIZE (418*418*3*2) //16 bits per point
#define WEIGHTS_FILE_SIZE (17704732) //16 bits per input
#define NUM_INPUT_FRAMES (INPUT_FILE_SIZE/ETH_NBYTES)
#define NUM_WEIGHT_FRAMES (WEIGHTS_FILE_SIZE/ETH_NBYTES)
#define WEIGTHS_BASE_ADDRESS (DDR_MEM + 0x00008000) //16kb for program + 16kb for stack
#define DATA_BASE_ADDRESS (DDR_MEM + 0x01408000)

//weights and data base address pointers
int16_t *fp_weights;
int16_t *fp_data;

//weights and data updatable pointers
unsigned int weight_pos = 0, data_pos = 0;

//ETHERNET variables
int rcv_timeout = 5000;
char data_to_send[ETH_NBYTES];
char data_rcv[ETH_NBYTES+18];
unsigned int bytes_to_receive, bytes_to_send, count_bytes;

//TIMER variables
unsigned int start, end;

//define base address of weights and data pointers
void define_memory_regions() {

  //data
  fp_data = (int16_t *) DATA_BASE_ADDRESS;

  //weights
  fp_weights = (int16_t *) WEIGTHS_BASE_ADDRESS;
}

void print_cache_status() {
  //uart_printf("ctrl_instr_hit = %d\n", ctrl_instr_hit(CACHE_CTRL));
  //uart_printf("ctrl_instr_miss = %d\n", ctrl_instr_miss(CACHE_CTRL));
  uart_printf("ctrl_data_read_hit = %d\n", ctrl_data_read_hit(CACHE_CTRL));
  uart_printf("ctrl_data_read_miss = %d\n", ctrl_data_read_miss(CACHE_CTRL));
  uart_printf("ctrl_data_write_hit = %d\n", ctrl_data_write_hit(CACHE_CTRL));
  uart_printf("ctrl_data_write_miss = %d\n\n", ctrl_data_write_miss(CACHE_CTRL));
}

//receive weights and data
void receive_data() {

  uart_printf("\nReady to receive input.network and weights\n");

  //char file pointers
  char * fp_data_char = (char *) DATA_BASE_ADDRESS;
  char * fp_weights_char = (char *) WEIGTHS_BASE_ADDRESS;
  int i, j;

  //Loop to receive input.network frames
  for(j = 0; j < NUM_INPUT_FRAMES+1; j++) {

     //wait to receive frame
     while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout) !=0);

     // start timer
     if(j == 0) {
       count_bytes = 0;
       timer_reset(TIMER);
       start = timer_get_count_us(TIMER);
     }

     //check if it is last packet (has less data that full payload size)
     if(j == NUM_INPUT_FRAMES) bytes_to_receive = INPUT_FILE_SIZE - count_bytes;
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

  //measure transference time
  end = timer_get_count_us(TIMER);
  uart_printf("input.network transferred in %d ms\n", (end-start)/1000);

  //Loop to receive weight frames
  for(j = 0; j < NUM_WEIGHT_FRAMES+1; j++) {

     //wait to receive frame
     while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout) !=0);

     // start timer
     if(j == 0) {
       count_bytes = 0;
       timer_reset(TIMER);
       start = timer_get_count_us(TIMER);
     }

     //check if it is last packet (has less data that full payload size)
     if(j == NUM_WEIGHT_FRAMES) bytes_to_receive = WEIGHTS_FILE_SIZE - count_bytes;
     else bytes_to_receive = ETH_NBYTES;

     //save in DDR
     for(i = 0; i < bytes_to_receive; i++) {
       fp_weights_char[j*ETH_NBYTES + i] = data_rcv[14+i];
       data_to_send[i] = data_rcv[14+i];
     }

     //send data back as ack
     eth_send_frame(data_to_send, ETH_NBYTES);

     //update byte counter
     count_bytes += ETH_NBYTES;
  }

  //measure transference time
  end = timer_get_count_us(TIMER);
  uart_printf("weights transferred in %d ms\n", (end-start)/1000);
}

//perform convolutional layer
void conv_layer_unroll_4_2(int w, int c, int num_ker, int ker_size, int pad, int batch_norm, int nextPadding, int nextStride, int ignorePadding, unsigned int new_output_pos) {

  //locate weight and data pointers
  unsigned int pos_delta = (w+2*pad)*(w+2*pad)*c;
  int16_t * w_pos;
  w_pos = (int16_t *) fp_weights + weight_pos;
  int16_t * bias_pos = (int16_t *) fp_weights + weight_pos + num_ker*ker_size*ker_size*c;
  int16_t * in_d_pos = (int16_t *) fp_data + data_pos;
  int16_t * out_d_pos;
  if(new_output_pos != 0) out_d_pos = (int16_t *) fp_data + new_output_pos; else out_d_pos = (int16_t *) in_d_pos + pos_delta;

  //local variables
  int i, j, k, l, m, n, new_w;
  unsigned int output_pos, output_pos2, output_pos3, output_pos4;
  unsigned int output_pos5, output_pos6, output_pos7, output_pos8;
  if(nextStride) new_w = w+1; else if(nextPadding) new_w = w+2; else new_w = w;
  int16_t op1, op2, op2_2, op2_3, op2_4, output_conv, mul_16;
  int16_t leaky = 3276; //0.1 in Q1.15;
  int32_t acc, acc2, acc3, acc4, acc5, acc6, acc7, acc8; 
  int32_t acc_final, acc_final2, acc_final3, acc_final4, acc_final5, acc_final6, acc_final7, acc_final8; 
  int32_t mul;
  int16_t shared_bias, shared_scales;

  //perform convolution
  for(i = 0; i < num_ker; i+=4) {               //Number of kernels
    for(j = 0; j < w; j+=2) {  	        	//Output map size
      for(k = 0; k < w; k++) {
	output_pos = i*new_w*new_w + j*new_w + k;
	output_pos2 = (i+1)*new_w*new_w + j*new_w + k;
	output_pos3 = (i+2)*new_w*new_w + j*new_w + k;
	output_pos4 = (i+3)*new_w*new_w + j*new_w + k;
	output_pos5 = i*new_w*new_w + (j+1)*new_w + k;
	output_pos6 = (i+1)*new_w*new_w + (j+1)*new_w + k;
	output_pos7 = (i+2)*new_w*new_w + (j+1)*new_w + k;
	output_pos8 = (i+3)*new_w*new_w + (j+1)*new_w + k;
	acc_final = 0;
	acc_final2 = 0;
	acc_final3 = 0;
	acc_final4 = 0;
	acc_final5 = 0;
	acc_final6 = 0;
	acc_final7 = 0;
	acc_final8 = 0;
	for(l = 0; l < c; l++) { 		//Number of channels
 	  acc = 0;
 	  acc2 = 0;
 	  acc3 = 0;
 	  acc4 = 0;
 	  acc5 = 0;
 	  acc6 = 0;
 	  acc7 = 0;
 	  acc8 = 0;
	  for(m = 0; m < ker_size; m++) {	//Kernel size
	    for(n = 0; n < ker_size; n++) {
	      op1 = in_d_pos[(j+ignorePadding)*(w+2*pad) + (k+ignorePadding) + l*(w+2*pad)*(w+2*pad) + m*(w+2*pad) + n]; //Q8.8
	      op2 = w_pos[i*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul = (int32_t)((int32_t)op1*(int32_t)op2); //Q12.20
	      acc += mul; //Q12.20
	      op2_2 = w_pos[(i+1)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul = (int32_t)((int32_t)op1*(int32_t)op2_2); //Q12.20
	      acc2 += mul; //Q12.20
	      op2_3 = w_pos[(i+2)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul = (int32_t)((int32_t)op1*(int32_t)op2_3); //Q12.20
	      acc3 += mul; //Q12.20
	      op2_4 = w_pos[(i+3)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul = (int32_t)((int32_t)op1*(int32_t)op2_4); //Q12.20
	      acc4 += mul; //Q12.20
	      op1 = in_d_pos[(j+1+ignorePadding)*(w+2*pad) + (k+ignorePadding) + l*(w+2*pad)*(w+2*pad) + m*(w+2*pad) + n]; //Q8.8
	      mul = (int32_t)((int32_t)op1*(int32_t)op2); //Q12.20
	      acc5 += mul; //Q12.20
	      mul = (int32_t)((int32_t)op1*(int32_t)op2_2); //Q12.20
	      acc6 += mul; //Q12.20
	      mul = (int32_t)((int32_t)op1*(int32_t)op2_3); //Q12.20
	      acc7 += mul; //Q12.20
	      mul = (int32_t)((int32_t)op1*(int32_t)op2_4); //Q12.20
	      acc8 += mul; //Q12.20
	    }
	  }
	  acc_final += acc; //Q12.20
	  acc_final2 += acc2; //Q12.20
	  acc_final3 += acc3; //Q12.20
	  acc_final4 += acc4; //Q12.20
	  acc_final5 += acc5; //Q12.20
	  acc_final6 += acc6; //Q12.20
	  acc_final7 += acc7; //Q12.20
	  acc_final8 += acc8; //Q12.20
	}

	//perform batch normalize and leaky activation
	shared_scales = bias_pos[i]; 
	shared_bias = bias_pos[num_ker+i];
	output_conv = (int16_t) ((int32_t) (acc_final << 3) >> 16);//Q12.20 to Q9.7
  	mul = (int32_t) output_conv * (int32_t) shared_scales;//Q9.7 * Q8.8 = Q17.15
	mul_16 = (int16_t) ((int32_t) (mul << 9) >> 16);//Q17.15 to Q8.8
	mul_16 += shared_bias; //Q8.8
	if(mul_16 < 0) {
	  mul = (int32_t) mul_16 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	  mul_16 = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
	}
	out_d_pos[output_pos] = mul_16;
	output_conv = (int16_t) ((int32_t) (acc_final5 << 3) >> 16);//Q12.20 to Q9.7
  	mul = (int32_t) output_conv * (int32_t) shared_scales;//Q9.7 * Q8.8 = Q17.15
	mul_16 = (int16_t) ((int32_t) (mul << 9) >> 16);//Q17.15 to Q8.8
	mul_16 += shared_bias; //Q8.8
	if(mul_16 < 0) {
	  mul = (int32_t) mul_16 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	  mul_16 = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
	}
	out_d_pos[output_pos5] = mul_16;

	shared_scales = bias_pos[i+1]; 
	shared_bias = bias_pos[num_ker+i+1];
	output_conv = (int16_t) ((int32_t) (acc_final2 << 3) >> 16);//Q12.20 to Q9.7
  	mul = (int32_t) output_conv * (int32_t) shared_scales;//Q9.7 * Q8.8 = Q17.15
	mul_16 = (int16_t) ((int32_t) (mul << 9) >> 16);//Q17.15 to Q8.8
	mul_16 += shared_bias; //Q8.8
	if(mul_16 < 0) {
	  mul = (int32_t) mul_16 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	  mul_16 = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
	}
	out_d_pos[output_pos2] = mul_16;
	output_conv = (int16_t) ((int32_t) (acc_final6 << 3) >> 16);//Q12.20 to Q9.7
  	mul = (int32_t) output_conv * (int32_t) shared_scales;//Q9.7 * Q8.8 = Q17.15
	mul_16 = (int16_t) ((int32_t) (mul << 9) >> 16);//Q17.15 to Q8.8
	mul_16 += shared_bias; //Q8.8
	if(mul_16 < 0) {
	  mul = (int32_t) mul_16 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	  mul_16 = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
	}
	out_d_pos[output_pos6] = mul_16;

	shared_scales = bias_pos[i+2]; 
	shared_bias = bias_pos[num_ker+i+2];
	output_conv = (int16_t) ((int32_t) (acc_final3 << 3) >> 16);//Q12.20 to Q9.7
  	mul = (int32_t) output_conv * (int32_t) shared_scales;//Q9.7 * Q8.8 = Q17.15
	mul_16 = (int16_t) ((int32_t) (mul << 9) >> 16);//Q17.15 to Q8.8
	mul_16 += shared_bias; //Q8.8
	if(mul_16 < 0) {
	  mul = (int32_t) mul_16 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	  mul_16 = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
	}
	out_d_pos[output_pos3] = mul_16;
	output_conv = (int16_t) ((int32_t) (acc_final7 << 3) >> 16);//Q12.20 to Q9.7
  	mul = (int32_t) output_conv * (int32_t) shared_scales;//Q9.7 * Q8.8 = Q17.15
	mul_16 = (int16_t) ((int32_t) (mul << 9) >> 16);//Q17.15 to Q8.8
	mul_16 += shared_bias; //Q8.8
	if(mul_16 < 0) {
	  mul = (int32_t) mul_16 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	  mul_16 = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
	}
	out_d_pos[output_pos7] = mul_16;

	shared_scales = bias_pos[i+3]; 
	shared_bias = bias_pos[num_ker+i+3];
	output_conv = (int16_t) ((int32_t) (acc_final4 << 3) >> 16);//Q12.20 to Q9.7
  	mul = (int32_t) output_conv * (int32_t) shared_scales;//Q9.7 * Q8.8 = Q17.15
	mul_16 = (int16_t) ((int32_t) (mul << 9) >> 16);//Q17.15 to Q8.8
	mul_16 += shared_bias; //Q8.8
	if(mul_16 < 0) {
	  mul = (int32_t) mul_16 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	  mul_16 = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
	}
	out_d_pos[output_pos4] = mul_16;
	output_conv = (int16_t) ((int32_t) (acc_final8 << 3) >> 16);//Q12.20 to Q9.7
  	mul = (int32_t) output_conv * (int32_t) shared_scales;//Q9.7 * Q8.8 = Q17.15
	mul_16 = (int16_t) ((int32_t) (mul << 9) >> 16);//Q17.15 to Q8.8
	mul_16 += shared_bias; //Q8.8
	if(mul_16 < 0) {
	  mul = (int32_t) mul_16 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	  mul_16 = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
	}
	out_d_pos[output_pos8] = mul_16;
      }
    }
  }

  //update weights and data pointers
  if(batch_norm) weight_pos += num_ker*2 + num_ker*c*ker_size*ker_size;
  else weight_pos += num_ker + num_ker*c*ker_size*ker_size;
  if(new_output_pos != 0) data_pos = new_output_pos; else data_pos += pos_delta;
}

//perform convolutional layer
void conv_layer_unroll_4_0_0_2(int w, int c, int num_ker, int ker_size, int pad, int batch_norm, int nextPadding, int nextStride, int ignorePadding, unsigned int new_output_pos) {

  //locate weight and data pointers
  unsigned int pos_delta = (w+2*pad)*(w+2*pad)*c;
  int16_t * w_pos;
  w_pos = (int16_t *) fp_weights + weight_pos;
  int16_t * bias_pos = (int16_t *) fp_weights + weight_pos + num_ker*ker_size*ker_size*c;
  int16_t * in_d_pos = (int16_t *) fp_data + data_pos;
  int16_t * out_d_pos;
  if(new_output_pos != 0) out_d_pos = (int16_t *) fp_data + new_output_pos; else out_d_pos = (int16_t *) in_d_pos + pos_delta;

  //local variables
  int i, j, k, l, m, n, new_w;
  unsigned int output_pos, output_pos2, output_pos3, output_pos4;
  if(nextStride) new_w = w+1; else if(nextPadding) new_w = w+2; else new_w = w;
  int16_t op1, op2, output_conv; 
  int16_t mul_16, mul_16_2, mul_16_3, mul_16_4;
  int16_t leaky = 3276; //0.1 in Q1.15;
  int32_t acc, acc2, acc3, acc4; 
  int32_t acc_final, acc_final2, acc_final3, acc_final4; 
  int32_t mul;

  //perform convolution
  for(i = 0; i < num_ker; i+=4) {               //Number of kernels
    for(j = 0; j < w; j++) {   	        	//Output map size
      for(k = 0; k < w; k++) {
        if(nextPadding) {
	  output_pos = i*new_w*new_w + (j+1)*new_w + (k+1);
	  output_pos2 = (i+1)*new_w*new_w + (j+1)*new_w + (k+1);
	  output_pos3 = (i+2)*new_w*new_w + (j+1)*new_w + (k+1);
	  output_pos4 = (i+3)*new_w*new_w + (j+1)*new_w + (k+1);
	} else { 
	  output_pos = i*new_w*new_w + j*new_w + k;
	  output_pos2 = (i+1)*new_w*new_w + j*new_w + k;
	  output_pos3 = (i+2)*new_w*new_w + j*new_w + k;
	  output_pos4 = (i+3)*new_w*new_w + j*new_w + k;
	}
	acc_final = 0;
	acc_final2 = 0;
	acc_final3 = 0;
	acc_final4 = 0;
	for(l = 0; l < c; l+=2) { 		//Number of channels
 	  acc = 0; 
 	  acc2 = 0; 
 	  acc3 = 0;
 	  acc4 = 0;
	  for(m = 0; m < ker_size; m++) {	//Kernel size
	    for(n = 0; n < ker_size; n++) {
	      op1 = in_d_pos[(j+ignorePadding)*(w+2*pad) + (k+ignorePadding) + l*(w+2*pad)*(w+2*pad) + m*(w+2*pad) + n]; //Q8.8
	      op2 = w_pos[i*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul = (int32_t)((int32_t)op1*(int32_t)op2); //Q12.20
	      acc += mul; //Q12.20
	      op2 = w_pos[(i+1)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul = (int32_t)((int32_t)op1*(int32_t)op2); //Q12.20
	      acc2 += mul; //Q12.20
	      op2 = w_pos[(i+2)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul = (int32_t)((int32_t)op1*(int32_t)op2); //Q12.20
	      acc3 += mul; //Q12.20
	      op2 = w_pos[(i+3)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul = (int32_t)((int32_t)op1*(int32_t)op2); //Q12.20
	      acc4 += mul; //Q12.20

  	      op1 = in_d_pos[(j+ignorePadding)*(w+2*pad) + (k+ignorePadding) + (l+1)*(w+2*pad)*(w+2*pad) + m*(w+2*pad) + n]; //Q8.8
	      op2 = w_pos[i*c*ker_size*ker_size + (l+1)*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul = (int32_t)((int32_t)op1*(int32_t)op2); //Q12.20
	      acc += mul; //Q12.20
	      op2 = w_pos[(i+1)*c*ker_size*ker_size + (l+1)*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul = (int32_t)((int32_t)op1*(int32_t)op2); //Q12.20
	      acc2 += mul;
	      op2 = w_pos[(i+2)*c*ker_size*ker_size + (l+1)*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul = (int32_t)((int32_t)op1*(int32_t)op2); //Q12.20
	      acc3 += mul;
	      op2 = w_pos[(i+3)*c*ker_size*ker_size + (l+1)*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul = (int32_t)((int32_t)op1*(int32_t)op2); //Q12.20
	      acc4 += mul;
	    }
	  }
	  acc_final += acc; //Q12.20
	  acc_final2 += acc2; //Q12.20
	  acc_final3 += acc3; //Q12.20
	  acc_final4 += acc4; //Q12.20
	}

	//perform batch normalize
	if(batch_norm) {
	  output_conv = (int16_t) ((int32_t) (acc_final << 3) >> 16);//Q12.20 to Q9.7
  	  mul = (int32_t) output_conv * (int32_t) bias_pos[i];//Q9.7 * Q8.8 = Q17.15
	  mul_16 = (int16_t) ((int32_t) (mul << 9) >> 16);//Q17.15 to Q8.8
	  mul_16 += bias_pos[num_ker+i]; //Q8.8
	  output_conv = (int16_t) ((int32_t) (acc_final2 << 3) >> 16);//Q12.20 to Q9.7
  	  mul = (int32_t) output_conv * (int32_t) bias_pos[i+1];//Q9.7 * Q8.8 = Q17.15
	  mul_16_2 = (int16_t) ((int32_t) (mul << 9) >> 16);//Q17.15 to Q8.8
	  mul_16_2 += bias_pos[num_ker+i+1]; //Q8.8
	  output_conv = (int16_t) ((int32_t) (acc_final3 << 3) >> 16);//Q12.20 to Q9.7
  	  mul = (int32_t) output_conv * (int32_t) bias_pos[i+2];//Q9.7 * Q8.8 = Q17.15
	  mul_16_3 = (int16_t) ((int32_t) (mul << 9) >> 16);//Q17.15 to Q8.8
	  mul_16_3 += bias_pos[num_ker+i+2]; //Q8.8
	  output_conv = (int16_t) ((int32_t) (acc_final4 << 3) >> 16);//Q12.20 to Q9.7
  	  mul = (int32_t) output_conv * (int32_t) bias_pos[i+3];//Q9.7 * Q8.8 = Q17.15
	  mul_16_4 = (int16_t) ((int32_t) (mul << 9) >> 16);//Q17.15 to Q8.8
	  mul_16_4 += bias_pos[num_ker+i+3]; //Q8.8

	  //perform leaky activation
	  if(mul_16 < 0) {
	    mul = (int32_t) mul_16 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	    mul_16 = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
	  }
	  out_d_pos[output_pos] = mul_16;
	  if(mul_16_2 < 0) {
	    mul = (int32_t) mul_16_2 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	    mul_16_2 = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
	  }
	  out_d_pos[output_pos2] = mul_16_2;
	  if(mul_16_3 < 0) {
	    mul = (int32_t) mul_16_3 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	    mul_16_3 = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
	  }
	  out_d_pos[output_pos3] = mul_16_3;
	  if(mul_16_4 < 0) {
	    mul = (int32_t) mul_16_4 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	    mul_16_4 = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
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
	  output_conv = (int16_t) ((int32_t) (acc_final2 << 4) >> 16);//Q12.20 to Q8.8
	  out_d_pos[output_pos2] = output_conv + bias_pos[i+1];
	  output_conv = (int16_t) ((int32_t) (acc_final3 << 4) >> 16);//Q12.20 to Q8.8
	  out_d_pos[output_pos3] = output_conv + bias_pos[i+2];
	  output_conv = (int16_t) ((int32_t) (acc_final4 << 4) >> 16);//Q12.20 to Q8.8
	  out_d_pos[output_pos4] = output_conv + bias_pos[i+3];
	}
      }
    }
  }

  //update weights and data pointers
  if(batch_norm) weight_pos += num_ker*2 + num_ker*c*ker_size*ker_size;
  else weight_pos += num_ker + num_ker*c*ker_size*ker_size;
  if(new_output_pos != 0) data_pos = new_output_pos; else data_pos += pos_delta;
}

//perform convolutional layer
void conv_layer_unroll_4_2_0_2(int w, int c, int num_ker, int ker_size, int pad, int batch_norm, int nextPadding, int nextStride, int ignorePadding, unsigned int new_output_pos) {

  //locate weight and data pointers
  unsigned int pos_delta = (w+2*pad)*(w+2*pad)*c;
  int16_t * w_pos;
  w_pos = (int16_t *) fp_weights + weight_pos;
  int16_t * bias_pos = (int16_t *) fp_weights + weight_pos + num_ker*ker_size*ker_size*c;
  int16_t * in_d_pos = (int16_t *) fp_data + data_pos;
  int16_t * out_d_pos;
  if(new_output_pos != 0) out_d_pos = (int16_t *) fp_data + new_output_pos; else out_d_pos = (int16_t *) in_d_pos + pos_delta;

  //local variables
  int i, j, k, l, m, n, new_w;
  unsigned int output_pos, output_pos2, output_pos3, output_pos4, output_pos5, output_pos6, output_pos7, output_pos8;
  if(nextStride) new_w = w+1; else if(nextPadding) new_w = w+2; else new_w = w;
  int16_t op1, op2, op2_2, op2_3, op2_4, op2_5, op2_6, op2_7, op2_8, output_conv, mul_16;
  int16_t leaky = 3276; //0.1 in Q1.15;
  int32_t acc, acc2, acc3, acc4, acc5, acc6, acc7, acc8; 
  int32_t acc_final, acc_final2, acc_final3, acc_final4, acc_final5, acc_final6, acc_final7, acc_final8; 
  int32_t mul;
  int16_t shared_bias, shared_scales;

  //perform convolution
  for(i = 0; i < num_ker; i+=4) {               //Number of kernels
    for(j = 0; j < w; j+=2) {           	//Output map size
      for(k = 0; k < w; k++) {
        if(nextPadding) {
	  output_pos = i*new_w*new_w + (j+1)*new_w + (k+1);
	  output_pos2 = (i+1)*new_w*new_w + (j+1)*new_w + (k+1);
	  output_pos3 = (i+2)*new_w*new_w + (j+1)*new_w + (k+1);
	  output_pos4 = (i+3)*new_w*new_w + (j+1)*new_w + (k+1);
	  output_pos5 = i*new_w*new_w + (j+2)*new_w + (k+1);
	  output_pos6 = (i+1)*new_w*new_w + (j+2)*new_w + (k+1);
	  output_pos7 = (i+2)*new_w*new_w + (j+2)*new_w + (k+1);
	  output_pos8 = (i+3)*new_w*new_w + (j+2)*new_w + (k+1);
	} else { 
	  output_pos = i*new_w*new_w + j*new_w + k;
	  output_pos2 = (i+1)*new_w*new_w + j*new_w + k;
	  output_pos3 = (i+2)*new_w*new_w + j*new_w + k;
	  output_pos4 = (i+3)*new_w*new_w + j*new_w + k;
	  output_pos5 = i*new_w*new_w + (j+1)*new_w + k;
	  output_pos6 = (i+1)*new_w*new_w + (j+1)*new_w + k;
	  output_pos7 = (i+2)*new_w*new_w + (j+1)*new_w + k;
	  output_pos8 = (i+3)*new_w*new_w + (j+1)*new_w + k;
	}
	acc_final = 0;
	acc_final2 = 0;
	acc_final3 = 0;
	acc_final4 = 0;
	acc_final5 = 0;
	acc_final6 = 0;
	acc_final7 = 0;
	acc_final8 = 0;
	for(l = 0; l < c; l+=2) { 		//Number of channels
 	  acc = 0; 
 	  acc2 = 0; 
 	  acc3 = 0;
 	  acc4 = 0;
 	  acc5 = 0; 
 	  acc6 = 0; 
 	  acc7 = 0;
 	  acc8 = 0;
	  for(m = 0; m < ker_size; m++) {	//Kernel size
	    for(n = 0; n < ker_size; n++) {
	      op1 = in_d_pos[(j+ignorePadding)*(w+2*pad) + (k+ignorePadding) + l*(w+2*pad)*(w+2*pad) + m*(w+2*pad) + n]; //Q8.8
	      op2 = w_pos[i*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul = (int32_t)((int32_t)op1*(int32_t)op2); //Q12.20
	      acc += mul; //Q12.20
	      op2_2 = w_pos[(i+1)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul = (int32_t)((int32_t)op1*(int32_t)op2_2); //Q12.20
	      acc2 += mul; //Q12.20
	      op2_3 = w_pos[(i+2)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul = (int32_t)((int32_t)op1*(int32_t)op2_3); //Q12.20
	      acc3 += mul; //Q12.20
	      op2_4 = w_pos[(i+3)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul = (int32_t)((int32_t)op1*(int32_t)op2_4); //Q12.20
	      acc4 += mul; //Q12.20
  	      op1 = in_d_pos[(j+ignorePadding)*(w+2*pad) + (k+ignorePadding) + (l+1)*(w+2*pad)*(w+2*pad) + m*(w+2*pad) + n]; //Q8.8
	      op2_5 = w_pos[i*c*ker_size*ker_size + (l+1)*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul = (int32_t)((int32_t)op1*(int32_t)op2_5); //Q12.20
	      acc += mul; //Q12.20
	      op2_6 = w_pos[(i+1)*c*ker_size*ker_size + (l+1)*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul = (int32_t)((int32_t)op1*(int32_t)op2_6); //Q12.20
	      acc2 += mul;
	      op2_7 = w_pos[(i+2)*c*ker_size*ker_size + (l+1)*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul = (int32_t)((int32_t)op1*(int32_t)op2_7); //Q12.20
	      acc3 += mul;
	      op2_8 = w_pos[(i+3)*c*ker_size*ker_size + (l+1)*ker_size*ker_size + m*ker_size + n]; //Q4.12
	      mul = (int32_t)((int32_t)op1*(int32_t)op2_8); //Q12.20
	      acc4 += mul;

	      op1 = in_d_pos[(j+1+ignorePadding)*(w+2*pad) + (k+ignorePadding) + l*(w+2*pad)*(w+2*pad) + m*(w+2*pad) + n]; //Q8.8
	      mul = (int32_t)((int32_t)op1*(int32_t)op2); //Q12.20
	      acc5 += mul; //Q12.20
	      mul = (int32_t)((int32_t)op1*(int32_t)op2_2); //Q12.20
	      acc6 += mul; //Q12.20
	      mul = (int32_t)((int32_t)op1*(int32_t)op2_3); //Q12.20
	      acc7 += mul; //Q12.20
	      mul = (int32_t)((int32_t)op1*(int32_t)op2_4); //Q12.20
	      acc8 += mul; //Q12.20
  	      op1 = in_d_pos[(j+1+ignorePadding)*(w+2*pad) + (k+ignorePadding) + (l+1)*(w+2*pad)*(w+2*pad) + m*(w+2*pad) + n]; //Q8.8
	      mul = (int32_t)((int32_t)op1*(int32_t)op2_5); //Q12.20
	      acc5 += mul; //Q12.20
	      mul = (int32_t)((int32_t)op1*(int32_t)op2_6); //Q12.20
	      acc6 += mul;
	      mul = (int32_t)((int32_t)op1*(int32_t)op2_7); //Q12.20
	      acc7 += mul;
	      mul = (int32_t)((int32_t)op1*(int32_t)op2_8); //Q12.20
	      acc8 += mul;
	    }
	  }
	  acc_final += acc; //Q12.20
	  acc_final2 += acc2; //Q12.20
	  acc_final3 += acc3; //Q12.20
	  acc_final4 += acc4; //Q12.20
	  acc_final5 += acc5; //Q12.20
	  acc_final6 += acc6; //Q12.20
	  acc_final7 += acc7; //Q12.20
	  acc_final8 += acc8; //Q12.20
	}

	//perform batch normalize and leaky activation
	if(batch_norm) {

	  shared_scales = bias_pos[i];
 	  shared_bias = bias_pos[num_ker+i];
	  output_conv = (int16_t) ((int32_t) (acc_final << 3) >> 16);//Q12.20 to Q9.7
  	  mul = (int32_t) output_conv * (int32_t) shared_scales;//Q9.7 * Q8.8 = Q17.15
	  mul_16 = (int16_t) ((int32_t) (mul << 9) >> 16);//Q17.15 to Q8.8
	  mul_16 += shared_bias; //Q8.8
	  if(mul_16 < 0) {
	    mul = (int32_t) mul_16 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	    mul_16 = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
	  }
	  out_d_pos[output_pos] = mul_16;
	  output_conv = (int16_t) ((int32_t) (acc_final5 << 3) >> 16);//Q12.20 to Q9.7
  	  mul = (int32_t) output_conv * (int32_t) shared_scales;//Q9.7 * Q8.8 = Q17.15
	  mul_16 = (int16_t) ((int32_t) (mul << 9) >> 16);//Q17.15 to Q8.8
	  mul_16 += shared_bias; //Q8.8
	  if(mul_16 < 0) {
	    mul = (int32_t) mul_16 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	    mul_16 = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
	  }
	  out_d_pos[output_pos5] = mul_16;

	  shared_scales = bias_pos[i+1];
 	  shared_bias = bias_pos[num_ker+i+1];
	  output_conv = (int16_t) ((int32_t) (acc_final2 << 3) >> 16);//Q12.20 to Q9.7
  	  mul = (int32_t) output_conv * (int32_t) shared_scales;//Q9.7 * Q8.8 = Q17.15
	  mul_16 = (int16_t) ((int32_t) (mul << 9) >> 16);//Q17.15 to Q8.8
	  mul_16 += shared_bias; //Q8.8
	  if(mul_16 < 0) {
	    mul = (int32_t) mul_16 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	    mul_16 = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
	  }
	  out_d_pos[output_pos2] = mul_16;
	  output_conv = (int16_t) ((int32_t) (acc_final6 << 3) >> 16);//Q12.20 to Q9.7
  	  mul = (int32_t) output_conv * (int32_t) shared_scales;//Q9.7 * Q8.8 = Q17.15
	  mul_16 = (int16_t) ((int32_t) (mul << 9) >> 16);//Q17.15 to Q8.8
	  mul_16 += shared_bias; //Q8.8
	  if(mul_16 < 0) {
	    mul = (int32_t) mul_16 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	    mul_16 = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
	  }
	  out_d_pos[output_pos6] = mul_16;

	  shared_scales = bias_pos[i+2];
 	  shared_bias = bias_pos[num_ker+i+2];
	  output_conv = (int16_t) ((int32_t) (acc_final3 << 3) >> 16);//Q12.20 to Q9.7
  	  mul = (int32_t) output_conv * (int32_t) shared_scales;//Q9.7 * Q8.8 = Q17.15
	  mul_16 = (int16_t) ((int32_t) (mul << 9) >> 16);//Q17.15 to Q8.8
	  mul_16 += shared_bias; //Q8.8
	  if(mul_16 < 0) {
	    mul = (int32_t) mul_16 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	    mul_16 = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
	  }
	  out_d_pos[output_pos3] = mul_16;
	  output_conv = (int16_t) ((int32_t) (acc_final7 << 3) >> 16);//Q12.20 to Q9.7
  	  mul = (int32_t) output_conv * (int32_t) shared_scales;//Q9.7 * Q8.8 = Q17.15
	  mul_16 = (int16_t) ((int32_t) (mul << 9) >> 16);//Q17.15 to Q8.8
	  mul_16 += shared_bias; //Q8.8
	  if(mul_16 < 0) {
	    mul = (int32_t) mul_16 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	    mul_16 = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
	  }
	  out_d_pos[output_pos7] = mul_16;

	  shared_scales = bias_pos[i+3];
 	  shared_bias = bias_pos[num_ker+i+3];
	  output_conv = (int16_t) ((int32_t) (acc_final4 << 3) >> 16);//Q12.20 to Q9.7
  	  mul = (int32_t) output_conv * (int32_t) shared_scales;//Q9.7 * Q8.8 = Q17.15
	  mul_16 = (int16_t) ((int32_t) (mul << 9) >> 16);//Q17.15 to Q8.8
	  mul_16 += shared_bias; //Q8.8
	  if(mul_16 < 0) {
	    mul = (int32_t) mul_16 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	    mul_16 = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
	  }
	  out_d_pos[output_pos4] = mul_16;
	  output_conv = (int16_t) ((int32_t) (acc_final8 << 3) >> 16);//Q12.20 to Q9.7
  	  mul = (int32_t) output_conv * (int32_t) shared_scales;//Q9.7 * Q8.8 = Q17.15
	  mul_16 = (int16_t) ((int32_t) (mul << 9) >> 16);//Q17.15 to Q8.8
	  mul_16 += shared_bias; //Q8.8
	  if(mul_16 < 0) {
	    mul = (int32_t) mul_16 * (int32_t) leaky; ////Q8.8 * Q1.15 = Q9.23
	    mul_16 = (int16_t) ((int32_t)(mul << 1 ) >> 16); //Convert to Q8.8
	  }
	  out_d_pos[output_pos8] = mul_16;

	}

	//otherwise, only add bias
	else {

	  shared_bias = bias_pos[i];
	  output_conv = (int16_t) ((int32_t) (acc_final << 4) >> 16);//Q12.20 to Q8.8
	  out_d_pos[output_pos] = output_conv + shared_bias;
	  output_conv = (int16_t) ((int32_t) (acc_final5 << 4) >> 16);//Q12.20 to Q8.8
	  out_d_pos[output_pos5] = output_conv + shared_bias;

	  shared_bias = bias_pos[i+1];
	  output_conv = (int16_t) ((int32_t) (acc_final2 << 4) >> 16);//Q12.20 to Q8.8
	  out_d_pos[output_pos2] = output_conv + shared_bias;
	  output_conv = (int16_t) ((int32_t) (acc_final6 << 4) >> 16);//Q12.20 to Q8.8
	  out_d_pos[output_pos6] = output_conv + shared_bias;

	  shared_bias = bias_pos[i+2];
	  output_conv = (int16_t) ((int32_t) (acc_final3 << 4) >> 16);//Q12.20 to Q8.8
	  out_d_pos[output_pos3] = output_conv + shared_bias;
	  output_conv = (int16_t) ((int32_t) (acc_final7 << 4) >> 16);//Q12.20 to Q8.8
	  out_d_pos[output_pos7] = output_conv + shared_bias;

	  shared_bias = bias_pos[i+3];
	  output_conv = (int16_t) ((int32_t) (acc_final4 << 4) >> 16);//Q12.20 to Q8.8
	  out_d_pos[output_pos4] = output_conv + shared_bias;
	  output_conv = (int16_t) ((int32_t) (acc_final8 << 4) >> 16);//Q12.20 to Q8.8
	  out_d_pos[output_pos8] = output_conv + shared_bias;

	}
      }
    }
  }

  //update weights and data pointers
  if(batch_norm) weight_pos += num_ker*2 + num_ker*c*ker_size*ker_size;
  else weight_pos += num_ker + num_ker*c*ker_size*ker_size;
  if(new_output_pos != 0) data_pos = new_output_pos; else data_pos += pos_delta;
}
//perform maxpool layer
void maxpool_layer(int w, int num_ker, int downsample, int ignorePadding, unsigned int new_output_pos) {

  //locate data pointers
  int16_t * in_d_pos = (int16_t *) fp_data + data_pos;
  int16_t * out_d_pos;
  if(new_output_pos != 0) out_d_pos = (int16_t *) fp_data + new_output_pos;
  else out_d_pos = (int16_t *) in_d_pos + (w+1-downsample+2*ignorePadding)*(w+1-downsample+2*ignorePadding)*num_ker;

  //local variables
  int i, j, k, l, m, new_w = w+1-downsample+2*ignorePadding, output_w = (w/(1+downsample))+2;
  int16_t max, max2, max3, max4, val, val2, val3, val4;

  //perform max pooling
  for(i = 0; i < num_ker; i+=4) { 		//Number of kernels
    for(j = 0; j < w/(1+downsample); j++) {	//Output map size
      for(k = 0; k < w/(1+downsample); k++) {
	for(l = 0; l < 2; l++) {		//2x2 block
 	  for(m = 0; m < 2; m++) {
	    val = in_d_pos[i*new_w*new_w + j*new_w*(1+downsample) + k*(1+downsample) + l*new_w + m + (1 + new_w)*ignorePadding];
	    val2 = in_d_pos[(i+1)*new_w*new_w + j*new_w*(1+downsample) + k*(1+downsample) + l*new_w + m + (1 + new_w)*ignorePadding];
	    val3 = in_d_pos[(i+2)*new_w*new_w + j*new_w*(1+downsample) + k*(1+downsample) + l*new_w + m + (1 + new_w)*ignorePadding];
	    val4 = in_d_pos[(i+3)*new_w*new_w + j*new_w*(1+downsample) + k*(1+downsample) + l*new_w + m + (1 + new_w)*ignorePadding];
	    if(l == 0 && m == 0) { max = val; max2 = val2; max3 = val3; max4 = val4; }
	    else { if(max < val) max = val; if(max2 < val2) max2 = val2; if(max3 < val3) max3 = val3; if(max4 < val4) max4 = val4; }
	  }
	}
	out_d_pos[i*output_w*output_w + (j+1)*output_w + (k+1)] = max;
	out_d_pos[(i+1)*output_w*output_w + (j+1)*output_w + (k+1)] = max2;
	out_d_pos[(i+2)*output_w*output_w + (j+1)*output_w + (k+1)] = max3;
	out_d_pos[(i+3)*output_w*output_w + (j+1)*output_w + (k+1)] = max4;
      }
    }
  }

  //update data pointer
  if(new_output_pos != 0) data_pos = new_output_pos;
  else data_pos += (w+1-downsample+2*ignorePadding)*(w+1-downsample+2*ignorePadding)*num_ker;
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

//send detection results back
void send_data(unsigned int data_pos_yolo, unsigned int data_amount) {

  //char file pointers
  unsigned int pos = 2*data_pos_yolo;
  char * fp_data_char = (char *) (DATA_BASE_ADDRESS + pos) ;
  int i, j;

  //layer parameters
  unsigned int LAYER_FILE_SIZE = data_amount*2;
  unsigned int NUM_LAYER_FRAMES = LAYER_FILE_SIZE/ETH_NBYTES;

  //Loop to send output of yolo layer
  for(j = 0; j < NUM_LAYER_FRAMES+1; j++) {

     // start timer
     if(j == 0) {
       count_bytes = 0;
       timer_reset(TIMER);
       start = timer_get_count_us(TIMER);
     }

     //check if it is last packet (has less data that full payload size)
     if(j == NUM_LAYER_FRAMES) bytes_to_send = LAYER_FILE_SIZE - count_bytes;
     else bytes_to_send = ETH_NBYTES;

     //prepare variable to be sent
     for(i = 0; i < bytes_to_send; i++) data_to_send[i] = fp_data_char[j*ETH_NBYTES + i];

     //send frame
     eth_send_frame(data_to_send, ETH_NBYTES);

     //update byte counter
     count_bytes += ETH_NBYTES;
  }

  //measure transference time
  end = timer_get_count_us(TIMER);
  uart_printf("\noutput layer transferred in %d ms\n\n", (end-start)/1000);
}

//reset certain DDR positions to zero due to padding
void reset_DDR() {
  
  //local variables
  unsigned int i, pos;

  //measure initial time
  uart_printf("\nSetting DDR positions to zero\n");
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);

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

int main(int argc, char **argv) {

  //init UART
  uart_init(UART,UART_CLK_FREQ/UART_BAUD_RATE);

  //send init message
  uart_printf("\nYOLO SW \n\n");
  uart_txwait();

  //init ETHERNET
  eth_init(ETHERNET);
  eth_set_rx_payload_size(ETH_NBYTES);

  //define memory regions
  define_memory_regions();
  unsigned int total_time;

  //load data and reset DDR to zero
#ifndef SIM
  receive_data();
  reset_DDR();
  ctrl_counter_reset(CACHE_CTRL);
#endif

  //layer1 (418x418x3 -> 416x416x16)
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  conv_layer_unroll_4_2(NTW_IN_W, NTW_IN_C, NTW_IN_NUM_KER, NTW_IN_KER_SIZE, NTW_IN_PAD, NTW_IN_BATCH_NORM, NTW_IN_NEXT_PADD, NTW_IN_NEXT_STRIDE, NTW_IN_IGNORE_PADD, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer1 %d ms\n", (end-start)/1000);
  total_time = (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //layer2 (416x416x16 -> 210x210x16)
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  maxpool_layer(LAYER_2_W, LAYER_2_NUM_KER, LAYER_2_DOWNSAMPLE, LAYER_2_IGNORE_PADD, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer2 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //layer3 (210x210x16 -> 208x208x32)
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  conv_layer_unroll_4_2_0_2(LAYER_3_W, LAYER_3_C, LAYER_3_NUM_KER, LAYER_3_KER_SIZE, LAYER_3_PAD, LAYER_3_BATCH_NORM, LAYER_3_NEXT_PADD, LAYER_3_NEXT_STRIDE, LAYER_3_IGNORE_PADD, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer3 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //layer4 (208x208x32 -> 106x106x32)
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  maxpool_layer(LAYER_4_W, LAYER_4_NUM_KER, LAYER_4_DOWNSAMPLE, LAYER_4_IGNORE_PADD, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer4 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //layer5 (106x106x32 -> 104x104x64)
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  conv_layer_unroll_4_2_0_2(LAYER_5_W, LAYER_5_C, LAYER_5_NUM_KER, LAYER_5_KER_SIZE, LAYER_5_PAD, LAYER_5_BATCH_NORM, LAYER_5_NEXT_PADD, LAYER_5_NEXT_STRIDE, LAYER_5_IGNORE_PADD, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer5 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //layer6 (104x104x64 -> 54x54x64)
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  maxpool_layer(LAYER_6_W, LAYER_6_NUM_KER, LAYER_6_DOWNSAMPLE, LAYER_6_IGNORE_PADD, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer6 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //layer7 (54x54x64 -> 52x52x128)
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  conv_layer_unroll_4_2_0_2(LAYER_7_W, LAYER_7_C, LAYER_7_NUM_KER, LAYER_7_KER_SIZE, LAYER_7_PAD, LAYER_7_BATCH_NORM, LAYER_7_NEXT_PADD, LAYER_7_NEXT_STRIDE, LAYER_7_IGNORE_PADD, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer7 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //layer8 (52x52x128 -> 28x28x128)
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  maxpool_layer(LAYER_8_W, LAYER_8_NUM_KER, LAYER_8_DOWNSAMPLE, LAYER_8_IGNORE_PADD, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer8 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //Initial address of layer 10 output
  unsigned int data_pos_layer8 = data_pos + DATA_LAYER_8;

  //layer9 (28x28x128 -> 28x28x256) -> Zero-padding
  //Result of layer 9 goes after result of layer 20
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  conv_layer_unroll_4_2_0_2(LAYER_9_W, LAYER_9_C, LAYER_9_NUM_KER, LAYER_9_KER_SIZE, LAYER_9_PAD, LAYER_9_BATCH_NORM, LAYER_9_NEXT_PADD, LAYER_9_NEXT_STRIDE, LAYER_9_IGNORE_PADD, data_pos + DATA_LAYER_8 + DATA_LAYER_10 + DATA_LAYER_11 + DATA_LAYER_12 + DATA_LAYER_13 + DATA_LAYER_14 + DATA_LAYER_15 + DATA_LAYER_16 + DATA_LAYER_17 + DATA_LAYER_19 + DATA_LAYER_20);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer9 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //layer10 (28x28x256 -> 15x15x256) -> Ignores padding from layer 9
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  maxpool_layer(LAYER_10_W, LAYER_10_NUM_KER, LAYER_10_DOWNSAMPLE, LAYER_10_IGNORE_PADD, data_pos_layer8);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer10 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //layer11 (15x15x256 -> 14x14x512)
  //Repeats last line and column of each feature map
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  conv_layer_unroll_4_0_0_2(LAYER_11_W, LAYER_11_C, LAYER_11_NUM_KER, LAYER_11_KER_SIZE, LAYER_11_PAD, LAYER_11_BATCH_NORM, LAYER_11_NEXT_PADD, LAYER_11_NEXT_STRIDE, LAYER_11_IGNORE_PADD, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer11 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //layer12 (14x14x512 -> 15x15x512)
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  maxpool_layer(LAYER_12_W, LAYER_12_NUM_KER, LAYER_12_DOWNSAMPLE, LAYER_12_IGNORE_PADD, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer12 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //layer13 (15x15x512 -> 13x13x1024)
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  conv_layer_unroll_4_0_0_2(LAYER_13_W, LAYER_13_C, LAYER_13_NUM_KER, LAYER_13_KER_SIZE, LAYER_13_PAD, LAYER_13_BATCH_NORM, LAYER_13_NEXT_PADD, LAYER_13_NEXT_STRIDE, LAYER_13_IGNORE_PADD, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer13 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //layer14 (13x13x1024 -> 15x15x256)
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  conv_layer_unroll_4_0_0_2(LAYER_14_W, LAYER_14_C, LAYER_14_NUM_KER, LAYER_14_KER_SIZE, LAYER_14_PAD, LAYER_14_BATCH_NORM, LAYER_14_NEXT_PADD, LAYER_14_NEXT_STRIDE, LAYER_14_IGNORE_PADD, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer14 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //Stores initial address of layer 14 output for first route layer
  unsigned int data_pos_layer14 = data_pos;

  //layer15 (15x15x256 -> 13x13x512)
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  conv_layer_unroll_4_0_0_2(LAYER_15_W, LAYER_15_C, LAYER_15_NUM_KER, LAYER_15_KER_SIZE, LAYER_15_PAD, LAYER_15_BATCH_NORM, LAYER_15_NEXT_PADD, LAYER_15_NEXT_STRIDE, LAYER_15_IGNORE_PADD, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer15 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //layer16 (13x13x512 -> 13x13x255)
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  conv_layer_unroll_4_0_0_2(LAYER_16_W, LAYER_16_C, LAYER_16_NUM_KER, LAYER_16_KER_SIZE, LAYER_16_PAD, LAYER_16_BATCH_NORM, LAYER_16_NEXT_PADD, LAYER_16_NEXT_STRIDE, LAYER_16_IGNORE_PADD, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer16 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //layer17 (13x13x255 -> 13x13x255)
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  yolo_layer(LAYER_17_W);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer17 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //Stores initial address of first yolo layer for sending
  unsigned int data_pos_layer17 = data_pos;

  //Stores initial address of the output of layer 19
  unsigned int previous_data_pos = data_pos + DATA_LAYER_17;

  //layer18 (points to the initial address of layer 14 output)
  data_pos = data_pos_layer14;

  //layer19 (15x15x256 -> 13x13x128)
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  conv_layer_unroll_4_0_0_2(LAYER_19_W, LAYER_19_C, LAYER_19_NUM_KER, LAYER_19_KER_SIZE, LAYER_19_PAD, LAYER_19_BATCH_NORM, LAYER_19_NEXT_PADD, LAYER_19_NEXT_STRIDE, LAYER_19_IGNORE_PADD, previous_data_pos);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer19 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //layer20 (13x13x128 -> 28x28x128)
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  upsample_layer(LAYER_20_W, LAYER_20_NUM_KER);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer20 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //layer22 (28x28x128 -> 26x26x256)
  //layer 21 (second route layer) is not needed as output of layer 9 is already after output of layer 20
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  conv_layer_unroll_4_2_0_2(LAYER_22_W, LAYER_22_C, LAYER_22_NUM_KER, LAYER_22_KER_SIZE, LAYER_22_PAD, LAYER_22_BATCH_NORM, LAYER_22_NEXT_PADD, LAYER_22_NEXT_STRIDE, LAYER_22_IGNORE_PADD, data_pos + DATA_LAYER_20 + DATA_LAYER_9);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer22 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //layer23 (26x26x256 -> 26x26x255)
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  conv_layer_unroll_4_2_0_2(LAYER_23_W, LAYER_23_C, LAYER_23_NUM_KER, LAYER_23_KER_SIZE, LAYER_23_PAD, LAYER_23_BATCH_NORM, LAYER_23_NEXT_PADD, LAYER_23_NEXT_STRIDE, LAYER_23_IGNORE_PADD, 0);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer23 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //layer24 (26x26x255 -> 26x26x255)
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);
  yolo_layer(LAYER_24_W);
  end = timer_get_count_us(TIMER);
  uart_printf("\nLayer24 %d ms\n", (end-start)/1000);
  total_time += (end-start)/1000;
  print_cache_status();
  ctrl_counter_reset(CACHE_CTRL);

  //return data
  uart_printf("\ntotal_time = %d seconds (%d minutes) \n", total_time/1000, (total_time/1000)/60);
  send_data(data_pos_layer17, DATA_LAYER_17);
  send_data(data_pos, DATA_LAYER_24);
  uart_putc(4);
  return 0;
}
