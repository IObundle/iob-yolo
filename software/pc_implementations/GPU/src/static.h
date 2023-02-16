/*
static.h

definition of static variables for embedded.c
separated from embedded.h so there is no double declaration of variables in other source files that include embedded.h
*/
#ifndef STATIC_H
#define STATIC_H

//global constants
static int IMG_H, IMG_W;
static int NEW_H, NEW_W;
static int EXTRA_W, EXTRA_H;
static float w_scale, h_scale; // this is also used in FIXED version?
#ifdef FIXED
static int16_t x_scales, x_bias;
static int16_t y_scales, y_bias;
static int16_t w_scales, h_scales;
#else //FLOAT
static float x_scales, x_bias;
static float y_scales, y_bias;
static float w_scales, h_scales;
#endif //ifdef FIXED

//Memory space
static int16_t ix[MAX_NEW_W*4], iy[MAX_NEW_H]; 
#ifdef FIXED
	static int16_t weights[TOTAL_WEIGTHS];
	static int16_t data[TOTAL_DATA];
	static uint8_t label[81 + MAX_LABEL_SIZE*81];
	static int16_t dx[MAX_NEW_W*2], dy[MAX_NEW_H*2];
	#ifdef GEMM
		static int16_t gemm_in[NTW_IN_W*NTW_IN_H*NTW_IN_NUM_KER*NTW_IN_KER_SIZE*NTW_IN_KER_SIZE];
		static int32_t gemm_out[NTW_IN_W*NTW_IN_H*NTW_IN_NUM_KER];
	#endif
#else
	static float weights[TOTAL_WEIGTHS];
	static float data[TOTAL_DATA];
	static float label[81 + MAX_LABEL_SIZE*81];
	static float dx[MAX_NEW_W*2], dy[MAX_NEW_H*2];
	#ifdef GEMM
		static float gemm_in[NTW_IN_W*NTW_IN_H*NTW_IN_NUM_KER*NTW_IN_KER_SIZE*NTW_IN_KER_SIZE];
		static float gemm_out[NTW_IN_W*NTW_IN_H*NTW_IN_NUM_KER];
	#endif
#endif

//Base addresses
#define WEIGTHS_BASE_ADDRESS weights
#define DATA_BASE_ADDRESS data
#define LABEL_BASE_ADDRESS label
#ifdef GEMM
	#define GEMM_IN_BASE_ADDRESS gemm_in
	#define GEMM_OUT_BASE_ADDRESS gemm_out
#endif

#ifdef GPU
//Layer18 (route)
#define LAYER_18_NUM_INPUTS 1
static int layer_18_input_layers[LAYER_18_NUM_INPUTS] = {14};
//layer 14 output = layer 15 input
static int layer_18_input_sizes[LAYER_18_NUM_INPUTS] = {(LAYER_15_W*LAYER_15_W*LAYER_15_C)}; 


//Layer21 (route)
#define LAYER_21_NUM_INPUTS 2
static int layer_21_input_layers[LAYER_21_NUM_INPUTS] = {20, 9};
static int layer_21_input_sizes[LAYER_21_NUM_INPUTS] = { ((LAYER_20_W*2)*(LAYER_20_W*2)*LAYER_20_NUM_KER), (LAYER_9_W*LAYER_9_W*LAYER_9_NUM_KER)};
#endif //ifdef GPU

#endif //ifndef STATIC_H
