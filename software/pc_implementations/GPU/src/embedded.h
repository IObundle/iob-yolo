#ifndef EMBEDDED_H
#define EMBADDED_H
#include <stdint.h>

//Input image
#define MAX_IMG_W (2048)
#define MAX_IMG_H (2048)
#define IMG_C 3
#define IMAGE_INPUT (MAX_IMG_W*MAX_IMG_H*IMG_C)

//Grey region
#ifdef INTERM_DATA
	#define EXTRA_W 0
	#define EXTRA_H 2
	#define GREY_PADD 0
#else
#ifdef GPU
	#define GREY_PADD 0
#else
	#define GREY_PADD 1
#endif //ifdef GPU
//EXTRA_W and EXTRA_H set at runtime
#endif

//Resized image
#define YOLO_INPUT 416
#define MAX_NEW_W YOLO_INPUT
#define MAX_NEW_H YOLO_INPUT

//Input network
#define NTW_IN_W YOLO_INPUT
#define NTW_IN_C IMG_C
#define NTW_IN_NUM_KER 16
#define NTW_IN_KER_SIZE 3
#define NTW_IN_PAD 1
#define NTW_IN_BATCH_NORM 1
#define NTW_IN_NEXT_PADD 0
#define NTW_IN_NEXT_STRIDE 0
#define NTW_IN_IGNORE_PADD 0
#define NTW_IN_OFFSET 1
#define NETWORK_INPUT_AUX (MAX_NEW_W*2*MAX_NEW_H*IMG_C)
#ifdef INTERM_DATA
	#define NTW_IN_H (NEW_H+2)
	#define NETWORK_INPUT ((NTW_IN_W+2*NTW_IN_PAD)*(NTW_IN_H+2*NTW_IN_OFFSET)*NTW_IN_C)
#else
	#define NTW_IN_H NTW_IN_W
	#define NETWORK_INPUT ((NTW_IN_W+2*NTW_IN_PAD)*(NTW_IN_H+2*NTW_IN_PAD)*NTW_IN_C)
#endif

//Layer1 (conv)
#ifdef INTERM_DATA
	#define DATA_LAYER_1 ((NTW_IN_W+NTW_IN_NEXT_STRIDE+2*NTW_IN_NEXT_PADD)*(NTW_IN_H+2*NTW_IN_OFFSET)*NTW_IN_NUM_KER)
#else
	#define DATA_LAYER_1 ((NTW_IN_W+NTW_IN_NEXT_STRIDE+2*NTW_IN_NEXT_PADD)*(NTW_IN_H+NTW_IN_NEXT_STRIDE+2*NTW_IN_NEXT_PADD)*NTW_IN_NUM_KER)
#endif
#define WEIGHTS_LAYER_1 (NTW_IN_NUM_KER+NTW_IN_NUM_KER*NTW_IN_KER_SIZE*NTW_IN_KER_SIZE*NTW_IN_C)

//Layer2 (maxpool)
#define LAYER_2_W NTW_IN_W
#define LAYER_2_NUM_KER NTW_IN_NUM_KER
#define LAYER_2_DOWNSAMPLE 1
#define LAYER_2_IGNORE_PADD 0
#ifdef INTERM_DATA
	#define LAYER_2_H (NTW_IN_H+2*NTW_IN_OFFSET)
	#define DATA_LAYER_2 ((LAYER_2_W/(1+LAYER_2_DOWNSAMPLE)+2)*(LAYER_2_H/2+4)*LAYER_2_NUM_KER)
#else
	#define LAYER_2_H LAYER_2_W
	#define DATA_LAYER_2 ((LAYER_2_W/(1+LAYER_2_DOWNSAMPLE)+2)*(LAYER_2_H/(1+LAYER_2_DOWNSAMPLE)+2)*LAYER_2_NUM_KER)
#endif

//Layer3 (conv)
#define LAYER_3_W (LAYER_2_W/2)
#define LAYER_3_C LAYER_2_NUM_KER
#define LAYER_3_NUM_KER LAYER_3_C*2
#define LAYER_3_KER_SIZE 3
#define LAYER_3_PAD 1
#define LAYER_3_BATCH_NORM 1
#define LAYER_3_NEXT_PADD 0
#define LAYER_3_NEXT_STRIDE 0
#define LAYER_3_IGNORE_PADD 0
#define LAYER_3_OFFSET 0
#ifdef INTERM_DATA
	#define LAYER_3_H (LAYER_2_H/2+2)
	#define DATA_LAYER_3 ((LAYER_3_W+LAYER_3_NEXT_STRIDE+2*LAYER_3_NEXT_PADD)*(LAYER_3_H+2*LAYER_3_OFFSET)*LAYER_3_NUM_KER)
#else
	#define LAYER_3_H LAYER_3_W
	#define DATA_LAYER_3 ((LAYER_3_W+LAYER_3_NEXT_STRIDE+2*LAYER_3_NEXT_PADD)*(LAYER_3_H+LAYER_3_NEXT_STRIDE+2*LAYER_3_NEXT_PADD)*LAYER_3_NUM_KER)
#endif
#define WEIGHTS_LAYER_3 (LAYER_3_NUM_KER+LAYER_3_NUM_KER*LAYER_3_KER_SIZE*LAYER_3_KER_SIZE*LAYER_3_C)

//Layer4 (maxpool)
#define LAYER_4_W LAYER_3_W
#define LAYER_4_NUM_KER LAYER_3_NUM_KER
#define LAYER_4_DOWNSAMPLE 1
#define LAYER_4_IGNORE_PADD 0
#ifdef INTERM_DATA
	#define LAYER_4_H (LAYER_3_H+2*LAYER_3_OFFSET)
	#define DATA_LAYER_4 ((LAYER_4_W/(1+LAYER_4_DOWNSAMPLE)+2)*(LAYER_4_H/2+4)*LAYER_4_NUM_KER)
#else
	#define LAYER_4_H LAYER_4_W
	#define DATA_LAYER_4 ((LAYER_4_W/(1+LAYER_4_DOWNSAMPLE)+2)*(LAYER_4_H/(1+LAYER_4_DOWNSAMPLE)+2)*LAYER_4_NUM_KER)
#endif

//Layer5 (conv)
#define LAYER_5_W (LAYER_4_W/2)
#define LAYER_5_C LAYER_4_NUM_KER
#define LAYER_5_NUM_KER LAYER_5_C*2
#define LAYER_5_KER_SIZE 3
#define LAYER_5_PAD 1
#define LAYER_5_BATCH_NORM 1
#define LAYER_5_NEXT_PADD 0
#define LAYER_5_NEXT_STRIDE 0
#define LAYER_5_IGNORE_PADD 0
#define LAYER_5_OFFSET 1
#ifdef INTERM_DATA
	#define LAYER_5_H (LAYER_4_H/2+2)
	#define DATA_LAYER_5 ((LAYER_5_W+LAYER_5_NEXT_STRIDE+2*LAYER_5_NEXT_PADD)*(LAYER_5_H+2*LAYER_5_OFFSET)*LAYER_5_NUM_KER)
#else
	#define LAYER_5_H LAYER_5_W
	#define DATA_LAYER_5 ((LAYER_5_W+LAYER_5_NEXT_STRIDE+2*LAYER_5_NEXT_PADD)*(LAYER_5_H+LAYER_5_NEXT_STRIDE+2*LAYER_5_NEXT_PADD)*LAYER_5_NUM_KER)
#endif
#define WEIGHTS_LAYER_5 (LAYER_5_NUM_KER+LAYER_5_NUM_KER*LAYER_5_KER_SIZE*LAYER_5_KER_SIZE*LAYER_5_C)

//Layer6 (maxpool)
#define LAYER_6_W LAYER_5_W
#define LAYER_6_NUM_KER LAYER_5_NUM_KER
#define LAYER_6_DOWNSAMPLE 1
#define LAYER_6_IGNORE_PADD 0
#ifdef INTERM_DATA
	#define LAYER_6_H (LAYER_5_H+2*LAYER_5_OFFSET)
	#define DATA_LAYER_6 ((LAYER_6_W/2+(1+LAYER_6_DOWNSAMPLE))*(LAYER_6_H/2+4)*LAYER_6_NUM_KER)
#else
	#define LAYER_6_H LAYER_6_W
	#define DATA_LAYER_6 ((LAYER_6_W/2+(1+LAYER_6_DOWNSAMPLE))*(LAYER_6_H/(1+LAYER_6_DOWNSAMPLE)+2)*LAYER_6_NUM_KER)
#endif

//Layer7 (conv)
#define LAYER_7_W (LAYER_6_W/2)
#define LAYER_7_C LAYER_6_NUM_KER
#define LAYER_7_NUM_KER LAYER_7_C*2
#define LAYER_7_KER_SIZE 3
#define LAYER_7_PAD 1
#define LAYER_7_BATCH_NORM 1
#define LAYER_7_NEXT_PADD 0
#define LAYER_7_NEXT_STRIDE 0
#define LAYER_7_IGNORE_PADD 0
#define LAYER_7_OFFSET 0
#ifdef INTERM_DATA
	#define LAYER_7_H (LAYER_6_H/2+2)
	#define DATA_LAYER_7 ((LAYER_7_W+LAYER_7_NEXT_STRIDE+2*LAYER_7_NEXT_PADD)*(LAYER_7_H+2*LAYER_7_OFFSET)*LAYER_7_NUM_KER)
#else
	#define LAYER_7_H LAYER_7_W
	#define DATA_LAYER_7 ((LAYER_7_W+LAYER_7_NEXT_STRIDE+2*LAYER_7_NEXT_PADD)*(LAYER_7_H+LAYER_7_NEXT_STRIDE+2*LAYER_7_NEXT_PADD)*LAYER_7_NUM_KER)
#endif
#define WEIGHTS_LAYER_7 (LAYER_7_NUM_KER+LAYER_7_NUM_KER*LAYER_7_KER_SIZE*LAYER_7_KER_SIZE*LAYER_7_C)

//Layer8 (maxpool)
#define LAYER_8_W LAYER_7_W
#define LAYER_8_NUM_KER LAYER_7_NUM_KER
#define LAYER_8_DOWNSAMPLE 1
#define LAYER_8_IGNORE_PADD 0
#ifdef INTERM_DATA
	#define LAYER_8_H (LAYER_7_H+2*LAYER_7_OFFSET)
	#define DATA_LAYER_8 ((LAYER_8_W/(1+LAYER_8_DOWNSAMPLE)+2)*(LAYER_8_H/2+4)*LAYER_8_NUM_KER)
#else
	#define LAYER_8_H LAYER_8_W
	#define DATA_LAYER_8 ((LAYER_8_W/(1+LAYER_8_DOWNSAMPLE)+2)*(LAYER_8_H/(1+LAYER_8_DOWNSAMPLE)+2)*LAYER_8_NUM_KER)
#endif

//Layer9 (conv)
#define LAYER_9_W (LAYER_8_W/2)
#define LAYER_9_C LAYER_8_NUM_KER
#define LAYER_9_NUM_KER LAYER_9_C*2
#define LAYER_9_KER_SIZE 3
#define LAYER_9_PAD 1
#define LAYER_9_BATCH_NORM 1
#define LAYER_9_NEXT_PADD 1
#define LAYER_9_NEXT_STRIDE 0
#define LAYER_9_IGNORE_PADD 0
#define LAYER_9_OFFSET 1
#ifdef INTERM_DATA
	#define LAYER_9_H (LAYER_8_H/2+2)
	#define DATA_LAYER_9 ((LAYER_9_W+LAYER_9_NEXT_STRIDE+2*LAYER_9_NEXT_PADD)*(LAYER_9_W+LAYER_9_NEXT_STRIDE+2*LAYER_9_NEXT_PADD)*LAYER_9_NUM_KER)
#else
	#define LAYER_9_H LAYER_9_W
	#define DATA_LAYER_9 ((LAYER_9_W+LAYER_9_NEXT_STRIDE+2*LAYER_9_NEXT_PADD)*(LAYER_9_H+LAYER_9_NEXT_STRIDE+2*LAYER_9_NEXT_PADD)*LAYER_9_NUM_KER)
#endif
#define WEIGHTS_LAYER_9 (LAYER_9_NUM_KER+LAYER_9_NUM_KER*LAYER_9_KER_SIZE*LAYER_9_KER_SIZE*LAYER_9_C)

//Layer10 (maxpool)
#define LAYER_10_W LAYER_9_W
#define LAYER_10_NUM_KER LAYER_9_NUM_KER
#define LAYER_10_DOWNSAMPLE 1
#define LAYER_10_IGNORE_PADD 1
#define DATA_LAYER_10 ((LAYER_10_W/(1+LAYER_10_DOWNSAMPLE)+2)*(LAYER_10_W/(1+LAYER_10_DOWNSAMPLE)+2)*LAYER_10_NUM_KER)

//Layer11 (conv)
#define LAYER_11_W (LAYER_10_W/2)
#define LAYER_11_C LAYER_10_NUM_KER
#define LAYER_11_NUM_KER LAYER_11_C*2
#define LAYER_11_KER_SIZE 3
#define LAYER_11_PAD 1
#define LAYER_11_BATCH_NORM 1
#define LAYER_11_NEXT_PADD 0
#define LAYER_11_NEXT_STRIDE 1
#define LAYER_11_IGNORE_PADD 0
#define LAYER_11_OFFSET 0
#define DATA_LAYER_11 ((LAYER_11_W+LAYER_11_NEXT_STRIDE+2*LAYER_11_NEXT_PADD)*(LAYER_11_W+LAYER_11_NEXT_STRIDE+2*LAYER_11_NEXT_PADD)*LAYER_11_NUM_KER)
#define WEIGHTS_LAYER_11 (LAYER_11_NUM_KER+LAYER_11_NUM_KER*LAYER_11_KER_SIZE*LAYER_11_KER_SIZE*LAYER_11_C)

//Layer12 (maxpool)
#define LAYER_12_W LAYER_11_W
#define LAYER_12_NUM_KER LAYER_11_NUM_KER
#define LAYER_12_DOWNSAMPLE 0
#define LAYER_12_IGNORE_PADD 0
#define DATA_LAYER_12 ((LAYER_12_W/(1+LAYER_12_DOWNSAMPLE)+2)*(LAYER_12_W/(1+LAYER_12_DOWNSAMPLE)+2)*LAYER_12_NUM_KER)

//Layer13 (conv)
#define LAYER_13_W LAYER_12_W
#define LAYER_13_C LAYER_12_NUM_KER
#define LAYER_13_NUM_KER LAYER_13_C*2
#define LAYER_13_KER_SIZE 3
#define LAYER_13_PAD 1
#define LAYER_13_BATCH_NORM 1
#define LAYER_13_NEXT_PADD 0
#define LAYER_13_NEXT_STRIDE 0
#define LAYER_13_IGNORE_PADD 0
#define LAYER_13_OFFSET 0
#define DATA_LAYER_13 ((LAYER_13_W+LAYER_13_NEXT_STRIDE+2*LAYER_13_NEXT_PADD)*(LAYER_13_W+LAYER_13_NEXT_STRIDE+2*LAYER_13_NEXT_PADD)*LAYER_13_NUM_KER)
#define WEIGHTS_LAYER_13 (LAYER_13_NUM_KER+LAYER_13_NUM_KER*LAYER_13_KER_SIZE*LAYER_13_KER_SIZE*LAYER_13_C)

//Layer14 (conv)
#define LAYER_14_W LAYER_13_W
#define LAYER_14_C LAYER_13_NUM_KER
#define LAYER_14_NUM_KER (LAYER_14_C/4)
#define LAYER_14_KER_SIZE 1
#define LAYER_14_PAD 0
#define LAYER_14_BATCH_NORM 1
#define LAYER_14_NEXT_PADD 1
#define LAYER_14_NEXT_STRIDE 0
#define LAYER_14_IGNORE_PADD 0
#define LAYER_14_OFFSET 0
#define DATA_LAYER_14 ((LAYER_14_W+LAYER_14_NEXT_STRIDE+2*LAYER_14_NEXT_PADD)*(LAYER_14_W+LAYER_14_NEXT_STRIDE+2*LAYER_14_NEXT_PADD)*LAYER_14_NUM_KER)
#define WEIGHTS_LAYER_14 (LAYER_14_NUM_KER+LAYER_14_NUM_KER*LAYER_14_KER_SIZE*LAYER_14_KER_SIZE*LAYER_14_C)

//Layer15 (conv)
#define LAYER_15_W LAYER_14_W
#define LAYER_15_C LAYER_14_NUM_KER
#define LAYER_15_NUM_KER (LAYER_15_C*2)
#define LAYER_15_KER_SIZE 3
#define LAYER_15_PAD 1
#define LAYER_15_BATCH_NORM 1
#define LAYER_15_NEXT_PADD 0
#define LAYER_15_NEXT_STRIDE 0
#define LAYER_15_IGNORE_PADD 0
#define LAYER_15_OFFSET 0
#define DATA_LAYER_15 ((LAYER_15_W+LAYER_15_NEXT_STRIDE+2*LAYER_15_NEXT_PADD)*(LAYER_15_W+LAYER_15_NEXT_STRIDE+2*LAYER_15_NEXT_PADD)*LAYER_15_NUM_KER)
#define WEIGHTS_LAYER_15 (LAYER_15_NUM_KER+LAYER_15_NUM_KER*LAYER_15_KER_SIZE*LAYER_15_KER_SIZE*LAYER_15_C)

//Layer16 (conv)
#define LAYER_16_W LAYER_15_W
#define LAYER_16_C LAYER_15_NUM_KER
#define LAYER_16_NUM_KER 255
#define LAYER_16_KER_SIZE 1
#define LAYER_16_PAD 0
#define LAYER_16_BATCH_NORM 0
#define LAYER_16_NEXT_PADD 0
#define LAYER_16_NEXT_STRIDE 0
#define LAYER_16_IGNORE_PADD 0
#define LAYER_16_OFFSET 0
#define DATA_LAYER_16 ((LAYER_16_W+LAYER_16_NEXT_STRIDE+2*LAYER_16_NEXT_PADD)*(LAYER_16_W+LAYER_16_NEXT_STRIDE+2*LAYER_16_NEXT_PADD)*LAYER_16_NUM_KER)
#define WEIGHTS_LAYER_16 (LAYER_16_NUM_KER+LAYER_16_NUM_KER*LAYER_16_KER_SIZE*LAYER_16_KER_SIZE*LAYER_16_C)

//Layer 17 (yolo)
#define LAYER_17_W LAYER_16_W
#define DATA_LAYER_17 (LAYER_17_W*LAYER_17_W*255)

//Layer19 (conv)
#define LAYER_19_W LAYER_14_W
#define LAYER_19_C LAYER_14_NUM_KER
#define LAYER_19_NUM_KER (LAYER_19_C/2)
#define LAYER_19_KER_SIZE 1
#define LAYER_19_PAD 1
#define LAYER_19_BATCH_NORM 1
#define LAYER_19_NEXT_PADD 0
#define LAYER_19_NEXT_STRIDE 0
#define LAYER_19_IGNORE_PADD 1
#define LAYER_19_OFFSET 0
#define DATA_LAYER_19 ((LAYER_19_W+LAYER_19_NEXT_STRIDE+2*LAYER_19_NEXT_PADD)*(LAYER_19_W+LAYER_19_NEXT_STRIDE+2*LAYER_19_NEXT_PADD)*LAYER_19_NUM_KER)
#define WEIGHTS_LAYER_19 (LAYER_19_NUM_KER+LAYER_19_NUM_KER*LAYER_19_KER_SIZE*LAYER_19_KER_SIZE*LAYER_19_C)

//Layer20 (upsample)
#define LAYER_20_W LAYER_19_W
#define LAYER_20_NUM_KER LAYER_19_NUM_KER
#define DATA_LAYER_20 ((LAYER_20_W*2+2)*(LAYER_20_W*2+2)*LAYER_20_NUM_KER)

//Layer22 (conv)
#define LAYER_22_W (LAYER_20_W*2)
#define LAYER_22_C (LAYER_20_NUM_KER + LAYER_9_NUM_KER)
#define LAYER_22_NUM_KER LAYER_9_NUM_KER
#define LAYER_22_KER_SIZE 3
#define LAYER_22_PAD 1
#define LAYER_22_BATCH_NORM 1
#define LAYER_22_NEXT_PADD 0
#define LAYER_22_NEXT_STRIDE 0
#define LAYER_22_IGNORE_PADD 0
#define LAYER_22_OFFSET 0
#define DATA_LAYER_22 ((LAYER_22_W+LAYER_22_NEXT_STRIDE+2*LAYER_22_NEXT_PADD)*(LAYER_22_W+LAYER_22_NEXT_STRIDE+2*LAYER_22_NEXT_PADD)*LAYER_22_NUM_KER)
#define WEIGHTS_LAYER_22 (LAYER_22_NUM_KER+LAYER_22_NUM_KER*LAYER_22_KER_SIZE*LAYER_22_KER_SIZE*LAYER_22_C)

//Layer23 (conv)
#define LAYER_23_W LAYER_22_W
#define LAYER_23_C LAYER_22_NUM_KER
#define LAYER_23_NUM_KER 255
#define LAYER_23_KER_SIZE 1
#define LAYER_23_PAD 0
#define LAYER_23_BATCH_NORM 0
#define LAYER_23_NEXT_PADD 0
#define LAYER_23_NEXT_STRIDE 0
#define LAYER_23_IGNORE_PADD 0
#define LAYER_23_OFFSET 0
#define DATA_LAYER_23 ((LAYER_23_W+LAYER_23_NEXT_STRIDE+2*LAYER_23_NEXT_PADD)*(LAYER_23_W+LAYER_23_NEXT_STRIDE+2*LAYER_23_NEXT_PADD)*LAYER_23_NUM_KER)
#define WEIGHTS_LAYER_23 (LAYER_23_NUM_KER+LAYER_23_NUM_KER*LAYER_23_KER_SIZE*LAYER_23_KER_SIZE*LAYER_23_C)

//Layer 24 (yolo)
#define LAYER_24_W LAYER_23_W
#define DATA_LAYER_24 (LAYER_24_W*LAYER_24_W*255)

//Total
#define TOTAL_WEIGTHS (WEIGHTS_LAYER_1 + WEIGHTS_LAYER_3 + WEIGHTS_LAYER_5 + WEIGHTS_LAYER_7 + WEIGHTS_LAYER_9 + WEIGHTS_LAYER_11 + WEIGHTS_LAYER_13 + WEIGHTS_LAYER_14 + WEIGHTS_LAYER_15 + WEIGHTS_LAYER_16 + WEIGHTS_LAYER_19 + WEIGHTS_LAYER_22 + WEIGHTS_LAYER_23)
#define TOTAL_DATA (IMAGE_INPUT + NETWORK_INPUT_AUX + NETWORK_INPUT + DATA_LAYER_1 + DATA_LAYER_2 + DATA_LAYER_3 + DATA_LAYER_4 + DATA_LAYER_5 + DATA_LAYER_6 + DATA_LAYER_7 + DATA_LAYER_8 + DATA_LAYER_9 + DATA_LAYER_10 + DATA_LAYER_11 + DATA_LAYER_12 + DATA_LAYER_13 + DATA_LAYER_14 + DATA_LAYER_15 + DATA_LAYER_16 + DATA_LAYER_19 + DATA_LAYER_20 + DATA_LAYER_22 + DATA_LAYER_23 + (DATA_LAYER_17+DATA_LAYER_24))

//Label parameters
#define MAX_LABEL_SIZE 2340


// Bounding boxes definitions
#ifdef FIXED

	//Constants for bounding boxes
#ifdef mAP
	#define threshold ((int16_t)(((float)0.005)*((int32_t)1<<7))) //Q9.7
        #define threshold_Q3_13 ((int16_t)(((float)0.005)*((int32_t)1<<13))) //Q3.13
#else
	#define threshold ((int16_t)(((float)0.5)*((int32_t)1<<7))) //Q9.7
	#define threshold_Q3_13 ((int16_t)(((float)0.5)*((int32_t)1<<13))) //Q3.13
#endif
	#define nms_threshold ((int16_t)(((float)0.45)*((int32_t)1<<13))) //Q3.13
	#define yolo1_div ((int16_t)(((float)1/LAYER_17_W)*((int32_t)1<<15))) //Q1.15
	#define yolo2_div ((int16_t)(((float)1/LAYER_24_W)*((int32_t)1<<15))) //Q1.15
#else //FLOAT
	//Constants for bounding boxes
#ifdef mAP
	#define threshold ((float)0.005)
#else
	#define threshold ((float)0.5)
#endif
	#define nms_threshold ((float)0.45)
	#define yolo1_div ((float)1/LAYER_17_W)
	#define yolo2_div ((float)1/LAYER_24_W)
#endif //ifdef FIXED

#ifndef GPU
//Box definitions
#define BOX_SIZE (84)
#define BOX_CLASS_OFFSET (4)
#endif // ifndef GPU

#ifdef GPU
extern int gpu_index;
#define NUM_LAYERS (24)
#define GPU_INDEX (0)

//Route layers definition

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>

#define BLOCK 512
#define MIN_VAL_INT16 (-32768)

//Box definitions
#define BOX_SIZE (85)
#define BOX_CLASS_OFFSET (5)

//activation definitions
#define LEAKY (0)
#define LEAKY_FP (3276) //0.1 in Q1.15 format
#define LOGISTIC (1)

//net_output_gpu definitions
#define NET_GPU_SIZE (NUM_LAYERS+3)

//Common for net_output_gpu
#define NET_GPU_WORKSPACE (NUM_LAYERS+1)
#define NET_GPU_WEIGHTS (NUM_LAYERS+2)
//largest convolution input (after image unrolling)
#define WORKSPACE_SIZE (NTW_IN_W*NTW_IN_H*NTW_IN_NUM_KER*NTW_IN_KER_SIZE*NTW_IN_KER_SIZE)

// cuda functions
void error(const char *s);
void cuda_set_device(int n);
int cuda_get_device();
void check_error(cudaError_t status);
cublasHandle_t blas_handle();
dim3 cuda_gridsize(size_t n);
float *cuda_make_array(float *x, size_t n);
int16_t *cuda_make_fp_array(int16_t *x, size_t n);
void cuda_free(void *x_gpu);
void cuda_pull_array(float *x_gpu, float *x, size_t n);
void cuda_pull_fp_array(int16_t *x_gpu, int16_t* x, size_t n);
void free_net_output_gpu(void** net_output_gpu, int size);
//forward layer functions
void fill_gpu(int N, float ALPHA, float * X, int INCX);
void fill_fp_gpu(int N, int16_t ALPHA, int16_t * X, int INCX);
#ifdef FIXED
void copy_fp2float_array_gpu(int N, int16_t *X, float* Y, int fracFP);
void copy_float2fp_array_gpu(int N, float *X, int16_t* Y, int fracFP);
void im2col_gpu(int16_t *im,
         int channels, int height, int width,
		int ksize, int stride, int pad, float *data_col, int fracFP);
void im2col_fp_gpu(int16_t *im,
         int channels, int height, int width,
		int ksize, int stride, int pad, int16_t *data_col);
void add_bias_fp_gpu(int16_t *output, int16_t *biases, int batch, int n, int size, int bias_shift);
void add_bias_gpu(int16_t *output, int16_t *biases, int batch, int n, int size);
void activate_array_gpu(int16_t *x, int n, int a);
void copy_fp_gpu(int N, int16_t * X, int INCX,  int16_t * Y, int INCY);
void upsample_gpu(int16_t *in, int w, int h, int c, int batch, int stride, int forward, int scale, int16_t *out);
void gemm_fp_gpu(int16_t *A_gpu, int16_t *B_gpu, int16_t *C_gpu, int M, int N, int K);
#else //Float
void im2col_gpu(float *im,
         int channels, int height, int width,
		int ksize, int stride, int pad, float *data_col);
void add_bias_gpu(float *output, float *biases, int batch, int n, int size);
void activate_array_gpu(float *x, int n, int a);
void upsample_gpu(float *in, int w, int h, int c, int batch, int stride, int forward, float scale, float *out);
#endif
void gemm_gpu(int TA, int TB, int M, int N, int K, float ALPHA, 
        float *A_gpu, int lda, 
        float *B_gpu, int ldb,
        float BETA,
	      float *C_gpu, int ldc);
void copy_gpu(int N, float * X, int INCX, float * Y, int INCY);

//top level forward layer functions
#ifdef FIXED
void forward_convolutional_layer_gpu(int w, int h, int c, int num_ker, int ker_size, int pad, int batch_norm, int nextPadding, int nextStride, int ignorePadding, unsigned int new_output_pos, unsigned int offset, void** net_output_gpu, int layer_num, int16_t* fp_weights, unsigned int* weight_pos);
void forward_yolo_layer_gpu(int w, void** net_output_gpu, int layer_num, int16_t* fp_data, int yolo_layer_output_pos);
void forward_maxpool_layer_gpu(int w, int h, int num_ker, int downsample, int ignorePadding, unsigned int new_output_pos, void** net_output_gpu, int layer_num);
void forward_route_layer_gpu(int inputs_num, int* input_layers, int* input_sizes, void** net_output_gpu, int layer_num);
void forward_upsample_layer_gpu(int w, int num_ker, void ** net_output_gpu, int layer_num);
#else //Float
void forward_convolutional_layer_gpu(int w, int h, int c, int num_ker, int ker_size, int pad, int batch_norm, int nextPadding, int nextStride, int ignorePadding, unsigned int new_output_pos, unsigned int offset, void** net_output_gpu, int layer_num, float* fp_weights, unsigned int* weight_pos);// DONE
void forward_maxpool_layer_gpu(int w, int h, int num_ker, int downsample, int ignorePadding, unsigned int new_output_pos, void** net_output_gpu, int layer_num); //DONE
void forward_yolo_layer_gpu(int w, void** net_output_gpu, int layer_num, float* fp_data, int yolo_layer_output_pos); //DONE
void forward_route_layer_gpu(int inputs_num, int* input_layers, int* input_sizes, void** net_output_gpu, int layer_num); //DONE
void forward_upsample_layer_gpu(int w, int num_ker, void ** net_output_gpu, int layer_num); //DONE
#endif //ifdef FIXED



#endif //ifdef GPU
#endif //ifndef EMBEDDED_H
