//Libraries
#include "embedded.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <float.h>
#include <math.h>

int print_vals;

//Variables dependent on image input
int IMG_W, IMG_H, IMG_C, IMAGE_INPUT;
int EXTRA_W, EXTRA_H, NEW_W, NEW_H, NETWORK_INPUT_AUX;

int item_count;

//Constants for image resize
#define w_scale ((float)(IMG_W-1)/(NEW_W-1))
#define h_scale ((float)(IMG_H-1)/(NEW_H-1))

#ifdef mAP
	//get image ID (according to COCO val2014 dataset)
	static int get_coco_image_id(char *filename) {
		char *p = strrchr(filename, '/');
		//char *c = strrchr(filename, '_');
		//if(c) p = c;
		return atoi(p+1);
	}
	static int coco_ids[] = {1,2,3,4,5,6,7,8,9,10,11,13,14,15,16,17,18,19,20,21,22,23,24,25,27,28,31,32,33,34,35,36,37,38,39,40,41,42,43,44,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,67,70,72,73,74,75,76,77,78,79,80,81,82,84,85,86,87,88,89,90};
#endif

#ifdef EIGHT_BITS

int16_t add_and_check_overflow(int16_t a, int16_t b){
	int16_t res = a + b;
	int sign_a = (a>>15)&&1, sign_b = (b>>15)&&1, sign_res = (res>>15)&&1;
	
	if( !(sign_a^sign_b) ){ // Same sign
		//printf("Same sign. There could be overflow\n");
		if( sign_res ^ sign_a ){ // res "flipped"
			printf("Overflow: sign_res=%d, sign_a=%d\n", sign_res, sign_a);
			res = sign_a ? 0x8000 : 0x7FFF;
		}
	}
	
	return res;
}

FILE *results;
#define conv_out_filename "../yolov3-tiny_8-bit_conv.hex"

//Constants for bounding boxes
#ifdef mAP
#define threshold ((int16_t)(((float)0.005)*((int32_t)1<<5))) //Q3.5
#else
	#define threshold ((int16_t)(((float)0.5)*((int32_t)1<<5))) //Q3.5
#endif
	#define nms_threshold ((int8_t)(((float)0.45)*((int32_t)1<<5))) //Q3.5
	#define yolo1_div ((int16_t)(((float)1/LAYER_17_W)*((int32_t)1<<15))) //Q1.15
	#define yolo2_div ((int16_t)(((float)1/LAYER_24_W)*((int32_t)1<<15))) //Q1.15
	#define x_scales ((int16_t)(((float)YOLO_INPUT/NEW_W)*((int32_t)1<<14))) //Q2.14
	#define x_bias ((int16_t)(((float)(YOLO_INPUT-NEW_W)/(NEW_W*2))*((int32_t)1<<13))) //Q3.13
	#define y_scales ((int16_t)(((float)YOLO_INPUT/NEW_H)*((int32_t)1<<14))) //Q2.14
	#define y_bias ((int16_t)(((float)(YOLO_INPUT-NEW_H)/(NEW_H*2))*((int32_t)1<<13))) //Q3.13
	#define w_scales ((int16_t)(((float)1/NEW_W)*((int32_t)1<<15))) //Q1.15
	#define h_scales ((int16_t)(((float)1/NEW_H)*((int32_t)1<<15))) //Q1.15
	#define c3 ((int16_t)0x0AAA) // pow(2,-3)+pow(2,-5)+pow(2,-7)+pow(2,-9)+pow(2,-11)+pow(2,-13) in Q2.14
	#define c4 ((int16_t)0x02C0) // pow(2,-5)+pow(2,-7)+pow(2,-8) in Q2.14
	//#define c3 ((int8_t)0x0A) // 2**(-3)+2**(-5) in Q2.6
	//#define c4 ((int8_t)0x02) // 2**-5 in Q2.6
	#define box_width 3
	#define label_height 20
	uint8_t nboxes = 0;

	//weights and data base address pointers
	int8_t * fp_weights;
	int8_t * fp_data;
	int8_t * fp_image;
	uint8_t * fp_labels;
	#ifdef GEMM
		int8_t * fp_gemm_in;
		int16_t * fp_gemm_out;
	#endif

	//weights and data updatable pointers
	unsigned int weight_pos = 0, data_pos = 0;

	//define base adress of weights and data pointers
	void define_memory_regions() {

		//image
		fp_image = (int8_t *) DATA_BASE_ADDRESS;
		
		//data
		fp_data = (int8_t *) (DATA_BASE_ADDRESS + IMAGE_INPUT);

		//weights
		fp_weights = (int8_t *) WEIGTHS_BASE_ADDRESS;
		
		//labels
		fp_labels = (uint8_t *) LABEL_BASE_ADDRESS;
		
	#ifdef GEMM
		fp_gemm_in = (int8_t *) GEMM_IN_BASE_ADDRESS;
		fp_gemm_out = (int16_t *) GEMM_OUT_BASE_ADDRESS;
	#endif
	}

	//reset DDR to zero
	void reset_DDR() {
		memset((void *) data, 0, TOTAL_DATA*sizeof(int8_t));
		int8_t * aux_p = (int8_t *) fp_data;
		aux_p += NETWORK_INPUT_AUX + NETWORK_INPUT + DATA_LAYER_1 + DATA_LAYER_2 + DATA_LAYER_3 + DATA_LAYER_4 + DATA_LAYER_5 + DATA_LAYER_6 + DATA_LAYER_7 + DATA_LAYER_8 + DATA_LAYER_10;
		int i;
		for(i = 0; i < DATA_LAYER_11; i++) aux_p[i] = 0x80; //min value in Q6.2 // previously 0x8000
	}

	//receive weights and data
	void receive_data(char **argv) {
		
		//local variables
		FILE *data;
		int i;
		
		//load input image
		if ((data = fopen(argv[1], "r+")) == NULL) {
			fprintf(stderr, "unable to open data file \n");
			exit(1);
		}
		
		//calculate dimensions
		//fseek(data, 12, SEEK_SET); //ignore image size info (3 int values)
		fread(&IMG_W, sizeof(int), 1, data);
		fread(&IMG_H, sizeof(int), 1, data);
		fread(&IMG_C, sizeof(int), 1, data);
		IMAGE_INPUT = IMG_W*IMG_H*IMG_C;
		if(IMG_W>=IMG_H) {
			NEW_W = YOLO_INPUT;
			NEW_H = ((IMG_H*NEW_W)/IMG_W);
			EXTRA_W = 0;
			EXTRA_H = ((NEW_W-NEW_H)/2);
		} else {
			NEW_H = YOLO_INPUT;
			NEW_W = ((IMG_W*NEW_H)/IMG_H);
			EXTRA_H = 0;
			EXTRA_W = ((NEW_H-NEW_W)/2);
		}
		NETWORK_INPUT_AUX = NEW_W*IMG_H*IMG_C;
		data_pos = NETWORK_INPUT_AUX;
		
		//define memory regions for pointers
		define_memory_regions();
			
		//Reset DDR to zero
		reset_DDR();
		
		//load image
		char * fp_image_char = (char *) fp_image;
		for(i = 0; i < IMAGE_INPUT; i++) fread(fp_image_char + i, sizeof(char), 1, data);
		fclose(data);
		
		//load weigths
		if ((data = fopen("../yolov3-tiny_batch-fp28bit.weights", "r+")) == NULL) {
			fprintf(stderr, "unable to open file yolov3-tiny_batch-8bit.weights\n");
			exit(1);
		}
		fread(fp_weights, sizeof(int8_t), TOTAL_WEIGTHS, data);
		fclose(data);
		
		/* Print 4th kernel of layer 1 */
		printf("Layer 1 -- 3th kernel\n");
		for(int i=0; i<LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*LAYER_1_C; i++)
			printf("%d\n", fp_weights[2*LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*LAYER_1_C+i]);
		/* Print 4th kernel of layer 1 */
		printf("Layer 1 -- 4th kernel\n");
		for(int i=0; i<LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*LAYER_1_C; i++)
			printf("%d\n", fp_weights[3*LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*LAYER_1_C+i]);

		//load labels
		if ((data = fopen("data/classes.bin", "r+")) == NULL) {
			fprintf(stderr, "unable to open file classes.bin\n");
			exit(1);
		}
		fread(fp_labels, sizeof(uint8_t), 81, data); //read label widths
		for(i = 0; i < 81; i++) fread(fp_labels+81+MAX_LABEL_SIZE*i, sizeof(uint8_t), fp_labels[i]*label_height, data); //read label data
		fclose(data);
		
	#ifdef INTERM_DATA
		//load intermediate results
		unsigned int pos_layer = NETWORK_INPUT;
		if ((data = fopen("../interm_data-fixed.network", "r+")) == NULL) {
			fprintf(stderr, "unable to open interm_data.network file \n");
			exit(1);
		}
			
		//layer 1
		for(i = 0; i < NTW_IN_NUM_KER; i++) {
			fread((fp_data + pos_layer), sizeof(int8_t), NTW_IN_W, data); //Read 1st line
			pos_layer += (NTW_IN_W*(NTW_IN_H+1));
			fread((fp_data + pos_layer), sizeof(int8_t), NTW_IN_W, data); //Read 2nd line
			pos_layer += NTW_IN_W;
		}
		
		//layer 2
		for(i = 0; i < LAYER_2_NUM_KER; i++) {
			fread((fp_data + pos_layer), sizeof(int8_t), (LAYER_3_W+2)*2, data); //Read 1st 2 lines
			pos_layer += (LAYER_3_W+2)*LAYER_3_H;
			fread((fp_data + pos_layer), sizeof(int8_t), (LAYER_3_W+2)*2, data); //Read 2nd 2 lines
			pos_layer += (LAYER_3_W+2)*2;
		}
		
		//layer 4
		pos_layer += DATA_LAYER_3;
		for(i = 0; i < LAYER_4_NUM_KER; i++) {
			fread((fp_data + pos_layer), sizeof(int8_t), (LAYER_5_W+2)*2, data); //Read 1st two lines
			pos_layer += (LAYER_5_W+2)*LAYER_5_H;
			fread((fp_data + pos_layer), sizeof(int8_t), (LAYER_5_W+2)*2, data); //Read 2nd two lines
			pos_layer += (LAYER_5_W+2)*2;
		}
		
		//layer 5
		for(i = 0; i < LAYER_5_NUM_KER; i++) {
			fread((fp_data + pos_layer), sizeof(int8_t), LAYER_5_W, data); //Read 1st line
			pos_layer += LAYER_5_W*(1+LAYER_5_H);
			fread((fp_data + pos_layer), sizeof(int8_t), LAYER_5_W, data); //Read 2nd line
			pos_layer += LAYER_5_W;
		}
		
		//layer 6
		for(i = 0; i < LAYER_6_NUM_KER; i++) {
			fread((fp_data + pos_layer), sizeof(int8_t), (LAYER_7_W+2)*2, data); //Read 1st 2 lines
			pos_layer += (LAYER_7_W+2)*LAYER_7_H;
			fread((fp_data + pos_layer), sizeof(int8_t), (LAYER_7_W+2)*2, data); //Read 2nd 2 lines
			pos_layer += (LAYER_7_W+2)*2;
		}
		
		//layer 8
		pos_layer += DATA_LAYER_7;
		for(i = 0; i < LAYER_8_NUM_KER; i++) {
			fread((fp_data + pos_layer), sizeof(int8_t), (LAYER_9_W+2)*2, data); //Read 1st rectangle
			pos_layer += (LAYER_9_W+2)*LAYER_9_H;
			fread((fp_data + pos_layer), sizeof(int8_t), (LAYER_9_W+2)*2, data); //Read 2nd rectangle
			pos_layer += (LAYER_9_W+2)*2;
		}
		
		//layer 9
		pos_layer += DATA_LAYER_10 + DATA_LAYER_11 + DATA_LAYER_12 + DATA_LAYER_13 + DATA_LAYER_14 + DATA_LAYER_15 + DATA_LAYER_16 + DATA_LAYER_17 + DATA_LAYER_19 + DATA_LAYER_20;
		for(i = 0; i < LAYER_9_NUM_KER; i++) {
			pos_layer += LAYER_9_W+2;
			fread((fp_data + pos_layer), sizeof(int8_t), (LAYER_9_W+2), data); //Read 1st rectangle
			pos_layer += (LAYER_9_W+2)*(1+LAYER_9_H);
			fread((fp_data + pos_layer), sizeof(int8_t), (LAYER_9_W+2), data); //Read 1st rectangle
			pos_layer += (LAYER_9_W+2)*2;
		}	
		fclose(data);
	#endif
	}

	//fill part of 416x416 region of resized image with grey (0.5 = 0x40 in Q1.7)
	void fill_grey() {
		int i, j, k;
		for(i = 0; i < NTW_IN_C; i++)
		#ifdef INTERM_DATA
			for(j = 0; j < NTW_IN_H+2; j++)
		#else
			for(j = 0; j < NTW_IN_H; j++)
		#endif
				for(k = 0; k < NTW_IN_W; k++) 
					fp_data[i*(NTW_IN_W+2)*(NTW_IN_H+2) + (j+GREY_PADD)*(NTW_IN_W+2) + (k+1) + NETWORK_INPUT_AUX] = 0x40;
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
			//ix
			ix[2*i] = val_i;
			ix[2*i+1] = val_i + 1;
			//dx
			dx[2*i] = (int8_t)((1-val_d)*((int8_t)1<<6)); //Q2.6
			dx[2*i+1] = (int8_t)(val_d*((int8_t)1<<6)); //Q2.6
		}
		
		//loop to initialize iy and dy
		for(i = 0; i < NEW_H; i++) {
			val = i*h_scale;
			iy[i] = (int) val;
			val_d = val - iy[i];
			//dy
			dy[2*i] = (int8_t)((1-val_d)*((int8_t)1<<6)); //Q2.6
			dy[2*i+1] = (int8_t)(val_d*((int8_t)1<<6)); //Q2.6
		}
	}
	
	//width resize
	void width_resize() {
		
		//local variables
		int16_t r, c, k;
		int16_t mul;
		
		//Width reduction
		for(r = 0; r < IMG_H; r++) {
			for(c = 0; c < NEW_W; c++) {
				for(k = 0; k < NTW_IN_C; k++) {
					mul = (int16_t)((int16_t)dx[2*c]*(int16_t)((uint8_t) fp_image[k*IMG_W*IMG_H + r*IMG_W + ix[2*c]])); //Q2.6 * Q0.8 = Q2.14
					mul = add_and_check_overflow(mul, (int16_t)((int16_t)dx[2*c+1]*(int16_t)((uint8_t) fp_image[k*IMG_W*IMG_H + r*IMG_W + ix[2*c+1]]))); //Q2.6 * Q0.8 = Q2.14
					//fp_data[k*NEW_W*IMG_H + r*NEW_W + c] = (int16_t) (mul >> 7); //Q10.22 to Q1.15
					if((mul>>7)<(int8_t)0x7F){
						if((mul>>7)>(int8_t)0x80)
							fp_data[k*NEW_W*IMG_H + r*NEW_W + c] = (int8_t) (mul >> 7); //Q2.14 to Q1.7
						else
							fp_data[k*NEW_W*IMG_H + r*NEW_W + c] = (int8_t) 0x80;
					} else {
						fp_data[k*NEW_W*IMG_H + r*NEW_W + c] = (int8_t) 0x7F;
					}
					//printf("%d\n", (int8_t) (mul>>7));
					//if (mul>>7 <0) exit(1);
				}
			}
		}
	}
	
	//height resize
	void height_resize() {

		//local variables
		int16_t r, c, k;
		int16_t mul;
		
		//Height reduction	
		for(r = 0; r < NEW_H; r++) {
			for(c = 0; c < NEW_W; c++) {
				for(k = 0; k < NTW_IN_C; k++) {
					mul = (int16_t)((int16_t)dy[2*r]*(int16_t)fp_data[k*NEW_W*IMG_H + iy[r]*NEW_W + c]); //Q2.6 * Q1.7 = Q3.13
					mul = add_and_check_overflow(mul, (int16_t)((int16_t)dy[2*r+1]*(int16_t)fp_data[k*NEW_W*IMG_H + (iy[r]+1)*NEW_W + c])); //Q2.6 * Q1.7 = Q3.13
					//fp_data[k*(NTW_IN_W+2)*(NTW_IN_H+2) + (r+GREY_PADD)*(NTW_IN_W+2) + (c+1) + EXTRA_W + ((NTW_IN_W+2)*EXTRA_H) + NETWORK_INPUT_AUX] = (int16_t)(mul >> 14); //Q3.29 to Q1.15
					fp_data[k*(NTW_IN_W+2)*(NTW_IN_H+2) + (r+GREY_PADD)*(NTW_IN_W+2) + (c+1) + EXTRA_W + ((NTW_IN_W+2)*EXTRA_H) + NETWORK_INPUT_AUX] = (int8_t)(mul >> 6); //Q3.13 to Q1.7
					//printf("%d\n", (int8_t)(mul>>6));
				}
			}
		}
	}

		//perform upsample layer
	void upsample_layer(int w, int num_ker) {

		//locate data pointers
		int8_t * in_d_pos = (int8_t *) fp_data + data_pos;
		int8_t * out_d_pos = (int8_t *) in_d_pos + w*w*num_ker;
		
		//local variables
		int i, j, k, l, m, output_w = w*2+2;
		int16_t val, val2;
		
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
		return val_out; //Q8.8
	}
	
	//perform convolutional layer
	void conv_layer(int w, int h, int c, int num_ker, int ker_size, int pad, int batch_norm, int nextPadding, int nextStride, int ignorePadding, unsigned int new_output_pos, unsigned int offset, int shift, int b_shift) {

		printf("output_pos: %d\n", new_output_pos);
		item_count=0;

		//adjust shift parameters
		shift-=8;
		b_shift-=8;

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
		int8_t * w_pos;
		w_pos = (int8_t *) fp_weights + weight_pos;
		int8_t * bias_pos = (int8_t *) fp_weights + weight_pos + num_ker*ker_size*ker_size*c;
		int8_t * in_d_pos = (int8_t *) fp_data + data_pos;
		int8_t * out_d_pos;
		if(new_output_pos != 0) out_d_pos = (int8_t *) fp_data + new_output_pos; else out_d_pos = (int8_t *) in_d_pos + pos_delta;
		
		printf("Bias and shift:\n");
		for(int i=0;i<num_ker;i++)
			printf("\t%d -> %d\n", bias_pos[i], ((int16_t) bias_pos[i]<<b_shift));
		printf("\n");
		//local variables
		int i, j, k, l, m, n;
		unsigned int output_pos;
		
	#ifdef GEMM
		
		int16_t val_16;
		
		//Transformation
		for(l = 0; l < c; l++)  						//Number of channels
			for(m = 0; m < ker_size; m++) 				//Kernel size
				for(n = 0; n < ker_size; n++) 
					for(j = 0; j < h; j++)   			//Output map size
						for(k = 0; k < w; k++) 
							fp_gemm_in[l*ker_size*ker_size*h*w + m*ker_size*h*w + n*h*w + j*w + k] = in_d_pos[(j+ignorePadding)*(w+2*pad) + (k+ignorePadding) + l*(w+2*pad)*new_h + m*(w+2*pad) + n];
						
		//Convolution
		for(i = 0; i < num_ker; i++) {
			for(j = 0; j < c*ker_size*ker_size; j++) {
				for(k = 0; k < w*h; k++) { 
					if(j == 0) fp_gemm_out[i*w*h + k] = bias_pos[i] << b_shift;
					fp_gemm_out[i*w*h + k] += (int16_t)((int16_t)w_pos[i*c*ker_size*ker_size + j]*(int16_t)fp_gemm_in[j*w*h + k]);
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
					
					//activation function
					val_16 = fp_gemm_out[i*w*h + j*w + k];
					if(batch_norm) {
						//leaky
						if(val_16 < 0) val_16 = (val_16 >> 4) + (val_16 >> 5) + (val_16 >> 7);						 
					} else {
						//sigmoid
						if(i != 2 && i != 3 && i != 87 && i != 88 && i != 172 && i != 173) val_16 = sigmoid(val_16);
					}
					
					//store results
					out_d_pos[output_pos] = (int8_t) (val_16 >> shift);
				}
			}
		}
		
	#else
		
		//local variables
		unsigned int output_pos2, output_pos3, output_pos4;
		int8_t op1, op2;
		int16_t acc, acc2, acc3, acc4, mul;
		//perform convolution
		for(i = 0; i < num_ker; i+=4) {								//Number of kernels
			for(j = 0; j < h; j++) {   								//Output map size
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
					acc = ((int16_t)bias_pos[i]) << b_shift;
					acc2 = ((int16_t)bias_pos[i+1]) << b_shift;
					acc3 = ((int16_t)bias_pos[i+2]) << b_shift;
					acc4 = ((int16_t)bias_pos[i+3]) << b_shift;
					if(print_vals) printf("b_shift: %d\tbias: %x\t%x\t%x\t%x\n", b_shift, acc, acc2, acc3, acc4);
					/*
					if(acc*bias_pos[i]<0 || acc2*bias_pos[i+1]<0 || acc3*bias_pos[i+2]<0 || acc4*bias_pos[i+3]<0){
						printf("ERROR: Bias overflow\n%d read as %d\n%d read as %d\n%d read as %d\n%d read as %d\n", bias_pos[i], acc, bias_pos[i+1], acc2, bias_pos[i+2], acc3, bias_pos[i+3], acc4);
						exit(1);
					}
					*/
					for(m = 0; m < ker_size; m++) {				//Kernel size
						for(n = 0; n < ker_size; n++) {
							for(l = 0; l < c; l++) { 			//Number of channels
								op1 = in_d_pos[(j+ignorePadding)*(w+2*pad) + (k+ignorePadding) + l*(w+2*pad)*new_h + m*(w+2*pad) + n];
								if(print_vals) printf("\nPixel = %d\nweights:\n", op1);
								op2 = w_pos[i*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n];
								//printf("pixel: %x\tweights: %x\t", op1, op2);
								mul = (int16_t)((int16_t)op1*(int16_t)op2);
								acc = add_and_check_overflow(acc, mul);
								if(print_vals) printf("%d\t", op2);
								op2 = w_pos[(i+1)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n];
								if(print_vals) printf("%d\t", op2);
								//printf("%x\t", op2);
								mul = (int16_t)((int16_t)op1*(int16_t)op2);
								acc2 = add_and_check_overflow(acc2, mul);	
								op2 = w_pos[(i+2)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n];
								if(print_vals) printf("%d\t", op2);
								//printf("%x\t", op2);
								mul = (int16_t)((int16_t)op1*(int16_t)op2);
								acc3 = add_and_check_overflow(acc3, mul);
								op2 = w_pos[(i+3)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n];
								if(print_vals) printf("%d\n", op2);
								//printf("%x\n", op2);
								mul = (int16_t)((int16_t)op1*(int16_t)op2);
								acc4 = add_and_check_overflow(acc4, mul);								
							}
						}
					}
					
										
					//activation function
					if(batch_norm) {
						//leaky
						if(acc < 0) acc = (acc >> 4) + (acc >> 5) + (acc >> 7);						
						if(acc2 < 0) acc2 = (acc2 >> 4) + (acc2 >> 5) + (acc2 >> 7);					
						if(acc3 < 0) acc3 = (acc3 >> 4) + (acc3 >> 5) + (acc3 >> 7);						
						if(acc4 < 0) acc4 = (acc4 >> 4) + (acc4 >> 5) + (acc4 >> 7); 
					} else {
						//sigmoid
						if(i != 88 && i != 172) acc = sigmoid(acc);
						if(i != 172) acc2 = sigmoid(acc2);
						if(i != 0) acc3 = sigmoid(acc3);
						if(i != 0 && i != 84) acc4 = sigmoid(acc4);
					}
					
					if(print_vals) printf("shift=%d\n%d\t%d\t%d\t%d\n", shift, acc, acc2, acc3, acc4);				
					
					//store results
					
					if((acc>>shift) > 0x7F){
						out_d_pos[output_pos] = (int8_t) 0x7F;
						printf("Saturated pixel: %x to %x\n", (acc>>shift), out_d_pos[output_pos]);
					}
					else if((acc>>shift) < (int8_t) 0x80){
						out_d_pos[output_pos] = (int8_t) 0x80;
						printf("Saturated pixel: %x to %x\n", (acc>>shift), out_d_pos[output_pos]);
					}
					else{
						out_d_pos[output_pos] = (int8_t) (acc>>shift);
					}

					if((acc2>>shift) > 0x7F){
						out_d_pos[output_pos2] = (int8_t) 0x7F;
						printf("Saturated pixel: %x to %x\n", (acc2>>shift), out_d_pos[output_pos2]);
					}
					else if((acc2>>shift) < (int8_t) 0x80){
						out_d_pos[output_pos2] = (int8_t) 0x80;
						printf("Saturated pixel: %x to %x\n", (acc2>>shift), out_d_pos[output_pos2]);
					}
					else{
						out_d_pos[output_pos2] = (int8_t) (acc2>>shift);
					}

					if((acc3>>shift) > 0x7F){
						out_d_pos[output_pos3] = (int8_t) 0x7F;
						printf("Saturated pixel: %x to %x\n", (acc3>>shift), out_d_pos[output_pos3]);
					}
					else if((acc3>>shift) < (int8_t) 0x80){
						out_d_pos[output_pos3] = (int8_t) 0x80;
						printf("Saturated pixel: %x to %x\n", (acc3>>shift), out_d_pos[output_pos3]);
					}
					else{
						out_d_pos[output_pos3] = (int8_t) (acc3>>shift);
					}

					if((acc4>>shift) > 0x7F){
						out_d_pos[output_pos4] = (int8_t) 0x7F;
						printf("Saturated pixel: %x to %x\n", (acc4>>shift), out_d_pos[output_pos4]);
					}
					else if((acc4>>shift) < (int8_t) 0x80){
						out_d_pos[output_pos4] = (int8_t) 0x80;
						printf("Saturated pixel: %x to %x\n", (acc4>>shift), out_d_pos[output_pos4]);
					}
					else{
						out_d_pos[output_pos4] = (int8_t) (acc4>>shift);
					}
					

					
					out_d_pos[output_pos] = (int8_t) (acc>>shift);
					out_d_pos[output_pos2] = (int8_t) (acc2>>shift);
					out_d_pos[output_pos3] = (int8_t) (acc3>>shift);
					out_d_pos[output_pos4] = (int8_t) (acc4>>shift);
					

				//	printf("shift: %d\taccs: %d\t%d\t%d\t%d\n", shift, acc, acc2, acc3, acc4);
					
				//	printf("outputs: %d\t%d\t%d\t%d\n", out_d_pos[output_pos], out_d_pos[output_pos2], out_d_pos[output_pos3], out_d_pos[output_pos4]);

					int output_positions[4] = {output_pos,output_pos2,output_pos3,output_pos4};
					/*
					fwrite((out_d_pos+output_pos), sizeof(int8_t), 1, results);
					fwrite((out_d_pos+output_pos2), sizeof(int8_t), 1, results);
					fwrite((out_d_pos+output_pos3), sizeof(int8_t), 1, results);
					fwrite((out_d_pos+output_pos4), sizeof(int8_t), 1, results);
					*/
					int res;
					for (int item=0; item<4; item++){
						do{
							res = fwrite((int8_t*) (out_d_pos)+(output_positions[item]), sizeof(int8_t), 1, results);
							if(print_vals) printf("%d\t", *((out_d_pos)+(output_positions[item])));
						}while(!res);
						item_count++;
					}
					if(print_vals){printf("\n");exit(0);}
					
				}
			}
		}
		
	#endif //relative to `#ifdef GEMM`
		
		//update weights and data pointers
		weight_pos += num_ker + num_ker*c*ker_size*ker_size;
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
		int8_t * in_d_pos = (int8_t *) fp_data + data_pos;
		int8_t * out_d_pos;
		if(new_output_pos != 0) out_d_pos = (int8_t *) fp_data + new_output_pos; 
		else out_d_pos = (int8_t *) in_d_pos + (w+1-downsample+2*ignorePadding)*new_h*num_ker;
		
		//local variables
		int i, j, k, l, m, new_w = w+1-downsample+2*ignorePadding;
		int8_t max, max2, max3, max4, val, val2, val3, val4;
		
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
			
	//polynomial approximation of exponential function
	int16_t exp_fnc(int8_t val) {
		int16_t val_16, exp_val_fixed;
		int32_t val_32;
		exp_val_fixed = ((int16_t) val<<8) + 0x2000; //1+w -> Q3.13 (val is therefore converted from Q3.5 to Q3.13)
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
	
	//apply sigmoid to input FM
	void yolo_layer(int w, int16_t xy_div, int first_yolo, unsigned int in_pos, unsigned int out_pos) {
		
		//locate data pointers
		int8_t * input = (int8_t *) fp_data + in_pos;
		int8_t * output = (int8_t *) fp_data + out_pos;
		
		//local variables
		int16_t i, j, k, m;
		int16_t val_16, obj_score, pred_score;
		int32_t val_32;
		int16_t yolo_bias[12] = {0x0280, 0x0380, 0x05C0, 0x06C0, 0x0940, 0x0E80, 0x1440, 0x1480, 0x21C0, 0x2A40, 0x5600, 0x4FC0}; //Q10.6
				
		//loops to go through yolo layer output
		for(i = 0; i < 3; i++) {
			for(j = 0; j < w; j++) {
				for(k = 0; k < w; k++) {
					
					//sigmoid of objectness score
					obj_score = (int16_t)(input[(4+85*i)*w*w + j*w + k]<<8); //Q3.13
					
					//check if objectness score is above threshold
					if(obj_score > threshold) {
																							
						//Calculate x
						val_16 = (int16_t)(input[85*i*w*w + j*w + k]<<8); //Q3.13
						val_32 = (int32_t)((int32_t)(val_16 + (k<<13))*(int32_t)xy_div); //Q3.13 *Q1.15 = Q4.28
						val_16 = (int16_t)(val_32 >> 15); //Q4.28 to Q3.13
						val_32 = (int32_t)((int32_t)val_16*(int32_t)x_scales); //Q3.13 * Q2.14 = Q5.27
						val_16 = (int16_t)(val_32 >> 14); //Q5.27 to Q3.13
						val_16 -= (int16_t)x_bias; //Q3.13
						output[84*nboxes] = (int8_t)(val_16>>8); //x in Q3.5 
												
						//Calculate y
						val_16 = (int16_t)(input[(1+85*i)*w*w + j*w + k]<<8); //Q3.13
						val_32 = (int32_t)((int32_t)(val_16 + (j<<13))*(int32_t)xy_div); //Q3.13 *Q1.15 = Q4.28
						val_16 = (int16_t)(val_32 >> 15); //Q4.28 to Q3.13
						val_32 = (int32_t)((int32_t)val_16*(int32_t)y_scales); //Q3.13 * Q2.14 = Q5.27
						val_16 = (int16_t)(val_32 >> 14); //Q5.27 to Q3.13
						val_16 -= (int16_t)y_bias; //Q3.13
						output[84*nboxes+1] = (int8_t)(val_16>>8); //y in Q3.5

						//Calculate w
						val_16 = (exp_fnc(input[(2+85*i)*w*w + j*w + k])); //Q3.13
						val_32 = (int32_t)((int32_t)val_16*(int32_t)w_scales); //Q3.13 * Q1.15 = Q4.28
						val_16 = (int16_t)(val_32 >> 15); //Q4.28 to Q3.13
						//val_32 = (int32_t)((int32_t)val_16*(int32_t)yolo_bias[2*(i+3*first_yolo)]); //Q2.14 * Q10.6 = Q12.20
						val_32 = (int32_t)((int32_t)val_16*(int32_t)yolo_bias[2*(i+(1-first_yolo)+3*first_yolo)]); //Q3.13 * Q10.6 = Q13.19 -> mask 1,2,3
						val_16 = (int16_t)(val_32 >> 6); //Q13.19 to Q3.13
						output[84*nboxes+2] = (int8_t)(val_16>>8); //w in Q3.5
						
						//Calculate h
						val_16 = (exp_fnc(input[(3+85*i)*w*w + j*w + k])); //Q3.13
						val_32 = (int32_t)((int32_t)val_16*(int32_t)h_scales); //Q3.13 * Q1.15 = Q4.28
						val_16 = (int16_t)(val_32 >> 15); //Q4.28 to Q3.13
						//val_32 = (int32_t)((int32_t)val_16*(int32_t)yolo_bias[2*(i+3*first_yolo)+1]); //Q2.14 * Q10.6 = Q12.20
						val_32 = (int32_t)((int32_t)val_16*(int32_t)yolo_bias[2*(i+(1-first_yolo)+3*first_yolo)+1]); //Q3.13 * Q10.6 = Q13.19 -> mask 1,2,3
						val_16 = (int16_t)(val_32 >> 6); //Q13.19 to Q3.13
						output[84*nboxes+3] = (int8_t)(val_16>>8); //h in Q3.5
												
						//Calculate probability scores
						for(m = 0; m < 80; m++) {
							val_16 = (int16_t)(input[(5+m+85*i)*w*w + j*w + k]<<8); //Q3.13
							val_32 = (int32_t)((int32_t)val_16*(int32_t)obj_score); //Q3.13 * Q3.13 = Q6.26
							pred_score = (int16_t)(val_32 >> 13); //Q6.26 to Q3.13
							if(pred_score <= threshold) pred_score = 0; //Q3.13
							output[84*nboxes+4+m] = (int8_t)(pred_score>>8); // prediction scores
						}
						
						//Update number of candidate boxes
						nboxes++;
					}
				}
			}
		}
	}

	//Calculate overlapp between 2 boxes
	int8_t overlap(int8_t x1, int8_t w1, int8_t x2, int8_t w2) {
		int8_t l1, l2, left, r1, r2, right;
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
		int8_t * in_d_pos = (int8_t *) fp_data + pos;
		uint8_t * out_d_pos = (uint8_t *) fp_data + (pos + 84*nboxes)*2;
		
		//Local variables
		int i, j, k, l;
		int obj_cnt;
		int8_t w, h, b_union, b_iou;
		int8_t x1, y1, w1, h1, x2, y2, w2, h2;
		int16_t mul_16, b_inter;
		
		//Loop to go through classes from candidate boxes
		for(i = 0; i < 80; i++) {
			
			//Count number of candidate boxes for given class
			obj_cnt = 0;
			for(j = 0; j < nboxes; j++) {
				if(in_d_pos[84*j+4+i] != 0) {
				
					//Store box ID in descending order of prob score
					if(obj_cnt == 0) out_d_pos[0] = j;
					else {
						
						//Search for position of new box ID
						for(k = 0; k < obj_cnt; k++)
							if(in_d_pos[84*j+4+i] > in_d_pos[84*out_d_pos[k]+4+i])
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
					if(in_d_pos[84*out_d_pos[j]+4+i] == 0) continue;
					for(k = j+1; k < obj_cnt; k++) {
						
						//Get boxes coordinates
						x1 = in_d_pos[84*out_d_pos[j]];
						y1 = in_d_pos[84*out_d_pos[j]+1];
						w1 = in_d_pos[84*out_d_pos[j]+2];
						h1 = in_d_pos[84*out_d_pos[j]+3];
						x2 = in_d_pos[84*out_d_pos[k]];
						y2 = in_d_pos[84*out_d_pos[k]+1];
						w2 = in_d_pos[84*out_d_pos[k]+2];
						h2 = in_d_pos[84*out_d_pos[k]+3];
						
						//Calculate IoU (intersection over union)
						w = overlap(x1, w1, x2, w2); //Q3.13
						h = overlap(y1, h1, y2, h2); //Q3.13
						if(w > 0 && h > 0) {
							b_inter = (int16_t)((int16_t)w*(int16_t)h); //Q3.5 * Q3.5 = Q6.10
							mul_16 = (int16_t)((int16_t)w1*(int16_t)h1); //Q3.5 * Q3.5 = Q6.10
							b_union = (int8_t)(mul_16 >> 5); //w1*h1 -> Q6.10 to Q3.5
							mul_16 = (int16_t)((int16_t)w2*(int16_t)h2); ///Q3.5 * Q3.5 = Q6.10
							b_union += (int8_t)(mul_16 >> 5); //w1*h1+w2*h2 -> Q6.10 to Q3.5
							b_union -= (int8_t)(b_inter >> 5); //w1*h1+w2*h2-inter -> Q6.10 to Q3.5
							if(b_union==0){
								printf("-- Avoided division by zero\n");
								in_d_pos[84*out_d_pos[k]+4+i]=0;
							}else{
								b_iou = (int8_t)((int16_t)b_inter/(int16_t)b_union); //Q6.10 / Q3.5 = Q3.5						
								if(b_iou > nms_threshold) in_d_pos[84*out_d_pos[k]+4+i]	= 0;
							}
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
		for(l = 0; l < label_height && (l+top_width) < IMG_H; l++){
			for(k = 0; k < label_w && (k+left+previous_w) < IMG_W; k++){
				//Q8.0*Q8.0=Q16.0 to Q8.0 -> red
				fp_image[(l+top_width)*IMG_W+(k+left+previous_w)] = ((uint16_t)((uint16_t)r*(uint16_t)fp_labels[(81+MAX_LABEL_SIZE*j)+l*label_w+k])) >> 8;				
				//green
				fp_image[IMG_W*IMG_H+(l+top_width)*IMG_W+(k+left+previous_w)] = ((uint16_t)((uint16_t)g *(uint16_t)fp_labels[(81+MAX_LABEL_SIZE*j)+l*label_w+k])) >> 8;
				//blue
				fp_image[2*IMG_W*IMG_H+(l+top_width)*IMG_W+(k+left+previous_w)] = ((uint16_t)((uint16_t)b *(uint16_t)fp_labels[(81+MAX_LABEL_SIZE*j)+l*label_w+k])) >> 8;
			}
		}
	}

	//Draw detections (bounding boxes and class labels) in input image
	void draw_detections(unsigned int pos) {
		
		//locate data pointers
		int8_t * in_d_pos = (int8_t *) fp_data + pos;
		
		//local variables
		int i, j, k;
		uint8_t colors[6][3] = { {255,0,255}, {0,0,255},{0,255,255},{0,255,0},{255,255,0},{255,0,0} }; //Q8.0
		uint8_t ratio, red, green, blue, label_w;
		uint16_t mul_u16;
		int32_t mul_32;
		int offset, ratio_min, ratio_max;
		int left, right, top, bot, top_width, previous_w;
		
		//Check valid detections
		for(i = 0; i < nboxes; i++) {
			
			//Find detected classes
			previous_w = 0;
			for(j = 0; j < 80; j++) {
				if(in_d_pos[84*i+4+j] != 0) {
								
					//Check if this was the first class detected for given box
					if(previous_w == 0) {
			
						//Randomly pick rgb colors for the box
						offset = j*123457 % 80;
						mul_u16 = (uint16_t)((uint16_t)offset*(uint16_t)((uint8_t)0x10)); //Q8.0 *Q0.8 = Q8.8
						ratio = (uint8_t)(mul_u16>>2); //Q8.8 to Q2.6
						ratio_min = (ratio >> 6);
						ratio_max = ratio_min + 1;
						ratio = ratio & 0x3F; //Q2.6
						mul_u16 = (uint16_t)((uint16_t)(0x40-ratio)*(uint16_t)colors[ratio_min][2]); //Q2.6 *Q8.0 = Q10.6
						mul_u16 += (uint16_t)((uint16_t)ratio*(uint16_t)colors[ratio_max][2]); //Q2.6 *Q8.0 = Q10.6
						red = (mul_u16 >> 6); //Q10.6 to Q8.0
						mul_u16 = (uint16_t)((uint16_t)(0x40-ratio)*(uint16_t)colors[ratio_min][1]); //Q2.6 *Q8.0 = Q10.6
						mul_u16 += (uint16_t)((uint16_t)ratio*(uint16_t)colors[ratio_max][1]); //Q2.6 *Q8.0 = Q10.6
						green = (mul_u16 >> 6); //Q10.6 to Q8.0
						mul_u16 = (uint16_t)((uint16_t)(0x40-ratio)*(uint16_t)colors[ratio_min][0]); //Q2.6 *Q8.0 = Q10.6
						mul_u16 += (uint16_t)((uint16_t)ratio*(uint16_t)colors[ratio_max][0]); //Q2.6 *Q8.0 = Q10.6
						blue = (mul_u16 >> 6); //Q10.6 to Q8.0
						
						//Calculate box coordinates in image frame				
						mul_u16 = (int16_t)((in_d_pos[84*i] - (in_d_pos[84*i+2]>>1))<<8); //Q3.13
						mul_32 = (int32_t)((int32_t)mul_u16 * (int32_t)IMG_W); //Q3.13 * Q16.0 = Q19.13
						left = (mul_32 >> 13);					
						mul_u16 = (int16_t)((in_d_pos[84*i] + (in_d_pos[84*i+2]>>1))<<8); //Q3.13
						mul_32 = (int32_t)((int32_t)mul_u16 * (int32_t)IMG_W); //Q3.13 * Q16.0 = Q19.13
						right = (mul_32 >> 13);
						mul_u16 = (int16_t)((in_d_pos[84*i+1] - (in_d_pos[84*i+3]>>1))<<8); //Q3.13
						mul_32 = (int32_t)((int32_t)mul_u16 * (int32_t)IMG_H); //Q3.13 * Q16.0 = Q19.13
						top = (mul_32 >> 13);
						mul_u16 = (int16_t)((in_d_pos[84*i+1] + (in_d_pos[84*i+3]>>1))<<8); //Q3.13
						mul_32 = (int32_t)((int32_t)mul_u16 * (int32_t)IMG_H); //Q3.13 * Q16.0 = Q19.13
						bot = (mul_32 >> 13);					

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
		uint16_t pred_16;
		const char *class_names[80] = {"person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", 
							"stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", 
							"backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", 
							"baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", 
							"banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed", "dining table", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};
		for(i = 0; i < nboxes; i++) {
			for(j = 0; j < 80; j++) {
				if(fp_data[box_pos+84*i+4+j] != 0) {
					pred_16 = (uint16_t)((uint16_t)fp_data[box_pos+84*i+4+j]*(uint16_t)100); //Q3.5 * Q8.0 = Q11.5
					if( (pred_16&0x1FFF) > 0x1000) printf("\n%s: %d%%", class_names[j], (pred_16>>5)+1);
					else printf("\n%s: %d%%", class_names[j], (pred_16>>5));
				}
			}
		}
		printf("\n");	
	}

	//send detection results back	
	void send_data(char **argv) {

		//file pointer
		FILE *data;
		
		//Find filename
		char * filename_full = argv[1], * ch_p = strtok(filename_full, "/."), *filename = ch_p;
		while(ch_p != NULL) {
			ch_p = strtok(NULL, "/.");
			if(strcmp(ch_p,"bin")==0) break;
			filename = ch_p;
		}
		char output_filename[100] = "../output_data/";
		strcat(output_filename, filename);
		strcat(output_filename, ".bin"); 
		
		//create new image
		if ((data = fopen(output_filename, "wb")) == NULL) {
			fprintf(stderr, "unable to open result file\n");
			exit(1);
		}
		int w, h, c, i;
		char * fp_image_char = (char *) fp_image;
		w = IMG_W;
		h = IMG_H;
		c = IMG_C;
		fwrite(&w, sizeof(int), 1, data);
		fwrite(&h, sizeof(int), 1, data);
		fwrite(&c, sizeof(int), 1, data);
		for(i = 0; i < IMAGE_INPUT; i++) fwrite(fp_image_char + i, sizeof(char), 1, data);
		fclose(data);
	}

	#ifdef mAP
		//print in json file
		static void print_cocos(FILE *fp, char *image_path, unsigned int pos) {
			
			//local variables
			int16_t * out_d_pos = (int16_t *) fp_data + pos;
			int i, j;
			int image_id = get_coco_image_id(image_path);
			float xmax, ymax;
			float bx, by, bw, bh;
			float x, y, w, h, prob;
			
			//loop to output bounding boxes in .json file
			for(i = 0; i < nboxes; i++) {
				
				//Convert from Q3.13 to float
				x = ((float)out_d_pos[84*i]/((int16_t)1<<13))*IMG_W;
				y = ((float)out_d_pos[84*i+1]/((int16_t)1<<13))*IMG_H;
				w = ((float)out_d_pos[84*i+2]/((int16_t)1<<13))*IMG_W;
				h = ((float)out_d_pos[84*i+3]/((int16_t)1<<13))*IMG_H;
								
				//Calculate x,y coordinates
				bx = x - w/2.;
				xmax = x + w/2.;
				by = y - h/2.;
				ymax = y + h/2.;
							
				//Limit x,y coordinates
				if (bx < 0) bx = 0;
				if (by < 0) by = 0;
				if (xmax > IMG_W) xmax = IMG_W;
				if (ymax > IMG_H) ymax = IMG_H;
				
				//Calculate relative box coordinates
				bw = xmax - bx;
				bh = ymax - by;
				
				//Print to .json file
				for(j = 0; j < 80; ++j) {
					prob = (float)out_d_pos[84*i+4+j]/((int16_t)1<<14);
					if(prob) 
						fprintf(fp, "{\"image_id\":%d, \"category_id\":%d, \"bbox\":[%f, %f, %f, %f], \"score\":%f},\n", image_id, coco_ids[j], bx, by, bw, bh, prob);
				}
			}
		}
	#endif //relative to `#ifdef mAP`
	
#else // relative to `#ifdef EIGHT_BITS`

#ifdef FIXED

FILE *results;
#define conv_out_filename "../yolov3-tiny_16-bit_conv.hex"

	//Constants for bounding boxes
#ifdef mAP
#define threshold ((int16_t)(((float)0.005)*((int32_t)1<<13))) //Q3.13
#else
	#define threshold ((int16_t)(((float)0.5)*((int32_t)1<<13))) //Q3.13
#endif
	#define nms_threshold ((int16_t)(((float)0.45)*((int32_t)1<<13))) //Q3.13
	#define yolo1_div ((int16_t)(((float)1/LAYER_17_W)*((int32_t)1<<15))) //Q1.15
	#define yolo2_div ((int16_t)(((float)1/LAYER_24_W)*((int32_t)1<<15))) //Q1.15
	#define x_scales ((int16_t)(((float)YOLO_INPUT/NEW_W)*((int32_t)1<<14))) //Q2.14
	#define x_bias ((int16_t)(((float)(YOLO_INPUT-NEW_W)/(NEW_W*2))*((int32_t)1<<13))) //Q3.13
	#define y_scales ((int16_t)(((float)YOLO_INPUT/NEW_H)*((int32_t)1<<14))) //Q2.14
	#define y_bias ((int16_t)(((float)(YOLO_INPUT-NEW_H)/(NEW_H*2))*((int32_t)1<<13))) //Q3.13
	#define w_scales ((int16_t)(((float)1/NEW_W)*((int32_t)1<<15))) //Q1.15
	#define h_scales ((int16_t)(((float)1/NEW_H)*((int32_t)1<<15))) //Q1.15
	#define c3 ((int16_t)0x0AAA) // pow(2,-3)+pow(2,-5)+pow(2,-7)+pow(2,-9)+pow(2,-11)+pow(2,-13) in Q2.14
	#define c4 ((int16_t)0x02C0) // pow(2,-5)+pow(2,-7)+pow(2,-8) in Q2.14
	#define box_width 3
	#define label_height 20
	uint8_t nboxes = 0;

	//weights and data base address pointers
	int16_t * fp_weights;
	int16_t * fp_data;
	int16_t * fp_image;
	uint8_t * fp_labels;
	#ifdef GEMM
		int16_t * fp_gemm_in;
		int32_t * fp_gemm_out;
	#endif

	//weights and data updatable pointers
	unsigned int weight_pos = 0, data_pos = 0;

	//define base adress of weights and data pointers
	void define_memory_regions() {
		
		//image
		fp_image = (int16_t *) DATA_BASE_ADDRESS;
		
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

	//reset DDR to zero
	void reset_DDR() {
		memset((void *) data, 0, TOTAL_DATA*sizeof(int16_t));
		int16_t * aux_p = (int16_t *) fp_data;
		aux_p += NETWORK_INPUT_AUX + NETWORK_INPUT + DATA_LAYER_1 + DATA_LAYER_2 + DATA_LAYER_3 + DATA_LAYER_4 + DATA_LAYER_5 + DATA_LAYER_6 + DATA_LAYER_7 + DATA_LAYER_8 + DATA_LAYER_10;
		int i;
		for(i = 0; i < DATA_LAYER_11; i++) aux_p[i] = 0x8000; //min value in Q6.10
	}

	//receive weights and data
	void receive_data(char **argv) {
		
		//local variables
		FILE *data;
		int i;
		
		//load input image
		if ((data = fopen(argv[1], "r+")) == NULL) {
			fprintf(stderr, "unable to open data file \n");
			exit(1);
		}
		
		//calculate dimensions
		//fseek(data, 12, SEEK_SET); //ignore image size info (3 int values)
		fread(&IMG_W, sizeof(int), 1, data);
		fread(&IMG_H, sizeof(int), 1, data);
		fread(&IMG_C, sizeof(int), 1, data);
		IMAGE_INPUT = IMG_W*IMG_H*IMG_C;
		if(IMG_W>=IMG_H) {
			NEW_W = YOLO_INPUT;
			NEW_H = ((IMG_H*NEW_W)/IMG_W);
			EXTRA_W = 0;
			EXTRA_H = ((NEW_W-NEW_H)/2);
		} else {
			NEW_H = YOLO_INPUT;
			NEW_W = ((IMG_W*NEW_H)/IMG_H);
			EXTRA_H = 0;
			EXTRA_W = ((NEW_H-NEW_W)/2);
		}
		NETWORK_INPUT_AUX = NEW_W*IMG_H*IMG_C;
		data_pos = NETWORK_INPUT_AUX;
		
		//define memory regions for pointers
		define_memory_regions();
			
		//Reset DDR to zero
		reset_DDR();
		
		//load image
		char * fp_image_char = (char *) fp_image;
		for(i = 0; i < IMAGE_INPUT; i++) fread(fp_image_char + 2*i, sizeof(char), 1, data);
		fclose(data);
		
		//load weigths
		if ((data = fopen("../yolov3-tiny_batch-fixed__script.weights", "r+")) == NULL) {
			fprintf(stderr, "unable to open file yolov3-tiny_batch-fixed.weights\n");
			exit(1);
		}
		fread(fp_weights, sizeof(int16_t), TOTAL_WEIGTHS, data);
		fclose(data);

		//load labels
		if ((data = fopen("data/classes.bin", "r+")) == NULL) {
			fprintf(stderr, "unable to open file classes.bin\n");
			exit(1);
		}
		fread(fp_labels, sizeof(uint8_t), 81, data); //read label widths
		for(i = 0; i < 81; i++) fread(fp_labels+81+MAX_LABEL_SIZE*i, sizeof(uint8_t), fp_labels[i]*label_height, data); //read label data
		fclose(data);
		
	#ifdef INTERM_DATA
		//load intermediate results
		unsigned int pos_layer = NETWORK_INPUT;
		if ((data = fopen("../interm_data-fixed.network", "r+")) == NULL) {
			fprintf(stderr, "unable to open interm_data.network file \n");
			exit(1);
		}
			
		//layer 1
		for(i = 0; i < NTW_IN_NUM_KER; i++) {
			fread((fp_data + pos_layer), sizeof(int16_t), NTW_IN_W, data); //Read 1st line
			pos_layer += (NTW_IN_W*(NTW_IN_H+1));
			fread((fp_data + pos_layer), sizeof(int16_t), NTW_IN_W, data); //Read 2nd line
			pos_layer += NTW_IN_W;
		}
		
		//layer 2
		for(i = 0; i < LAYER_2_NUM_KER; i++) {
			fread((fp_data + pos_layer), sizeof(int16_t), (LAYER_3_W+2)*2, data); //Read 1st 2 lines
			pos_layer += (LAYER_3_W+2)*LAYER_3_H;
			fread((fp_data + pos_layer), sizeof(int16_t), (LAYER_3_W+2)*2, data); //Read 2nd 2 lines
			pos_layer += (LAYER_3_W+2)*2;
		}
		
		//layer 4
		pos_layer += DATA_LAYER_3;
		for(i = 0; i < LAYER_4_NUM_KER; i++) {
			fread((fp_data + pos_layer), sizeof(int16_t), (LAYER_5_W+2)*2, data); //Read 1st two lines
			pos_layer += (LAYER_5_W+2)*LAYER_5_H;
			fread((fp_data + pos_layer), sizeof(int16_t), (LAYER_5_W+2)*2, data); //Read 2nd two lines
			pos_layer += (LAYER_5_W+2)*2;
		}
		
		//layer 5
		for(i = 0; i < LAYER_5_NUM_KER; i++) {
			fread((fp_data + pos_layer), sizeof(int16_t), LAYER_5_W, data); //Read 1st line
			pos_layer += LAYER_5_W*(1+LAYER_5_H);
			fread((fp_data + pos_layer), sizeof(int16_t), LAYER_5_W, data); //Read 2nd line
			pos_layer += LAYER_5_W;
		}
		
		//layer 6
		for(i = 0; i < LAYER_6_NUM_KER; i++) {
			fread((fp_data + pos_layer), sizeof(int16_t), (LAYER_7_W+2)*2, data); //Read 1st 2 lines
			pos_layer += (LAYER_7_W+2)*LAYER_7_H;
			fread((fp_data + pos_layer), sizeof(int16_t), (LAYER_7_W+2)*2, data); //Read 2nd 2 lines
			pos_layer += (LAYER_7_W+2)*2;
		}
		
		//layer 8
		pos_layer += DATA_LAYER_7;
		for(i = 0; i < LAYER_8_NUM_KER; i++) {
			fread((fp_data + pos_layer), sizeof(int16_t), (LAYER_9_W+2)*2, data); //Read 1st rectangle
			pos_layer += (LAYER_9_W+2)*LAYER_9_H;
			fread((fp_data + pos_layer), sizeof(int16_t), (LAYER_9_W+2)*2, data); //Read 2nd rectangle
			pos_layer += (LAYER_9_W+2)*2;
		}
		
		//layer 9
		pos_layer += DATA_LAYER_10 + DATA_LAYER_11 + DATA_LAYER_12 + DATA_LAYER_13 + DATA_LAYER_14 + DATA_LAYER_15 + DATA_LAYER_16 + DATA_LAYER_17 + DATA_LAYER_19 + DATA_LAYER_20;
		for(i = 0; i < LAYER_9_NUM_KER; i++) {
			pos_layer += LAYER_9_W+2;
			fread((fp_data + pos_layer), sizeof(int16_t), (LAYER_9_W+2), data); //Read 1st rectangle
			pos_layer += (LAYER_9_W+2)*(1+LAYER_9_H);
			fread((fp_data + pos_layer), sizeof(int16_t), (LAYER_9_W+2), data); //Read 1st rectangle
			pos_layer += (LAYER_9_W+2)*2;
		}	
		fclose(data);
	#endif
	}

	//fill part of 416x316 region of resized image with grey (0.5 = 0x4000 in Q1.15)
	void fill_grey() {
		int i, j, k;
		for(i = 0; i < NTW_IN_C; i++)
		#ifdef INTERM_DATA
			for(j = 0; j < NTW_IN_H+2; j++)
		#else
			for(j = 0; j < NTW_IN_H; j++)
		#endif
				for(k = 0; k < NTW_IN_W; k++) 
					fp_data[i*(NTW_IN_W+2)*(NTW_IN_H+2) + (j+GREY_PADD)*(NTW_IN_W+2) + (k+1) + NETWORK_INPUT_AUX] = 0x4000;
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
			//ix
			ix[2*i] = val_i;
			ix[2*i+1] = val_i + 1;
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
	
	//width resize
	void width_resize() {
		
		//local variables
		int16_t r, c, k;
		int32_t mul;
		
		//Width reduction
		for(r = 0; r < IMG_H; r++) {
			for(c = 0; c < NEW_W; c++) {
				for(k = 0; k < NTW_IN_C; k++) {
					mul = (int32_t)((int32_t)dx[2*c]*(int32_t)(fp_image[k*IMG_W*IMG_H + r*IMG_W + ix[2*c]])); //Q2.14 * Q8.8 = Q10.22
					mul += (int32_t)((int32_t)dx[2*c+1]*(int32_t)(fp_image[k*IMG_W*IMG_H + r*IMG_W + ix[2*c+1]])); //Q10.22
					fp_data[k*NEW_W*IMG_H + r*NEW_W + c] = (int16_t) (mul >> 7); //Q10.22 to Q1.15
				}
			}
		}
	}
	
	//height resize
	void height_resize() {

		//local variables
		int16_t r, c, k;
		int32_t mul;
		
		//Height reduction	
		for(r = 0; r < NEW_H; r++) {
			for(c = 0; c < NEW_W; c++) {
				for(k = 0; k < NTW_IN_C; k++) {
					mul = (int32_t)((int32_t)dy[2*r]*(int32_t)fp_data[k*NEW_W*IMG_H + iy[r]*NEW_W + c]); //Q2.14 * Q1.15 = Q3.29
					mul += (int32_t)((int32_t)dy[2*r+1]*(int32_t)fp_data[k*NEW_W*IMG_H + (iy[r]+1)*NEW_W + c]); //Q3.29
					fp_data[k*(NTW_IN_W+2)*(NTW_IN_H+2) + (r+GREY_PADD)*(NTW_IN_W+2) + (c+1) + EXTRA_W + ((NTW_IN_W+2)*EXTRA_H) + NETWORK_INPUT_AUX] = (int16_t)(mul >> 14); //Q3.29 to Q1.15
				}
			}
		}
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
		
	//linear approximation of sigmoid function
	int32_t sigmoid(int32_t val) {
		int32_t fp2375 = 0x2600000, fp084375 = 0xD80000, fp0625 = 0xA00000, fp05 = 0x800000; //Q8.24
		int32_t fp5 = 0x5000000, fp1 = 0x1000000; //Q8.24
		int32_t val_out;
		if(val < 0.) val_out = ~val + 1; //emulates multiplying by -1 
		else val_out = val;
		if(val_out >= fp5) val_out = fp1;
		else if(val_out >= fp2375) val_out = fp084375 + (val_out >> 5); //emulates multiplying by 0.03125 = 2^(-5)
		else if(val_out >= fp1) val_out = fp0625 + (val_out >> 3); //emulates multiplying by 0.125 = 2^(-3)
		else val_out = fp05 + (val_out >> 2); //emulates multiplying by 0.25 = 2^(-2);
		if(val < 0.) val_out = fp1 - val_out;
		return val_out; //Q8.24
	}
	
	//perform convolutional layer
	void conv_layer(int w, int h, int c, int num_ker, int ker_size, int pad, int batch_norm, int nextPadding, int nextStride, int ignorePadding, unsigned int new_output_pos, unsigned int offset, int shift, int b_shift) {

		item_count=0;
		
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
		if(print_vals) printf("\n");
		int16_t * in_d_pos = (int16_t *) fp_data + data_pos;
		int16_t * out_d_pos;
		if(new_output_pos != 0) out_d_pos = (int16_t *) fp_data + new_output_pos; else out_d_pos = (int16_t *) in_d_pos + pos_delta;
		
		printf("Bias and shift:\n");
		for(int i=0;i<num_ker;i++)
			printf("\t%d -> %d\n", bias_pos[i], (bias_pos[i]<<b_shift));
		printf("\n");
		//local variables
		int i, j, k, l, m, n;
		unsigned int output_pos;
		
	#ifdef GEMM
		
		int32_t val_32;
		
		//Transformation
		for(l = 0; l < c; l++)  						//Number of channels
			for(m = 0; m < ker_size; m++) 				//Kernel size
				for(n = 0; n < ker_size; n++) 
					for(j = 0; j < h; j++)   			//Output map size
						for(k = 0; k < w; k++) 
							fp_gemm_in[l*ker_size*ker_size*h*w + m*ker_size*h*w + n*h*w + j*w + k] = in_d_pos[(j+ignorePadding)*(w+2*pad) + (k+ignorePadding) + l*(w+2*pad)*new_h + m*(w+2*pad) + n];
						
		//Convolution
		for(i = 0; i < num_ker; i++) {
			for(j = 0; j < c*ker_size*ker_size; j++) {
				for(k = 0; k < w*h; k++) { 
					if(j == 0) fp_gemm_out[i*w*h + k] = bias_pos[i] << b_shift;
					fp_gemm_out[i*w*h + k] += (int32_t)((int32_t)w_pos[i*c*ker_size*ker_size + j]*(int32_t)fp_gemm_in[j*w*h + k]);
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
					
					//activation function
					val_32 = fp_gemm_out[i*w*h + j*w + k];
					if(batch_norm) {
						//leaky
						if(val_32 < 0) val_32 = (val_32 >> 4) + (val_32 >> 5) + (val_32 >> 7);						 
					} else {
						//sigmoid
						if(i != 2 && i != 3 && i != 87 && i != 88 && i != 172 && i != 173) val_32 = sigmoid(val_32);
					}
					
					//store results
					out_d_pos[output_pos] = (int16_t) (val_32 >> shift);
				}
			}
		}
		
	#else
		
		//local variables
		unsigned int output_pos2, output_pos3, output_pos4;
		int16_t op1, op2;
		int32_t acc, acc2, acc3, acc4, mul; 
		
		//perform convolution
		for(i = 0; i < num_ker; i+=4) {								//Number of kernels
			for(j = 0; j < h; j++) {   								//Output map size
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
					acc = bias_pos[i] << b_shift;
					acc2 = bias_pos[i+1] << b_shift;
					acc3 = bias_pos[i+2] << b_shift;
					acc4 = bias_pos[i+3] << b_shift;
					if(print_vals) printf("b_shift: %d\tbias: %x\t%x\t%x\t%x\n", b_shift, acc, acc2, acc3, acc4);
										
					for(m = 0; m < ker_size; m++) {				//Kernel size
						for(n = 0; n < ker_size; n++) {
							for(l = 0; l < c; l++) { 			//Number of channels
								op1 = in_d_pos[(j+ignorePadding)*(w+2*pad) + (k+ignorePadding) + l*(w+2*pad)*new_h + m*(w+2*pad) + n];
								if(print_vals) printf("\nPixel: %d\nweights:\n", op1);
								op2 = w_pos[i*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n];
								if(print_vals) printf("%d\t", op2);
								mul = (int32_t)((int32_t)op1*(int32_t)op2);
								acc += mul;
								op2 = w_pos[(i+1)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n];
								if(print_vals) printf("%d\t", op2);
								mul = (int32_t)((int32_t)op1*(int32_t)op2);
								acc2 += mul;	
								op2 = w_pos[(i+2)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n];
								if(print_vals) printf("%d\t", op2);
								mul = (int32_t)((int32_t)op1*(int32_t)op2);
								acc3 += mul;
								op2 = w_pos[(i+3)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n];
								if(print_vals) printf("%d\n", op2);
								mul = (int32_t)((int32_t)op1*(int32_t)op2);
								acc4 += mul;								
							}
						}
					}
										
					//activation function
					if(batch_norm) {
						//leaky
						if(acc < 0) acc = (acc >> 4) + (acc >> 5) + (acc >> 7);						
						if(acc2 < 0) acc2 = (acc2 >> 4) + (acc2 >> 5) + (acc2 >> 7);					
						if(acc3 < 0) acc3 = (acc3 >> 4) + (acc3 >> 5) + (acc3 >> 7);						
						if(acc4 < 0) acc4 = (acc4 >> 4) + (acc4 >> 5) + (acc4 >> 7); 
					} else {
						//sigmoid
						if(i != 88 && i != 172) acc = sigmoid(acc);
						if(i != 172) acc2 = sigmoid(acc2);
						if(i != 0) acc3 = sigmoid(acc3);
						if(i != 0 && i != 84) acc4 = sigmoid(acc4);
					}
										
					//store results
					out_d_pos[output_pos] = (int16_t) (acc >> shift);
					out_d_pos[output_pos2] = (int16_t) (acc2 >> shift);
					out_d_pos[output_pos3] = (int16_t) (acc3 >> shift);
					out_d_pos[output_pos4] = (int16_t) (acc4 >> shift);

					int output_ptr[4] = {output_pos, output_pos2, output_pos3, output_pos4};

					int res;
					for (int item=0; item<4; item++){
						for(int _byte=0; _byte<sizeof(int16_t); _byte++){
							do{
								res = fwrite((uint8_t*)(&out_d_pos[output_ptr[item]])+_byte, sizeof(uint8_t), 1, results);
							}while(!res);
						}
						if(print_vals) printf("%d\n", out_d_pos[output_ptr[item]]);
						item_count++;
					}
					if(print_vals) exit(0);
				}
			}
		}
		
	#endif //relative to `#ifdef GEMM`
		
		//update weights and data pointers
		weight_pos += num_ker + num_ker*c*ker_size*ker_size;
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
		int16_t * in_d_pos = (int16_t *) fp_data + data_pos;
		int16_t * out_d_pos;
		if(new_output_pos != 0) out_d_pos = (int16_t *) fp_data + new_output_pos; 
		else out_d_pos = (int16_t *) in_d_pos + (w+1-downsample+2*ignorePadding)*new_h*num_ker;
		
		//local variables
		int i, j, k, l, m, new_w = w+1-downsample+2*ignorePadding;
		int16_t max, max2, max3, max4, val, val2, val3, val4;
		
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
			
	//polynomial approximation of exponential function
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
				
	//apply sigmoid to input FM
	void yolo_layer(int w, int16_t xy_div, int first_yolo, unsigned int in_pos, unsigned int out_pos) {
		
		//locate data pointers
		int16_t * input = (int16_t *) fp_data + in_pos;
		int16_t * output = (int16_t *) fp_data + out_pos;
		
		//local variables
		int16_t i, j, k, m;
		int16_t val_16, obj_score, pred_score;
		int32_t val_32;
		int16_t yolo_bias[12] = {0x0280, 0x0380, 0x05C0, 0x06C0, 0x0940, 0x0E80, 0x1440, 0x1480, 0x21C0, 0x2A40, 0x5600, 0x4FC0}; //Q10.6
				
		//loops to go through yolo layer output
		for(i = 0; i < 3; i++) {
			for(j = 0; j < w; j++) {
				for(k = 0; k < w; k++) {
					
					//sigmoid of objectness score
					obj_score = input[(4+85*i)*w*w + j*w + k]; //Q3.13
					
					//check if objectness score is above threshold
					if(obj_score > threshold) {
																							
						//Calculate x
						val_16 = input[85*i*w*w + j*w + k]; //Q3.13
						val_32 = (int32_t)((int32_t)(val_16 + (k<<13))*(int32_t)xy_div); //Q3.13 *Q1.15 = Q4.28
						val_16 = (int16_t)(val_32 >> 15); //Q4.28 to Q3.13
						val_32 = (int32_t)((int32_t)val_16*(int32_t)x_scales); //Q3.13 * Q2.14 = Q5.27
						val_16 = (int16_t)(val_32 >> 14); //Q5.27 to Q3.13
						val_16 -= (int16_t)x_bias; //Q3.13
						output[84*nboxes] = val_16; //x
												
						//Calculate y
						val_16 = input[(1+85*i)*w*w + j*w + k]; //Q3.13
						val_32 = (int32_t)((int32_t)(val_16 + (j<<13))*(int32_t)xy_div); //Q3.13 *Q1.15 = Q4.28
						val_16 = (int16_t)(val_32 >> 15); //Q4.28 to Q3.13
						val_32 = (int32_t)((int32_t)val_16*(int32_t)y_scales); //Q3.13 * Q2.14 = Q5.27
						val_16 = (int16_t)(val_32 >> 14); //Q5.27 to Q3.13
						val_16 -= (int16_t)y_bias; //Q3.13
						output[84*nboxes+1] = val_16; //y

						//Calculate w
						val_16 =  exp_fnc(input[(2+85*i)*w*w + j*w + k]); //Q3.13
						val_32 = (int32_t)((int32_t)val_16*(int32_t)w_scales); //Q3.13 * Q1.15 = Q4.28
						val_16 = (int16_t)(val_32 >> 15); //Q4.28 to Q3.13
						//val_32 = (int32_t)((int32_t)val_16*(int32_t)yolo_bias[2*(i+3*first_yolo)]); //Q2.14 * Q10.6 = Q12.20
						val_32 = (int32_t)((int32_t)val_16*(int32_t)yolo_bias[2*(i+(1-first_yolo)+3*first_yolo)]); //Q3.13 * Q10.6 = Q13.19 -> mask 1,2,3
						val_16 = (int16_t)(val_32 >> 6); //Q13.19 to Q3.13
						output[84*nboxes+2] = val_16; //w
						
						//Calculate h
						val_16 = exp_fnc(input[(3+85*i)*w*w + j*w + k]); //Q3.13
						val_32 = (int32_t)((int32_t)val_16*(int32_t)h_scales); //Q3.13 * Q1.15 = Q4.28
						val_16 = (int16_t)(val_32 >> 15); //Q4.28 to Q3.13
						//val_32 = (int32_t)((int32_t)val_16*(int32_t)yolo_bias[2*(i+3*first_yolo)+1]); //Q2.14 * Q10.6 = Q12.20
						val_32 = (int32_t)((int32_t)val_16*(int32_t)yolo_bias[2*(i+(1-first_yolo)+3*first_yolo)+1]); //Q3.13 * Q10.6 = Q13.19 -> mask 1,2,3
						val_16 = (int16_t)(val_32 >> 6); //Q13.19 to Q3.13
						output[84*nboxes+3] = val_16; //h
												
						//Calculate probability scores
						for(m = 0; m < 80; m++) {
							val_16 = input[(5+m+85*i)*w*w + j*w + k]; //Q3.13
							val_32 = (int32_t)((int32_t)val_16*(int32_t)obj_score); //Q3.13 * Q3.13 = Q6.26
							pred_score = (int16_t)(val_32 >> 13); //Q6.26 to Q3.13
							if(pred_score <= threshold) pred_score = 0; //Q3.13
							output[84*nboxes+4+m] = pred_score; // prediction scores
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
		uint16_t * out_d_pos = (uint16_t *) fp_data + (pos + 84*nboxes)*2;
		
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
				if(in_d_pos[84*j+4+i] != 0) {
				
					//Store box ID in descending order of prob score
					if(obj_cnt == 0) out_d_pos[0] = j;
					else {
						
						//Search for position of new box ID
						for(k = 0; k < obj_cnt; k++)
							if(in_d_pos[84*j+4+i] > in_d_pos[84*out_d_pos[k]+4+i])
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
					if(in_d_pos[84*out_d_pos[j]+4+i] == 0) continue;
					for(k = j+1; k < obj_cnt; k++) {
						
						//Get boxes coordinates
						x1 = in_d_pos[84*out_d_pos[j]];
						y1 = in_d_pos[84*out_d_pos[j]+1];
						w1 = in_d_pos[84*out_d_pos[j]+2];
						h1 = in_d_pos[84*out_d_pos[j]+3];
						x2 = in_d_pos[84*out_d_pos[k]];
						y2 = in_d_pos[84*out_d_pos[k]+1];
						w2 = in_d_pos[84*out_d_pos[k]+2];
						h2 = in_d_pos[84*out_d_pos[k]+3];
						
						//Calculate IoU (intersection over union)
						w = overlap(x1, w1, x2, w2); //Q3.13
						h = overlap(y1, h1, y2, h2); //Q3.13
						if(w > 0 && h > 0) {
							b_inter = (int32_t)((int32_t)w*(int32_t)h); //Q3.13 * Q3.13 = Q6.26
							mul_32 = (int32_t)((int32_t)w1*(int32_t)h1); //Q3.13 * Q3.13 = Q6.26
							b_union = (int16_t)(mul_32 >> 13); //w1*h1 -> Q6.26 to Q3.13
							mul_32 = (int32_t)((int32_t)w2*(int32_t)h2); ///Q3.13 * Q3.13 = Q6.26
							b_union += (int16_t)(mul_32 >> 13); //w1*h1+w2*h2 -> Q6.26 to Q3.13
							b_union -= (int16_t)(b_inter >> 13); //w1*h1+w2*h2-inter -> Q6.26 to Q3.13
							b_iou = (int16_t)((int32_t)b_inter/(int32_t)b_union); //Q6.26 / Q3.13 = Q3.13						
							if(b_iou > nms_threshold) in_d_pos[84*out_d_pos[k]+4+i]	= 0;
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
		for(l = 0; l < label_height && (l+top_width) < IMG_H; l++){
			for(k = 0; k < label_w && (k+left+previous_w) < IMG_W; k++){
				//Q8.0*Q8.0=Q16.0 to Q8.0 -> red
				fp_image[(l+top_width)*IMG_W+(k+left+previous_w)] = ((uint16_t)((uint16_t)r*(uint16_t)fp_labels[(81+MAX_LABEL_SIZE*j)+l*label_w+k])) >> 8;				
				//green
				fp_image[IMG_W*IMG_H+(l+top_width)*IMG_W+(k+left+previous_w)] = ((uint16_t)((uint16_t)g *(uint16_t)fp_labels[(81+MAX_LABEL_SIZE*j)+l*label_w+k])) >> 8;
				//blue
				fp_image[2*IMG_W*IMG_H+(l+top_width)*IMG_W+(k+left+previous_w)] = ((uint16_t)((uint16_t)b *(uint16_t)fp_labels[(81+MAX_LABEL_SIZE*j)+l*label_w+k])) >> 8;
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
				if(in_d_pos[84*i+4+j] != 0) {
								
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
						mul_16 = in_d_pos[84*i] - (in_d_pos[84*i+2]>>1); //Q3.13
						mul_32 = (int32_t)((int32_t)mul_16 * (int32_t)IMG_W); //Q3.13 * Q16.0 = Q19.13
						left = (mul_32 >> 13);					
						mul_16 = in_d_pos[84*i] + (in_d_pos[84*i+2]>>1); //Q3.13
						mul_32 = (int32_t)((int32_t)mul_16 * (int32_t)IMG_W); //Q3.13 * Q16.0 = Q19.13
						right = (mul_32 >> 13);
						mul_16 = in_d_pos[84*i+1] - (in_d_pos[84*i+3]>>1); //Q3.13
						mul_32 = (int32_t)((int32_t)mul_16 * (int32_t)IMG_H); //Q3.13 * Q16.0 = Q19.13
						top = (mul_32 >> 13);
						mul_16 = in_d_pos[84*i+1] + (in_d_pos[84*i+3]>>1); //Q3.13
						mul_32 = (int32_t)((int32_t)mul_16 * (int32_t)IMG_H); //Q3.13 * Q16.0 = Q19.13
						bot = (mul_32 >> 13);					

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
		const char *class_names[80] = {"person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", 
							"stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", 
							"backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", 
							"baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", 
							"banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed", "dining table", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};
		for(i = 0; i < nboxes; i++) {
			for(j = 0; j < 80; j++) {
				if(fp_data[box_pos+84*i+4+j] != 0) {
					pred_32 = (uint32_t)((uint32_t)fp_data[box_pos+84*i+4+j]*(uint32_t)100); //Q3.13 * Q16.0 = Q19.13
					if( (pred_32&0x1FFF) > 0x1000) printf("\n%s: %d%%", class_names[j], (pred_32>>13)+1);
					else printf("\n%s: %d%%", class_names[j], (pred_32>>13));
				}
			}
		}
		printf("\n");	
	}

	//send detection results back	
	void send_data(char **argv) {

		//file pointer
		FILE *data;
		
		//Find filename
		char * filename_full = argv[1], * ch_p = strtok(filename_full, "/."), *filename = ch_p;
		while(ch_p != NULL) {
			ch_p = strtok(NULL, "/.");
			if(strcmp(ch_p,"bin")==0) break;
			filename = ch_p;
		}
		char output_filename[100] = "../output_data/";
		strcat(output_filename, filename);
		strcat(output_filename, ".bin"); 
		
		//create new image
		if ((data = fopen(output_filename, "wb")) == NULL) {
			fprintf(stderr, "unable to open result file\n");
			exit(1);
		}
		int w, h, c, i;
		char * fp_image_char = (char *) fp_image;
		w = IMG_W;
		h = IMG_H;
		c = IMG_C;
		fwrite(&w, sizeof(int), 1, data);
		fwrite(&h, sizeof(int), 1, data);
		fwrite(&c, sizeof(int), 1, data);
		for(i = 0; i < IMAGE_INPUT; i++) fwrite(fp_image_char + 2*i, sizeof(char), 1, data);
		fclose(data);
	}

	#ifdef mAP
		//print in json file
		static void print_cocos(FILE *fp, char *image_path, unsigned int pos) {
			
			//local variables
			int16_t * out_d_pos = (int16_t *) fp_data + pos;
			int i, j;
			int image_id = get_coco_image_id(image_path);
			float xmax, ymax;
			float bx, by, bw, bh;
			float x, y, w, h, prob;
			
			//loop to output bounding boxes in .json file
			for(i = 0; i < nboxes; i++) {
				
				//Convert from Q3.13 to float
				x = ((float)out_d_pos[84*i]/((int16_t)1<<13))*IMG_W;
				y = ((float)out_d_pos[84*i+1]/((int16_t)1<<13))*IMG_H;
				w = ((float)out_d_pos[84*i+2]/((int16_t)1<<13))*IMG_W;
				h = ((float)out_d_pos[84*i+3]/((int16_t)1<<13))*IMG_H;
								
				//Calculate x,y coordinates
				bx = x - w/2.;
				xmax = x + w/2.;
				by = y - h/2.;
				ymax = y + h/2.;
							
				//Limit x,y coordinates
				if (bx < 0) bx = 0;
				if (by < 0) by = 0;
				if (xmax > IMG_W) xmax = IMG_W;
				if (ymax > IMG_H) ymax = IMG_H;
				
				//Calculate relative box coordinates
				bw = xmax - bx;
				bh = ymax - by;
				
				//Print to .json file
				for(j = 0; j < 80; ++j) {
					prob = (float)out_d_pos[84*i+4+j]/((int16_t)1<<14);
					if(prob) 
						fprintf(fp, "{\"image_id\":%d, \"category_id\":%d, \"bbox\":[%f, %f, %f, %f], \"score\":%f},\n", image_id, coco_ids[j], bx, by, bw, bh, prob);
				}
			}
		}
	#endif //relative to `#ifdef mAP`
	
#else //relative to `#ifdef FIXED`
	
	#include <math.h>

FILE *results;
#define conv_out_filename "../yolov3-tiny_float_conv.hex"

	//Constants for bounding boxes
#ifdef mAP
	#define threshold ((float)0.005)
#else
	#define threshold ((float)0.5)
#endif
	#define nms_threshold ((float)0.45)
	#define yolo1_div ((float)1/LAYER_17_W)
	#define yolo2_div ((float)1/LAYER_24_W)
	#define x_scales ((float)YOLO_INPUT/NEW_W)
	#define x_bias ((float)(YOLO_INPUT-NEW_W)/(NEW_W*2))
	#define y_scales ((float)YOLO_INPUT/NEW_H)
	#define y_bias ((float)(YOLO_INPUT-NEW_H)/(NEW_H*2))
	#define w_scales ((float)1/NEW_W)
	#define h_scales ((float)1/NEW_H)
	#define c3 (pow(2,-3)+pow(2,-5)+pow(2,-7)+pow(2,-9)+pow(2,-11)+pow(2,-13))
	#define c4 (pow(2,-5)+pow(2,-7)+pow(2,-8))
	#define box_width 3
	#define label_height 20
	int nboxes = 0;

	//weights and data base address pointers
	float * fp_weights;
	float * fp_data;
	float * fp_image;
	float * fp_labels;
#ifdef GEMM
	float * fp_gemm_in;
	float * fp_gemm_out;
#endif

	//weights and data updatable pointers
	unsigned int weight_pos = 0, data_pos = 0;
	
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
	
	//reset DDR to zero
	void reset_DDR() {
		memset((void *) data, 0, TOTAL_DATA*sizeof(float));
		float * aux_p = (float *) fp_data;
		aux_p += NETWORK_INPUT_AUX + NETWORK_INPUT + DATA_LAYER_1 + DATA_LAYER_2 + DATA_LAYER_3 + DATA_LAYER_4 + DATA_LAYER_5 + DATA_LAYER_6 + DATA_LAYER_7 + DATA_LAYER_8 + DATA_LAYER_10;
		int i;
		for(i = 0; i < DATA_LAYER_11; i++) aux_p[i] = -FLT_MAX;
	}

	//receive weights and data
	void receive_data(char **argv) {
				
		//file pointers
		FILE *data;
		unsigned char aux;
		int i, j, num;
		
		//load input image
		if ((data = fopen(argv[1], "r+")) == NULL) {
			fprintf(stderr, "unable to open data file \n");
			exit(1);
		}
		//fseek(data, 12, SEEK_SET); //ignore image size info (3 int values)
		
		//calculate dimensions
		//fseek(data, 12, SEEK_SET); //ignore image size info (3 int values)
		fread(&IMG_W, sizeof(int), 1, data);
		fread(&IMG_H, sizeof(int), 1, data);
		fread(&IMG_C, sizeof(int), 1, data);
		IMAGE_INPUT = IMG_W*IMG_H*IMG_C;
		if(IMG_W>=IMG_H) {
			NEW_W = YOLO_INPUT;
			NEW_H = ((IMG_H*NEW_W)/IMG_W);
			EXTRA_W = 0;
			EXTRA_H = ((NEW_W-NEW_H)/2);
		} else {
			NEW_H = YOLO_INPUT;
			NEW_W = ((IMG_W*NEW_H)/IMG_H);
			EXTRA_H = 0;
			EXTRA_W = ((NEW_H-NEW_W)/2);
		}
		NETWORK_INPUT_AUX = NEW_W*IMG_H*IMG_C;
		data_pos = NETWORK_INPUT_AUX;
		
		//define memory regions for pointers
		define_memory_regions();
			
		//Reset DDR to zero
		reset_DDR();
		
		//load image		
		for(i = 0; i < IMAGE_INPUT; i++) {
			fread(&aux, sizeof(char), 1, data);
			fp_image[i] = (float) (aux / 255.);
		}
		fclose(data);
		
		//load weigths
		if ((data = fopen("../yolov3-tiny_batch-float.weights", "r+")) == NULL) {
			fprintf(stderr, "unable to open file yolov3-tiny_batch-float.weights\n");
			exit(1);
		}
		fread(fp_weights, sizeof(float), TOTAL_WEIGTHS, data);
		fclose(data);
		/* Print 4th kernel of layer 1 */
		printf("Layer 1 -- 4th kernel\n");
		for(int i=0; i<LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*LAYER_1_C; i++)
			printf("%f\n", fp_weights[3*LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*LAYER_1_C+i]);
		
		//load labels
		if ((data = fopen("data/classes.bin", "r+")) == NULL) {
			fprintf(stderr, "unable to open file classes.bin\n");
			exit(1);
		}
		//read label widths
		for(i = 0; i < 81; i++) {
			fread(&aux, sizeof(char), 1, data);
			fp_labels[i] = (float) aux; 
		}
		//read label data
		for(i = 0; i < 81; i++) {
			num = (int) fp_labels[i];
			for(j = 0; j < num * label_height; j++) {
				fread(&aux, sizeof(uint8_t), 1, data);
				fp_labels[81 + MAX_LABEL_SIZE*i + j] = (float) (aux / 255.);
			}
		}
		fclose(data);
		
	#ifdef INTERM_DATA
		//load intermediate results
		unsigned int pos_layer = NETWORK_INPUT;
		if ((data = fopen("../interm_data-float.network", "r+")) == NULL) {
			fprintf(stderr, "unable to open interm_data.network file \n");
			exit(1);
		}
			
		//layer 1
		for(i = 0; i < NTW_IN_NUM_KER; i++) {
			fread((fp_data + pos_layer), sizeof(float), NTW_IN_W, data); //Read 1st line
			pos_layer += (NTW_IN_W*(NTW_IN_H+1));
			fread((fp_data + pos_layer), sizeof(float), NTW_IN_W, data); //Read 2nd line
			pos_layer += NTW_IN_W;
		}
		
		//layer 2
		for(i = 0; i < LAYER_2_NUM_KER; i++) {
			fread((fp_data + pos_layer), sizeof(float), (LAYER_3_W+2)*2, data); //Read 1st 2 lines
			pos_layer += (LAYER_3_W+2)*LAYER_3_H;
			fread((fp_data + pos_layer), sizeof(float), (LAYER_3_W+2)*2, data); //Read 2nd 2 lines
			pos_layer += (LAYER_3_W+2)*2;
		}
		
		//layer 4
		pos_layer += DATA_LAYER_3;
		for(i = 0; i < LAYER_4_NUM_KER; i++) {
			fread((fp_data + pos_layer), sizeof(float), (LAYER_5_W+2)*2, data); //Read 1st two lines
			pos_layer += (LAYER_5_W+2)*LAYER_5_H;
			fread((fp_data + pos_layer), sizeof(float), (LAYER_5_W+2)*2, data); //Read 2nd two lines
			pos_layer += (LAYER_5_W+2)*2;
		}
		
		//layer 5
		for(i = 0; i < LAYER_5_NUM_KER; i++) {
			fread((fp_data + pos_layer), sizeof(float), LAYER_5_W, data); //Read 1st line
			pos_layer += LAYER_5_W*(1+LAYER_5_H);
			fread((fp_data + pos_layer), sizeof(float), LAYER_5_W, data); //Read 2nd line
			pos_layer += LAYER_5_W;
		}
		
		//layer 6
		for(i = 0; i < LAYER_6_NUM_KER; i++) {
			fread((fp_data + pos_layer), sizeof(float), (LAYER_7_W+2)*2, data); //Read 1st 2 lines
			pos_layer += (LAYER_7_W+2)*LAYER_7_H;
			fread((fp_data + pos_layer), sizeof(float), (LAYER_7_W+2)*2, data); //Read 2nd 2 lines
			pos_layer += (LAYER_7_W+2)*2;
		}
		
		//layer 8
		pos_layer += DATA_LAYER_7;
		for(i = 0; i < LAYER_8_NUM_KER; i++) {
			fread((fp_data + pos_layer), sizeof(float), (LAYER_9_W+2)*2, data); //Read 1st rectangle
			pos_layer += (LAYER_9_W+2)*LAYER_9_H;
			fread((fp_data + pos_layer), sizeof(float), (LAYER_9_W+2)*2, data); //Read 2nd rectangle
			pos_layer += (LAYER_9_W+2)*2;
		}
		
		//layer 9
		pos_layer += DATA_LAYER_10 + DATA_LAYER_11 + DATA_LAYER_12 + DATA_LAYER_13 + DATA_LAYER_14 + DATA_LAYER_15 + DATA_LAYER_16 + DATA_LAYER_17 + DATA_LAYER_19 + DATA_LAYER_20;
		for(i = 0; i < LAYER_9_NUM_KER; i++) {
			pos_layer += LAYER_9_W+2;
			fread((fp_data + pos_layer), sizeof(float), (LAYER_9_W+2), data); //Read 1st rectangle
			pos_layer += (LAYER_9_W+2)*(1+LAYER_9_H);
			fread((fp_data + pos_layer), sizeof(float), (LAYER_9_W+2), data); //Read 1st rectangle
			pos_layer += (LAYER_9_W+2)*2;
		}	
		fclose(data);
	#endif
	}

	//fill region of resized image with grey (0.5)
	void fill_grey() {
		int i, j, k;
		for(i = 0; i < NTW_IN_C; i++)
		#ifdef INTERM_DATA
			for(j = 0; j < NTW_IN_H+2; j++)
		#else
			for(j = 0; j < NTW_IN_H; j++)
		#endif
				for(k = 0; k < NTW_IN_W; k++)
					fp_data[i*(NTW_IN_W+2)*(NTW_IN_H+2) + (j+GREY_PADD)*(NTW_IN_W+2) + (k+1) + NETWORK_INPUT_AUX] = 0.5;
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
			//ix
			ix[2*i] = val_i;
			ix[2*i+1] = val_i + 1;
			//dx
			dx[2*i] = 1 - val_d;
			dx[2*i+1] = val_d;
		}
		
		//loop to initialize iy and dy
		for(i = 0; i < NEW_H; i++) {
			val = i*h_scale;
			iy[i] = (int) val;
			val_d = val - iy[i];
			//dy
			dy[2*i] = 1 - val_d;
			dy[2*i+1] = val_d; 
		}
	}

	//width resize
	void width_resize() {
		
		//local variables
		int16_t r, c, k;
		
		//Width reduction
		for(k = 0; k < NTW_IN_C; k++)
			for(r = 0; r < IMG_H; r++)		
				for(c = 0; c < NEW_W; c++)		
					fp_data[k*IMG_H*NEW_W + r*NEW_W + c] = dx[2*c]*fp_image[k*IMG_W*IMG_H + r*IMG_W + ix[2*c]] + dx[2*c+1]*fp_image[k*IMG_W*IMG_H + r*IMG_W + ix[2*c+1]];
	}
	
	//height resize
	void height_resize() {

		//local variables
		int16_t r, c, k;
		
		//Height reduction	
		for(k = 0; k < NTW_IN_C; k++)
			for(r = 0; r < NEW_H; r++)		
				for(c = 0; c < NEW_W; c++)
					fp_data[k*(NTW_IN_W+2)*(NTW_IN_H+2) + (r+GREY_PADD)*(NTW_IN_W+2) + (c+1) + EXTRA_W + ((NTW_IN_W+2)*EXTRA_H) + NETWORK_INPUT_AUX] = dy[2*r]*fp_data[k*NEW_W*IMG_H + iy[r]*NEW_W + c] + dy[2*r+1]*fp_data[k*NEW_W*IMG_H + (iy[r]+1)*NEW_W + c];
	}

	//sigmoid function
	float sigmoid(float val) {
	#ifdef LINEAR_EXP
		//linear approximation of sigmoid function
		float val_out;
		if(val < 0.) val_out = -val;
		else val_out = val;	
		if(val_out >= 5.) val_out = 1;
		else if(val_out >= 2.375) val_out = 0.03125*val_out+0.84375;
		else if(val_out >= 1.) val_out = 0.125*val_out+0.625;
		else val_out = 0.25*val_out+0.5;
		if(val < 0.) val_out = 1. - val_out;
		return val_out;
	#else
		return (1./(1. + exp(-val)));
	#endif
	}
	
	//perform convolutional layer
	void conv_layer(int w, int h, int c, int num_ker, int ker_size, int pad, int batch_norm, int nextPadding, int nextStride, int ignorePadding, unsigned int new_output_pos, unsigned int offset, int shift, int b_shift) {

		item_count=0;
		
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
	
		printf("Bias:\n");
		for(int i=0;i<num_ker;i++)
			printf("\t%.4f\n", bias_pos[i]);
		//local variables
		int i, j, k, l, m, n;
		unsigned int output_pos;
		float output_conv;
	#ifdef MAX_MIN
		float min = 100000000, max = -100000000;
	#endif
				
	#ifdef GEMM
	
		//Transformation
		for(l = 0; l < c; l++)  						//Number of channels
			for(m = 0; m < ker_size; m++) 				//Kernel size
				for(n = 0; n < ker_size; n++) 
					for(j = 0; j < h; j++)   			//Output map size
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
					
					//Add bias
					output_conv = fp_gemm_out[i*w*h + j*w + k] + bias_pos[i];
					
					//activation function
					if(batch_norm) {
						if (output_conv < 0) output_conv *= 0.1015625;
					} else {
						if(i != 2 && i != 3 && i != 87 && i != 88 && i != 172 && i != 173) output_conv = sigmoid(output_conv);
					}
					out_d_pos[output_pos] = output_conv;
					
					//register max and min output values
				#ifdef MAX_MIN
					if(min > output_conv) min = output_conv;
					if(max < output_conv) max = output_conv;
				#endif
				}
			}
		}
		
		//Write max and min to file
	#ifdef MAX_MIN
		FILE * data;
		if ((data = fopen("max_min.hex", "a")) == NULL) {
			fprintf(stderr, "unable to open result file\n");
			exit(1);
		}
		fwrite(&max, sizeof(float), 1, data);
		fwrite(&min, sizeof(float), 1, data);
		fclose(data);
		
		if((data = fopen("max_min.txt","a")) == NULL){
			fprintf(stderr, "unable to open result file\n");
			exit(1);
		}
		fprintf(data, "Max=%.4f\tMin=%.4f\n", max, min);
		fclose(data);
	#endif
				
	#else
		
		//local variables
		unsigned int output_pos2, output_pos3, output_pos4;
		float op1, op2, op2_2, op2_3, op2_4;
		float output_conv2, output_conv3, output_conv4;
		float acc_final, acc_final2, acc_final3, acc_final4;
		
		//perform convolution
		for(i = 0; i < num_ker; i+=4) {								//Number of kernels
			for(j = 0; j < h; j++) {   								//Output map size
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
					//printf("bias: %f\t%f\t%f\t%f\n",bias_pos[i],bias_pos[i+1],bias_pos[i+2],bias_pos[i+3]);
					for(m = 0; m < ker_size; m++) {				//Kernel size
						for(n = 0; n < ker_size; n++) {
							for(l = 0; l < c; l++) { 						//Number of channels
								op1 = in_d_pos[(j+ignorePadding)*(w+2*pad) + (k+ignorePadding) + l*(w+2*pad)*new_h + m*(w+2*pad) + n];
								op2 = w_pos[i*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n];
								acc_final += op1*op2;
								op2_2 = w_pos[(i+1)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n];
								acc_final2 += op1*op2_2;
								op2_3 = w_pos[(i+2)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n];
								acc_final3 += op1*op2_3;
								op2_4 = w_pos[(i+3)*c*ker_size*ker_size + l*ker_size*ker_size + m*ker_size + n];
								acc_final4 += op1*op2_4;
								//printf("pixel: %f\tweights: %f\t%f\t%f\t%f\n", op1,op2,op2_2,op2_3,op2_4);
							}
						}
					}
					
					//add bias
					output_conv = acc_final + bias_pos[i];
					output_conv2 = acc_final2 + bias_pos[i+1];
					output_conv3 = acc_final3 + bias_pos[i+2];
					output_conv4 = acc_final4 + bias_pos[i+3];
					
					//perform leaky activation function
					if(batch_norm) {
						if(output_conv < 0) output_conv *= 0.1015625;						
						if(output_conv2 < 0) output_conv2 *= 0.1015625;						
						if(output_conv3 < 0) output_conv3 *= 0.1015625;							
						if(output_conv4 < 0) output_conv4 *= 0.1015625;	
					} else {
						if(i != 88 && i != 172) output_conv = sigmoid(output_conv);
						if(i != 172) output_conv2 = sigmoid(output_conv2);
						if(i != 0) output_conv3 = sigmoid(output_conv3);
						if(i != 0 && i != 84) output_conv4 = sigmoid(output_conv4);
					}
					
					out_d_pos[output_pos] = output_conv;
					out_d_pos[output_pos2] = output_conv2;
					out_d_pos[output_pos3] = output_conv3;
					out_d_pos[output_pos4] = output_conv4;
					
					
					/*
					char *p=(char*)&output_conv;
					for(int by=0; by<4; by++)
						printf("%02hhX ",p[by]);
					printf("\t%a\t%f\n", output_conv, output_conv);
					
					p=(char*)&output_conv2;
					for(int by=0; by<4; by++)
						printf("%02hhX ",p[by]);
					printf("\t%a\t%f\n", output_conv2, output_conv2);
					*/

					uint8_t* output_ptr[4] = {(uint8_t*) &output_conv, (uint8_t*) &output_conv2, (uint8_t*) &output_conv3, (uint8_t*) &output_conv4};
					
					//fwrite((uint8_t*)&output_conv, sizeof(uint8_t), 4, results);
					//fwrite((uint8_t*)&output_conv2, sizeof(uint8_t), 4, results);
					//fwrite((uint8_t*)&output_conv3, sizeof(uint8_t), 4, results);
					//fwrite((uint8_t*)&output_conv4, sizeof(uint8_t), 4, results);
					int res;
					for (int item=0; item<4; item++){
						for(int _byte=0; _byte<sizeof(float); _byte++){
							do{
								res = fwrite(output_ptr[item]+_byte, sizeof(uint8_t), 1, results);
							}while(!res);
						}
						//printf("%f\t", *((float*)output_ptr[item]));
						item_count++;
					}
					//printf("\n");//exit(0);
				}
			}
		}
		
	#endif //relative to `#ifdef GEMM`
		
		//update weights and data pointers
		weight_pos += num_ker + num_ker*c*ker_size*ker_size;
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
	
	//Exponential function
	float exp_fnc(float val) {
	#ifdef LINEAR_EXP
		//Polynomial approximation of exponential function
		float val_aux, exp_val_fixed;
		exp_val_fixed = val + 1;
		val_aux = val*val; //w^2
		exp_val_fixed += 0.5*val_aux; //1+w+0.5*w^2
		val_aux *= val; //w^3
		exp_val_fixed += c3*val; //1+w+0.5*w^2+c3*w^3
		val_aux *= val; //w^4
		exp_val_fixed += c4*val_aux; //1+w+0.5*w^2+c3*w^3+c4*w^4
		return exp_val_fixed;
	#else
		return exp(val);
	#endif
	}
		
	//apply sigmoid to input FM
	void yolo_layer(int w, float xy_div, int first_yolo, unsigned int in_pos, unsigned int out_pos) {
		
		//locate data pointers
		float * input = (float *) fp_data + in_pos;
		float * output = (float *) fp_data + out_pos;
		
		//local variables
		int i, j, k, l;
		float obj_score, pred_score;
		int yolo_bias[12] = {10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319};
		
		//perform yolo layer
		for(i = 0; i < 3; i++) {
			for(j = 0; j < w; j++) {
				for(k = 0; k < w; k++) {
				
					//sigmoid of objectness score
					//obj_score = sigmoid(input[(4+85*i)*w*w + j*w + k]);				
					obj_score = input[(4+85*i)*w*w + j*w + k];				
			
					//check if objectness score is above threshold
					if(obj_score > threshold) {
						
						//Calculate x
						//output[84*nboxes] = (sigmoid(input[85*i*w*w + j*w + k])+k)*xy_div*x_scales - x_bias;
						output[84*nboxes] = (input[85*i*w*w + j*w + k]+k)*xy_div*x_scales - x_bias;
											
						//Calculate y
						//output[84*nboxes+1] = (sigmoid(input[(1+85*i)*w*w + j*w + k])+j)*xy_div*y_scales - y_bias;
						output[84*nboxes+1] = (input[(1+85*i)*w*w + j*w + k]+j)*xy_div*y_scales - y_bias;

						//Calculate w
						//output[84*nboxes+2] = exp_fnc(input[(2+85*i)*w*w + j*w + k])*w_scales*yolo_bias[2*(i+3*first_yolo)]; //mask 0,1,2
						output[84*nboxes+2] = exp_fnc(input[(2+85*i)*w*w + j*w + k])*w_scales*yolo_bias[2*(i+(1-first_yolo)+3*first_yolo)]; //mask 1,2,3
						
						//Calculate h
						//output[84*nboxes+3] = exp_fnc(input[(3+85*i)*w*w + j*w + k])*h_scales*yolo_bias[2*(i+3*first_yolo)+1]; //mask 0,1,2
						output[84*nboxes+3] = exp_fnc(input[(3+85*i)*w*w + j*w + k])*h_scales*yolo_bias[2*(i+(1-first_yolo)+3*first_yolo)+1]; //mask 1,2,3
						
						//Calculate probability scores
						for(l = 0; l < 80; l++) {
							//pred_score = sigmoid(input[(5+l+85*i)*w*w + j*w + k])*obj_score;
							pred_score = input[(5+l+85*i)*w*w + j*w + k]*obj_score;
							if(pred_score <= threshold) pred_score = 0;
							output[84*nboxes+4+l] = pred_score;
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
		uint16_t * out_d_pos = (uint16_t *) fp_data + (pos + 84*nboxes)*4;
				
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
				if(in_d_pos[84*j+4+i] != 0) {
									
					//Store box ID in descending order of prob score
					if(obj_cnt == 0) out_d_pos[0] = j;
					else {
						
						//Search for position of new box ID
						for(k = 0; k < obj_cnt; k++)
							if(in_d_pos[84*j+4+i] > in_d_pos[84*out_d_pos[k]+4+i])
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
					if(in_d_pos[84*out_d_pos[j]+4+i] == 0) continue;
					for(k = j+1; k < obj_cnt; k++) {
						
						//Get boxes coordinates
						x1 = in_d_pos[84*out_d_pos[j]];
						y1 = in_d_pos[84*out_d_pos[j]+1];
						w1 = in_d_pos[84*out_d_pos[j]+2];
						h1 = in_d_pos[84*out_d_pos[j]+3];
						x2 = in_d_pos[84*out_d_pos[k]];
						y2 = in_d_pos[84*out_d_pos[k]+1];
						w2 = in_d_pos[84*out_d_pos[k]+2];
						h2 = in_d_pos[84*out_d_pos[k]+3];
						
						//Calculate IoU (intersection over union)
						w = overlap(x1, w1, x2, w2);
						h = overlap(y1, h1, y2, h2);
						if(w > 0 && h > 0) {
							b_inter = w*h;
							b_union = w1*h1 + w2*h2 - b_inter;
							b_iou = b_inter/b_union;						
							if(b_iou > nms_threshold) in_d_pos[84*out_d_pos[k]+4+i] = 0;
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
				if(in_d_pos[84*i+4+j] != 0) {
													
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
						left = (in_d_pos[84*i] - (in_d_pos[84*i+2]/2))*IMG_W;
						right = (in_d_pos[84*i] + (in_d_pos[84*i+2]/2))*IMG_W;
						top = (in_d_pos[84*i+1] - (in_d_pos[84*i+3]/2))*IMG_H;
						bot = (in_d_pos[84*i+1] + (in_d_pos[84*i+3]/2))*IMG_H;				

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
		const char *class_names[80] = {"person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", 
							"stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", 
							"backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", 
							"baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", 
							"banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed", "dining table", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};
		for(i = 0; i < nboxes; i++)
			for(j = 0; j < 80; j++)
				if(fp_data[box_pos+84*i+4+j] != 0) 
					printf("\n%s: %.2f%%", class_names[j], fp_data[box_pos+84*i+4+j]*100);
		printf("\n");	
	}
	
	//send detection results back	
	void send_data(char **argv) {

		//file pointer
		FILE *data;
		
		//Find filename
		char * filename_full = argv[1], * ch_p = strtok(filename_full, "/."), *filename = ch_p;
		while(ch_p != NULL) {
			ch_p = strtok(NULL, "/.");
			if(strcmp(ch_p,"bin")==0) break;
			filename = ch_p;
		}
		char output_filename[100] = "../output_data/";
		strcat(output_filename, filename);
		strcat(output_filename, ".bin"); 
		
		//create new image
		if ((data = fopen(output_filename, "wb")) == NULL) {
			fprintf(stderr, "unable to open result file\n");
			exit(1);
		}
		
		//write image dimensions
		int w, h, c;
		w = IMG_W;
		h = IMG_H;
		c = IMG_C;
		fwrite(&w, sizeof(int), 1, data);
		fwrite(&h, sizeof(int), 1, data);
		fwrite(&c, sizeof(int), 1, data);
		
		//write image pixels
		unsigned char pixel;
		int i;
		for(i = 0; i < IMAGE_INPUT; i++) {
			pixel = fp_image[i]*255;
			fwrite(&pixel, sizeof(pixel), 1, data);
		}
		fclose(data);
	}
	
	#ifdef mAP
		//print in json file
		static void print_cocos(FILE *fp, char *image_path, unsigned int pos) {
			
			//local variables
			float * out_d_pos = (float *) fp_data + pos;
			int i, j;
			int image_id = get_coco_image_id(image_path);
			float xmax, ymax;
			float bx, by, bw, bh;
			
			//loop to output bounding boxes in .json file
			for(i = 0; i < nboxes; i++) {
				
				//not relative
				out_d_pos[84*i] *= IMG_W;
				out_d_pos[84*i+1] *= IMG_H;
				out_d_pos[84*i+2] *= IMG_W;
				out_d_pos[84*i+3] *= IMG_H;
				
				//Calculate x,y coordinates
				bx = out_d_pos[84*i] - out_d_pos[84*i+2]/2.;
				xmax = out_d_pos[84*i] + out_d_pos[84*i+2]/2.;
				by = out_d_pos[84*i+1] - out_d_pos[84*i+3]/2.;
				ymax = out_d_pos[84*i+1] + out_d_pos[84*i+3]/2.;
											
				//Limit x,y coordinates
				if (bx < 0) bx = 0;
				if (by < 0) by = 0;
				if (xmax > IMG_W) xmax = IMG_W;
				if (ymax > IMG_H) ymax = IMG_H;
				
				//Calculate relative box coordinates
				bw = xmax - bx;
				bh = ymax - by;
				
				//Print to .json file
				for(j = 0; j < 80; ++j)
					if(out_d_pos[84*i+4+j]) 
						fprintf(fp, "{\"image_id\":%d, \"category_id\":%d, \"bbox\":[%f, %f, %f, %f], \"score\":%f},\n", image_id, coco_ids[j], bx, by, bw, bh, out_d_pos[84*i+4+j]);
			}
		}
	#endif
	
#endif //relative to `#ifdef FIXED`

#endif //
	

//run tiny-yolo network
int main(int argc, char **argv) {

	print_vals=0;

	//open output file
	if((results = fopen(conv_out_filename, "wb"))==NULL){
		printf("Unable to open results file\n");
		exit(1);
	}

	//load data
	receive_data(argv);
	
	
	//layers 9, 10, 14 and 19 initial addresses
	unsigned int data_pos_layer10 = data_pos + NETWORK_INPUT + DATA_LAYER_1 + DATA_LAYER_2 + DATA_LAYER_3 + DATA_LAYER_4 + DATA_LAYER_5 + DATA_LAYER_6 + DATA_LAYER_7 + DATA_LAYER_8;
	unsigned int data_pos_layer14 = data_pos_layer10 + DATA_LAYER_10 + DATA_LAYER_11 + DATA_LAYER_12 + DATA_LAYER_13;
	unsigned int data_pos_layer19 = data_pos_layer14 + DATA_LAYER_14 + DATA_LAYER_15 + DATA_LAYER_16;
	unsigned int data_pos_layer9 = data_pos_layer19 + DATA_LAYER_19 + DATA_LAYER_20;
	unsigned int box_pos = data_pos_layer9 + DATA_LAYER_9 + DATA_LAYER_22 + DATA_LAYER_23 + DATA_LAYER_24;
		
	//fill part of 416x316 region of resized image with grey (0.5 = 0x0080 in Q8.8)
	fill_grey();
	
	//initialize ix, iy, dx and dy arrays
	prepare_resize();
	
	//start time measurement
	clock_t start, end;
	double cpu_time_used = 0;
	
	//width resize
	start = clock();
	width_resize();
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("\nWidth reduction in %f seconds\n", ((double) (end - start)) / CLOCKS_PER_SEC);
#endif
	//exit(0);

	//height resize
	start = clock();
	height_resize();
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Height reduction in %f seconds\n", ((double) (end - start)) / CLOCKS_PER_SEC);
#endif
	
	//Quantization Test. Print some values
	printf("----------\n  Input:\n");
	for(int i=0;i<10;i++)
		#if defined(FIXED) || defined(EIGHT_BITS)
		printf("%d: %x\n", i, (fp_data)[i]);
		#else
		printf("%d: %.4f\n", i, (fp_data)[i]);
		#endif
	printf("----------\n Weights:\n");
	for(int i=0;i<10;i++)
		#if defined(FIXED) || defined(EIGHT_BITS)
		printf("%d: %x\n", i, (fp_weights)[i]);
		#else
		printf("%d: %.4f\n", i, (fp_weights)[i]);
		#endif


	//layer1 (418x316x3 -> 416x316x16)
	start = clock();
	conv_layer(LAYER_1_W, LAYER_1_H, LAYER_1_C, LAYER_1_NUM_KER, LAYER_1_KER_SIZE, LAYER_1_PAD, LAYER_1_BATCH_NORM, LAYER_1_NEXT_PADD, LAYER_1_NEXT_STRIDE, LAYER_1_IGNORE_PADD, 0, LAYER_1_OFFSET, LAYER_1_SHIFT, LAYER_1_B_SHIFT);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Layer1 done in %f seconds\t%d values written to file\n", ((double) (end - start)) / CLOCKS_PER_SEC, item_count);
#endif
	/*
	//Quantization Test. Print some values
	printf("----------\n  Data:\n");
	for(int i=0;i<10;i++)
		#if defined(FIXED) || defined(EIGHT_BITS)
		printf("%d: %x\n", i, (fp_data+data_pos)[i]);
		#else
		printf("%d: %.4f\n", i, (fp_data+data_pos)[i]);
		#endif
	return 0;	
	*/
	//exit(0);

	//layer2 (416x316x16 -> 210x162x16)
	start = clock();
	maxpool_layer(LAYER_2_W, LAYER_2_H, LAYER_2_NUM_KER, LAYER_2_DOWNSAMPLE, LAYER_2_IGNORE_PADD, 0);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Layer2 done in %f seconds\n", ((double) (end - start)) / CLOCKS_PER_SEC);
#endif

	//layer3 (210x162x16 -> 208x160x32)
	start = clock();
	conv_layer(LAYER_3_W, LAYER_3_H, LAYER_3_C, LAYER_3_NUM_KER, LAYER_3_KER_SIZE, LAYER_3_PAD, LAYER_3_BATCH_NORM, LAYER_3_NEXT_PADD, LAYER_3_NEXT_STRIDE, LAYER_3_IGNORE_PADD, 0, LAYER_3_OFFSET, LAYER_3_SHIFT, LAYER_3_B_SHIFT);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Layer3 done in %f seconds\t%d values written to file\n", ((double) (end - start)) / CLOCKS_PER_SEC, item_count);
#endif
	
	//layer4 (208x160x32 -> 106x84x32)
	start = clock();
	maxpool_layer(LAYER_4_W, LAYER_4_H, LAYER_4_NUM_KER, LAYER_4_DOWNSAMPLE, LAYER_4_IGNORE_PADD, 0);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Layer4 done in %f seconds\n", ((double) (end - start)) / CLOCKS_PER_SEC);
#endif
	
	//layer5 (106x84x32 -> 104x84x64)
	start = clock();
	conv_layer(LAYER_5_W, LAYER_5_H, LAYER_5_C, LAYER_5_NUM_KER, LAYER_5_KER_SIZE, LAYER_5_PAD, LAYER_5_BATCH_NORM, LAYER_5_NEXT_PADD, LAYER_5_NEXT_STRIDE, LAYER_5_IGNORE_PADD, 0, LAYER_5_OFFSET, LAYER_5_SHIFT, LAYER_5_B_SHIFT);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Layer5 done in %f seconds\t%d values written to file\n", ((double) (end - start)) / CLOCKS_PER_SEC, item_count);
#endif
	
	//layer6 (104x84x64 -> 54x46x64)
	start = clock();
	maxpool_layer(LAYER_6_W, LAYER_6_H, LAYER_6_NUM_KER, LAYER_6_DOWNSAMPLE, LAYER_6_IGNORE_PADD, 0);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Layer6 done in %f seconds\n", ((double) (end - start)) / CLOCKS_PER_SEC);
#endif
			
	//layer7 (54x46x64 -> 52x44x128)
	start = clock();
	conv_layer(LAYER_7_W, LAYER_7_H, LAYER_7_C, LAYER_7_NUM_KER, LAYER_7_KER_SIZE, LAYER_7_PAD, LAYER_7_BATCH_NORM, LAYER_7_NEXT_PADD, LAYER_7_NEXT_STRIDE, LAYER_7_IGNORE_PADD, 0, LAYER_7_OFFSET, LAYER_7_SHIFT, LAYER_7_B_SHIFT);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Layer7 done in %f seconds\t%d values written to file\n", ((double) (end - start)) / CLOCKS_PER_SEC, item_count);
#endif
	
	//layer8 (52x44x128 -> 28x26x128)
	start = clock();
	maxpool_layer(LAYER_8_W, LAYER_8_H, LAYER_8_NUM_KER, LAYER_8_DOWNSAMPLE, LAYER_8_IGNORE_PADD, 0);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Layer8 done in %f seconds\n", ((double) (end - start)) / CLOCKS_PER_SEC);
#endif
	//layer9 (28x26x128 -> 28x28x256) -> Zero-padding
	//Result of layer 9 goes after result of layer 20
	start = clock();
	conv_layer(LAYER_9_W, LAYER_9_H, LAYER_9_C, LAYER_9_NUM_KER, LAYER_9_KER_SIZE, LAYER_9_PAD, LAYER_9_BATCH_NORM, LAYER_9_NEXT_PADD, LAYER_9_NEXT_STRIDE, LAYER_9_IGNORE_PADD, data_pos_layer9, LAYER_9_OFFSET, LAYER_9_SHIFT, LAYER_9_B_SHIFT);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Layer9 done in %f seconds\t%d values written to file\n", ((double) (end - start)) / CLOCKS_PER_SEC, item_count);
#endif
	
	//layer10 (28x28x256 -> 15x15x256) -> Ignores padding from layer 9
	start = clock();
	maxpool_layer(LAYER_10_W, LAYER_10_W, LAYER_10_NUM_KER, LAYER_10_DOWNSAMPLE, LAYER_10_IGNORE_PADD, data_pos_layer10);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Layer10 done in %f seconds\n", ((double) (end - start)) / CLOCKS_PER_SEC);
#endif
		
	//layer11 (15x15x256 -> 14x14x512)
	//Repeats last line and column of each feature map
	start = clock();
	conv_layer(LAYER_11_W, LAYER_11_W, LAYER_11_C, LAYER_11_NUM_KER, LAYER_11_KER_SIZE, LAYER_11_PAD, LAYER_11_BATCH_NORM, LAYER_11_NEXT_PADD, LAYER_11_NEXT_STRIDE, LAYER_11_IGNORE_PADD, 0, LAYER_11_OFFSET, LAYER_11_SHIFT, LAYER_11_B_SHIFT);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Layer11 done in %f seconds\t%d values written to file\n", ((double) (end - start)) / CLOCKS_PER_SEC, item_count);
#endif		
	
	//layer12 (14x14x512 -> 15x15x512)
	start = clock();
	maxpool_layer(LAYER_12_W, LAYER_12_W, LAYER_12_NUM_KER, LAYER_12_DOWNSAMPLE, LAYER_12_IGNORE_PADD, 0);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Layer12 done in %f seconds\n", ((double) (end - start)) / CLOCKS_PER_SEC);
#endif
	
	//layer13 (15x15x512 -> 13x13x1024)
	start = clock();
	conv_layer(LAYER_13_W, LAYER_13_W, LAYER_13_C, LAYER_13_NUM_KER, LAYER_13_KER_SIZE, LAYER_13_PAD, LAYER_13_BATCH_NORM, LAYER_13_NEXT_PADD, LAYER_13_NEXT_STRIDE, LAYER_13_IGNORE_PADD, 0, LAYER_13_OFFSET, LAYER_13_SHIFT, LAYER_13_B_SHIFT);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Layer13 done in %f seconds\t%d values written to file\n", ((double) (end - start)) / CLOCKS_PER_SEC, item_count);
#endif
	
	//layer14 (13x13x1024 -> 15x15x256)
	start = clock();
	conv_layer(LAYER_14_W, LAYER_14_W, LAYER_14_C, LAYER_14_NUM_KER, LAYER_14_KER_SIZE, LAYER_14_PAD, LAYER_14_BATCH_NORM, LAYER_14_NEXT_PADD, LAYER_14_NEXT_STRIDE, LAYER_14_IGNORE_PADD, 0, LAYER_14_OFFSET, LAYER_14_SHIFT, LAYER_14_B_SHIFT);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Layer14 done in %f seconds\t%d values written to file\n", ((double) (end - start)) / CLOCKS_PER_SEC, item_count);
#endif
		
	//layer15 (15x15x256 -> 13x13x512)
	start = clock();
	conv_layer(LAYER_15_W, LAYER_15_W, LAYER_15_C, LAYER_15_NUM_KER, LAYER_15_KER_SIZE, LAYER_15_PAD, LAYER_15_BATCH_NORM, LAYER_15_NEXT_PADD, LAYER_15_NEXT_STRIDE, LAYER_15_IGNORE_PADD, 0, LAYER_15_OFFSET, LAYER_15_SHIFT, LAYER_15_B_SHIFT);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Layer15 done in %f seconds\t%d values written to file\n", ((double) (end - start)) / CLOCKS_PER_SEC, item_count);
#endif
	
	//layer16 (13x13x512 -> 13x13x255)
	start = clock();
	conv_layer(LAYER_16_W, LAYER_16_W, LAYER_16_C, LAYER_16_NUM_KER, LAYER_16_KER_SIZE, LAYER_16_PAD, LAYER_16_BATCH_NORM, LAYER_16_NEXT_PADD, LAYER_16_NEXT_STRIDE, LAYER_16_IGNORE_PADD, 0, LAYER_16_OFFSET, LAYER_16_SHIFT, LAYER_16_B_SHIFT);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Layer16 done in %f seconds\t%d values written to file\n", ((double) (end - start)) / CLOCKS_PER_SEC, item_count);
#endif

	//layer17
	start = clock();
	yolo_layer(LAYER_17_W, yolo1_div, 1, data_pos, box_pos);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Layer17 done in %f seconds\n", ((double) (end - start)) / CLOCKS_PER_SEC);
#endif
		
	//layer19 (15x15x256 -> 13x13x128)
	data_pos = data_pos_layer14;
	start = clock();
	conv_layer(LAYER_19_W, LAYER_19_W, LAYER_19_C, LAYER_19_NUM_KER, LAYER_19_KER_SIZE, LAYER_19_PAD, LAYER_19_BATCH_NORM, LAYER_19_NEXT_PADD, LAYER_19_NEXT_STRIDE, LAYER_19_IGNORE_PADD, data_pos_layer19, LAYER_19_OFFSET, LAYER_19_SHIFT, LAYER_19_B_SHIFT);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Layer19 done in %f seconds\t", ((double) (end - start)) / CLOCKS_PER_SEC);
	printf("%d values written to file\n", item_count);
#endif

	//layer20 (13x13x128 -> 28x28x128)
	start = clock();
	upsample_layer(LAYER_20_W, LAYER_20_NUM_KER);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Layer20 done in %f seconds\n", ((double) (end - start)) / CLOCKS_PER_SEC);
#endif
	
	//layer22 (28x28x128 -> 26x26x256)
	//layer 21 (second route layer) is not needed as output of layer 9 is already after output of layer 20
	start = clock();
	conv_layer(LAYER_22_W, LAYER_22_W, LAYER_22_C, LAYER_22_NUM_KER, LAYER_22_KER_SIZE, LAYER_22_PAD, LAYER_22_BATCH_NORM, LAYER_22_NEXT_PADD, LAYER_22_NEXT_STRIDE, LAYER_22_IGNORE_PADD, 0, LAYER_22_OFFSET, LAYER_22_SHIFT, LAYER_22_B_SHIFT);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Layer22 done in %f seconds\t", ((double) (end - start)) / CLOCKS_PER_SEC);
	printf("%d values written to file\n", item_count);
#endif

	//layer23 (26x26x256 -> 26x26x255)
	start = clock();
	conv_layer(LAYER_23_W, LAYER_23_W, LAYER_23_C, LAYER_23_NUM_KER, LAYER_23_KER_SIZE, LAYER_23_PAD, LAYER_23_BATCH_NORM, LAYER_23_NEXT_PADD, LAYER_23_NEXT_STRIDE, LAYER_23_IGNORE_PADD, 0, LAYER_23_OFFSET, LAYER_23_SHIFT, LAYER_23_B_SHIFT);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
#ifndef mAP
	printf("Layer23 done in %f seconds\t", ((double) (end - start)) / CLOCKS_PER_SEC);
	printf("%d values written to file\n", item_count);
#endif

	//layer24
	start = clock();	
	yolo_layer(LAYER_24_W, yolo2_div, 0, data_pos, box_pos);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
	
#ifndef mAP
	printf("Layer24 done in %f seconds\n", ((double) (end - start)) / CLOCKS_PER_SEC);
#endif

	//get candidate boxes
	start = clock();
	filter_boxes(box_pos);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;

#ifdef mAP

	//Create .json file with results
	FILE *fp = 0;
	fp = fopen("mAP/coco_results.json", "a");
	print_cocos(fp, argv[1], box_pos);
    fclose(fp);

#else
	printf("Get candidate boxes done in %f seconds\n", ((double) (end - start)) / CLOCKS_PER_SEC);
	
	//draw detections
	start = clock();
	draw_detections(box_pos);
	end = clock();
	cpu_time_used += ((double) (end - start)) / CLOCKS_PER_SEC;
	printf("Draw detections done in %f seconds\n", ((double) (end - start)) / CLOCKS_PER_SEC);
	
	//print execution time
	printf("\nTotal execution time = %f seconds\n", cpu_time_used);
	
	//print results
	print_results(box_pos);
				
	//return data
	send_data(argv);
#endif
	fclose(results);
	return 0;
}
