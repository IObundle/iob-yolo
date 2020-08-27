//Layer1,2 (conv + maxpool)
#define LAYER_1_INPADD 1
#define LAYER_1_IGNOREPAD 0
#define LAYER_1_STRIDE 0
#define LAYER_1_OUTPADD 1
#define LAYER_1_MAXPOOL 1
#define LAYER_1_W 416
#define LAYER_1_NUM_KER 16
#define LAYER_1_KER_SIZE 3
#define LAYER_1_C 3
#define LAYER_1_P_OFF 10
#define LAYER_1_W_OFF 5
#define DATA_LAYER_1 ((LAYER_1_W+2)*((LAYER_1_W+2)*LAYER_1_C+LAYER_1_P_OFF))
#define WEIGHTS_LAYER_1 (LAYER_1_NUM_KER+LAYER_1_NUM_KER*(LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*LAYER_1_C+LAYER_1_W_OFF))
#define LAYER_2_W 210
#define DATA_LAYER_2 (LAYER_2_W*LAYER_2_W*LAYER_1_NUM_KER)

//Layer3,4 (conv + maxpool)
#define LAYER_3_IGNOREPAD 0
#define LAYER_3_STRIDE 0
#define LAYER_3_OUTPADD 1
#define LAYER_3_MAXPOOL 1
#define LAYER_3_W 208
#define LAYER_3_NUM_KER 32
#define LAYER_3_KER_SIZE 3
#define WEIGHTS_LAYER_3 (LAYER_3_NUM_KER+LAYER_3_NUM_KER*LAYER_3_KER_SIZE*LAYER_3_KER_SIZE*LAYER_1_NUM_KER)
#define LAYER_4_W 106
#define DATA_LAYER_4 (LAYER_4_W*LAYER_4_W*LAYER_3_NUM_KER)

//Layer5,6 (conv + maxpool)
#define LAYER_5_IGNOREPAD 0
#define LAYER_5_STRIDE 0
#define LAYER_5_OUTPADD 1
#define LAYER_5_MAXPOOL 1
#define LAYER_5_W 104
#define LAYER_5_NUM_KER 64
#define LAYER_5_KER_SIZE 3
#define WEIGHTS_LAYER_5 (LAYER_5_NUM_KER+LAYER_5_NUM_KER*LAYER_5_KER_SIZE*LAYER_5_KER_SIZE*LAYER_3_NUM_KER)
#define LAYER_6_W 54
#define DATA_LAYER_6 (LAYER_6_W*LAYER_6_W*LAYER_5_NUM_KER)

//Layer7,8 (conv + maxpool)
#define LAYER_7_IGNOREPAD 0
#define LAYER_7_STRIDE 0
#define LAYER_7_OUTPADD 1
#define LAYER_7_MAXPOOL 1
#define LAYER_7_W 52
#define LAYER_7_NUM_KER 128
#define LAYER_7_KER_SIZE 3
#define WEIGHTS_LAYER_7 (LAYER_7_NUM_KER+LAYER_7_NUM_KER*LAYER_7_KER_SIZE*LAYER_7_KER_SIZE*LAYER_5_NUM_KER)
#define LAYER_8_W 28
#define DATA_LAYER_8 (LAYER_8_W*LAYER_8_W*LAYER_7_NUM_KER)

//Layer9 (conv)
#define LAYER_9_IGNOREPAD 0
#define LAYER_9_STRIDE 0
#define LAYER_9_OUTPADD 1
#define LAYER_9_MAXPOOL 0
#define LAYER_9_W 26
#define LAYER_9_NUM_KER 256
#define LAYER_9_KER_SIZE 3
#define WEIGHTS_LAYER_9 (LAYER_9_NUM_KER+LAYER_9_NUM_KER*LAYER_9_KER_SIZE*LAYER_9_KER_SIZE*LAYER_7_NUM_KER)
#define DATA_LAYER_9 ((LAYER_9_W+2)*(LAYER_9_W+2)*LAYER_9_NUM_KER)

//Layer10 (maxpool)
#define LAYER_10_INPADD 1
#define LAYER_10_STRIDE 0
#define LAYER_10_W 15
#define DATA_LAYER_10 (LAYER_10_W*LAYER_10_W*LAYER_9_NUM_KER)

//Layer11 (conv)
#define LAYER_11_IGNOREPAD 0
#define LAYER_11_STRIDE 1
#define LAYER_11_OUTPADD 0
#define LAYER_11_MAXPOOL 0
#define LAYER_11_W 13
#define LAYER_11_NUM_KER 512
#define LAYER_11_KER_SIZE 3
#define WEIGHTS_LAYER_11 (LAYER_11_NUM_KER+LAYER_11_NUM_KER*LAYER_11_KER_SIZE*LAYER_11_KER_SIZE*LAYER_9_NUM_KER)
#define DATA_LAYER_11 ((LAYER_11_W+1)*(LAYER_11_W+1)*LAYER_11_NUM_KER)

//Layer12 (maxpool)
#define LAYER_12_INPADD 0
#define LAYER_12_STRIDE 1
#define LAYER_12_W 15
#define DATA_LAYER_12 (LAYER_12_W*LAYER_12_W*LAYER_11_NUM_KER)

//Layer13 (conv)
#define LAYER_13_IGNOREPAD 0
#define LAYER_13_STRIDE 0
#define LAYER_13_OUTPADD 0
#define LAYER_13_MAXPOOL 0
#define LAYER_13_W 13
#define LAYER_13_NUM_KER 1024
#define LAYER_13_KER_SIZE 3
#define WEIGHTS_LAYER_13 (LAYER_13_NUM_KER+LAYER_13_NUM_KER*LAYER_13_KER_SIZE*LAYER_13_KER_SIZE*LAYER_11_NUM_KER)
#define DATA_LAYER_13 (LAYER_13_W*LAYER_13_W*LAYER_13_NUM_KER)

//Layer14 (conv)
#define LAYER_14_IGNOREPAD 0
#define LAYER_14_STRIDE 0
#define LAYER_14_OUTPADD 1
#define LAYER_14_MAXPOOL 0
#define LAYER_14_W 13
#define LAYER_14_NUM_KER 256
#define LAYER_14_KER_SIZE 1
#define WEIGHTS_LAYER_14 (LAYER_14_NUM_KER+LAYER_14_NUM_KER*LAYER_14_KER_SIZE*LAYER_14_KER_SIZE*LAYER_13_NUM_KER)
#define DATA_LAYER_14 ((LAYER_14_W+2)*(LAYER_14_W+2)*LAYER_14_NUM_KER)

//Layer15 (conv)
#define LAYER_15_IGNOREPAD 0
#define LAYER_15_STRIDE 0
#define LAYER_15_OUTPADD 0
#define LAYER_15_MAXPOOL 0
#define LAYER_15_W 13
#define LAYER_15_NUM_KER 512
#define LAYER_15_KER_SIZE 3
#define WEIGHTS_LAYER_15 (LAYER_15_NUM_KER+LAYER_15_NUM_KER*LAYER_15_KER_SIZE*LAYER_15_KER_SIZE*LAYER_14_NUM_KER)
#define DATA_LAYER_15 (LAYER_15_W*LAYER_15_W*LAYER_15_NUM_KER)

//Layer16 (conv)
#define LAYER_16_IGNOREPAD 0
#define LAYER_16_STRIDE 0
#define LAYER_16_OUTPADD 0
#define LAYER_16_MAXPOOL 0
#define LAYER_16_W 13
#define LAYER_16_NUM_KER 256
#define LAYER_16_KER_SIZE 1
#define WEIGHTS_LAYER_16 (LAYER_16_NUM_KER+LAYER_16_NUM_KER*LAYER_16_KER_SIZE*LAYER_16_KER_SIZE*LAYER_15_NUM_KER)
#define DATA_LAYER_16 (LAYER_16_W*LAYER_16_W*LAYER_16_NUM_KER)

//Total
#define TOTAL_WEIGHTS (WEIGHTS_LAYER_1 + WEIGHTS_LAYER_3 + WEIGHTS_LAYER_5 + WEIGHTS_LAYER_7 + WEIGHTS_LAYER_9 + WEIGHTS_LAYER_11 + WEIGHTS_LAYER_13 + WEIGHTS_LAYER_14 + WEIGHTS_LAYER_15 + WEIGHTS_LAYER_16)
