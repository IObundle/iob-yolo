//Layer1,2 (conv + maxpool)
#define LAYER_1_W 416
#define LAYER_1_NUM_KER 16
#define LAYER_1_KER_SIZE 3
#if nYOLOmacs == 1
  #define LAYER_1_C 3
  #define LAYER_1_P_OFF 10
  #define LAYER_1_W_OFF 5
#elif ((nYOLOmacs==2)||(nYOLOmacs==4))
  #define LAYER_1_C 4
  #define LAYER_1_P_OFF 8
  #define LAYER_1_W_OFF 12
#else //nYOLOmacs==8
  #define LAYER_1_C 8
  #define LAYER_1_P_OFF 0
  #define LAYER_1_W_OFF 8
#endif
#define DATA_LAYER_1 ((LAYER_1_W+2)*((LAYER_1_W+2)*LAYER_1_C+LAYER_1_P_OFF))
#define WEIGHTS_LAYER_1 (LAYER_1_NUM_KER+LAYER_1_NUM_KER*(LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*LAYER_1_C+LAYER_1_W_OFF))
#define LAYER_2_W 210
#define DATA_LAYER_2 (LAYER_2_W*LAYER_2_W*LAYER_1_NUM_KER)
#define LAYER_1_SHIFT 17
#define LAYER_1_B_SHIFT (16-14)

//Layer3,4 (conv + maxpool)
#define LAYER_3_W 208
#define LAYER_3_NUM_KER 32
#define LAYER_3_KER_SIZE 3
#define WEIGHTS_LAYER_3 (LAYER_3_NUM_KER+LAYER_3_NUM_KER*LAYER_3_KER_SIZE*LAYER_3_KER_SIZE*LAYER_1_NUM_KER)
#define LAYER_4_W 106
#define DATA_LAYER_4 (LAYER_4_W*LAYER_4_W*LAYER_3_NUM_KER)
#define LAYER_3_SHIFT 14
#define LAYER_3_B_SHIFT (16-11)

//Layer5,6 (conv + maxpool)
#define LAYER_5_W 104
#define LAYER_5_NUM_KER 64
#define LAYER_5_KER_SIZE 3
#define WEIGHTS_LAYER_5 (LAYER_5_NUM_KER+LAYER_5_NUM_KER*LAYER_5_KER_SIZE*LAYER_5_KER_SIZE*LAYER_3_NUM_KER)
#define LAYER_6_W 54
#define DATA_LAYER_6 (LAYER_6_W*LAYER_6_W*LAYER_5_NUM_KER)
#define LAYER_5_SHIFT 14
#define LAYER_5_B_SHIFT (16-12)

//Layer7,8 (conv + maxpool)
#define LAYER_7_OUTPADD 1
#define LAYER_7_W 52
#define LAYER_7_NUM_KER 128
#define LAYER_7_KER_SIZE 3
#define WEIGHTS_LAYER_7 (LAYER_7_NUM_KER+LAYER_7_NUM_KER*LAYER_7_KER_SIZE*LAYER_7_KER_SIZE*LAYER_5_NUM_KER)
#define LAYER_8_W 28
#define DATA_LAYER_8 (LAYER_8_W*LAYER_8_W*LAYER_7_NUM_KER)
#define LAYER_7_SHIFT 15
#define LAYER_7_B_SHIFT (16-12)

//Layer9 (conv)
#define LAYER_9_PINGPONG 1
#define LAYER_9_ZXY 1
#define LAYER_9_LEAKY 1
#define LAYER_9_UPSAMPLE 0
#define LAYER_9_IGNOREPAD 0
#define LAYER_9_STRIDE 0
#define LAYER_9_OUTPADD 1
#define LAYER_9_W 26
#define LAYER_9_NUM_KER 256
#define LAYER_9_KER_SIZE 3
#define WEIGHTS_LAYER_9 (LAYER_9_NUM_KER+LAYER_9_NUM_KER*LAYER_9_KER_SIZE*LAYER_9_KER_SIZE*LAYER_7_NUM_KER)
#define DATA_LAYER_9 ((LAYER_9_W+2)*(LAYER_9_W+2)*LAYER_9_NUM_KER)
#define LAYER_9_SHIFT 15
#define LAYER_9_B_SHIFT (16-12)

//Layer10 (maxpool)
#define LAYER_10_OUTPADD 1
#define LAYER_10_INPADD 1
#define LAYER_10_STRIDE 0
#define LAYER_10_W 15
#define DATA_LAYER_10 (LAYER_10_W*LAYER_10_W*LAYER_9_NUM_KER)

//Layer11 (conv)
#define LAYER_11_PINGPONG 0
#define LAYER_11_ZXY 1
#define LAYER_11_LEAKY 1
#define LAYER_11_UPSAMPLE 0
#define LAYER_11_IGNOREPAD 0
#define LAYER_11_STRIDE 1
#define LAYER_11_OUTPADD 0
#define LAYER_11_W 13
#define LAYER_11_NUM_KER 512
#define LAYER_11_KER_SIZE 3
#define WEIGHTS_LAYER_11 (LAYER_11_NUM_KER+LAYER_11_NUM_KER*LAYER_11_KER_SIZE*LAYER_11_KER_SIZE*LAYER_9_NUM_KER)
#define DATA_LAYER_11 ((LAYER_11_W+1)*(LAYER_11_W+1)*LAYER_11_NUM_KER)
#define LAYER_11_SHIFT 15
#define LAYER_11_B_SHIFT (16-12)

//Layer12 (maxpool)
#define LAYER_12_OUTPADD 1
#define LAYER_12_INPADD 0
#define LAYER_12_STRIDE 1
#define LAYER_12_W 15
#define DATA_LAYER_12 (LAYER_12_W*LAYER_12_W*LAYER_11_NUM_KER)

//Layer13 (conv)
#define LAYER_13_PINGPONG 0
#define LAYER_13_ZXY 1
#define LAYER_13_LEAKY 1
#define LAYER_13_UPSAMPLE 0
#define LAYER_13_IGNOREPAD 0
#define LAYER_13_STRIDE 0
#define LAYER_13_OUTPADD 0
#define LAYER_13_W 13
#define LAYER_13_NUM_KER 1024
#define LAYER_13_KER_SIZE 3
#define WEIGHTS_LAYER_13 (LAYER_13_NUM_KER+LAYER_13_NUM_KER*LAYER_13_KER_SIZE*LAYER_13_KER_SIZE*LAYER_11_NUM_KER)
#define DATA_LAYER_13 (LAYER_13_W*LAYER_13_W*LAYER_13_NUM_KER)
#define LAYER_13_SHIFT 15
#define LAYER_13_B_SHIFT (16-11)

//Layer14 (conv)
#define LAYER_14_PINGPONG 0
#define LAYER_14_ZXY 0
#define LAYER_14_LEAKY 1
#define LAYER_14_UPSAMPLE 0
#define LAYER_14_IGNOREPAD 0
#define LAYER_14_STRIDE 0
#define LAYER_14_OUTPADD 1
#define LAYER_14_W 13
#define LAYER_14_NUM_KER 256
#define LAYER_14_KER_SIZE 1
#define WEIGHTS_LAYER_14 (LAYER_14_NUM_KER+LAYER_14_NUM_KER*LAYER_14_KER_SIZE*LAYER_14_KER_SIZE*LAYER_13_NUM_KER)
#define DATA_LAYER_14 ((LAYER_14_W+2)*(LAYER_14_W+2)*LAYER_14_NUM_KER)
#define LAYER_14_SHIFT 13
#define LAYER_14_B_SHIFT (16-9)

//Layer15 (conv)
#define LAYER_15_PINGPONG 0
#define LAYER_15_ZXY 0
#define LAYER_15_LEAKY 1
#define LAYER_15_UPSAMPLE 0
#define LAYER_15_IGNOREPAD 0
#define LAYER_15_STRIDE 0
#define LAYER_15_OUTPADD 0
#define LAYER_15_W 13
#define LAYER_15_NUM_KER 512
#define LAYER_15_KER_SIZE 3
#define WEIGHTS_LAYER_15 (LAYER_15_NUM_KER+LAYER_15_NUM_KER*LAYER_15_KER_SIZE*LAYER_15_KER_SIZE*LAYER_14_NUM_KER)
#define DATA_LAYER_15 (LAYER_15_W*LAYER_15_W*LAYER_15_NUM_KER)
#define LAYER_15_SHIFT 15
#define LAYER_15_B_SHIFT (16-12)

//Layer16 (conv + yolo)
#define LAYER_16_PINGPONG 0
#define LAYER_16_ZXY 0
#define LAYER_16_LEAKY 0
#define LAYER_16_UPSAMPLE 0
#define LAYER_16_IGNOREPAD 0
#define LAYER_16_STRIDE 0
#define LAYER_16_OUTPADD 0
#define LAYER_16_W 13
#define LAYER_16_NUM_KER 256
#define LAYER_16_KER_SIZE 1
#define WEIGHTS_LAYER_16 (LAYER_16_NUM_KER+LAYER_16_NUM_KER*LAYER_16_KER_SIZE*LAYER_16_KER_SIZE*LAYER_15_NUM_KER)
#define DATA_LAYER_16 (LAYER_16_W*LAYER_16_W*LAYER_16_NUM_KER)
#define LAYER_16_SHIFT 11
#define LAYER_16_B_SHIFT (16-11)

//Layer19 (conv + upsample)
#define LAYER_19_PINGPONG 1
#define LAYER_19_ZXY 0
#define LAYER_19_LEAKY 1
#define LAYER_19_UPSAMPLE 1
#define LAYER_19_IGNOREPAD 1
#define LAYER_19_STRIDE 0
#define LAYER_19_OUTPADD 1
#define LAYER_19_W 13
#define LAYER_19_NUM_KER 128
#define LAYER_19_KER_SIZE 1
#define WEIGHTS_LAYER_19 (LAYER_19_NUM_KER+LAYER_19_NUM_KER*LAYER_19_KER_SIZE*LAYER_19_KER_SIZE*LAYER_14_NUM_KER)
#define DATA_LAYER_19 ((LAYER_19_W*2+2)*(LAYER_19_W*2+2)*LAYER_19_NUM_KER)
#define LAYER_19_SHIFT 15
#define LAYER_19_B_SHIFT (16-11)

//Layer22 (conv)
#define LAYER_22_PINGPONG 0
#define LAYER_22_ZXY 0
#define LAYER_22_LEAKY 1
#define LAYER_22_UPSAMPLE 0
#define LAYER_22_IGNOREPAD 0
#define LAYER_22_STRIDE 0
#define LAYER_22_OUTPADD 0
#define LAYER_22_W 26
#define LAYER_22_NUM_KER 256
#define LAYER_22_KER_SIZE 3
#define WEIGHTS_LAYER_22 (LAYER_22_NUM_KER+LAYER_22_NUM_KER*LAYER_22_KER_SIZE*LAYER_22_KER_SIZE*(LAYER_9_NUM_KER+LAYER_19_NUM_KER))
#define DATA_LAYER_22 (LAYER_22_W*LAYER_22_W*LAYER_22_NUM_KER)
#define LAYER_22_SHIFT 14
#define LAYER_22_B_SHIFT (16-11)

//Layer23 (conv + yolo)
#define LAYER_23_PINGPONG 1
#define LAYER_23_ZXY 0
#define LAYER_23_LEAKY 0
#define LAYER_23_UPSAMPLE 0
#define LAYER_23_IGNOREPAD 0
#define LAYER_23_STRIDE 0
#define LAYER_23_OUTPADD 0
#define LAYER_23_W 26
#define LAYER_23_NUM_KER 256
#define LAYER_23_KER_SIZE 1
#define WEIGHTS_LAYER_23 (LAYER_23_NUM_KER+LAYER_23_NUM_KER*LAYER_23_KER_SIZE*LAYER_23_KER_SIZE*LAYER_22_NUM_KER)
#define DATA_LAYER_23 (LAYER_23_W*LAYER_23_W*LAYER_23_NUM_KER)
#define LAYER_23_SHIFT 11
#define LAYER_23_B_SHIFT (16-12)

//Total
#ifdef SIM
  #define TOTAL_WEIGHTS (WEIGHTS_LAYER_23)
#else
  #define TOTAL_WEIGHTS (WEIGHTS_LAYER_1 + WEIGHTS_LAYER_3 + WEIGHTS_LAYER_5 + WEIGHTS_LAYER_7 + WEIGHTS_LAYER_9 + WEIGHTS_LAYER_11 + WEIGHTS_LAYER_13 + WEIGHTS_LAYER_14 + WEIGHTS_LAYER_15 + WEIGHTS_LAYER_16 + WEIGHTS_LAYER_19 + WEIGHTS_LAYER_22 + WEIGHTS_LAYER_23)
#endif
