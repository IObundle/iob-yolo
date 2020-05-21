//Input image
#define IMG_W 768
#define IMG_H 576
#define IMG_C 3
#define IMAGE_INPUT (IMG_W*IMG_H*IMG_C)

//Resized image
#define YOLO_INPUT 416
#if IMG_W>=IMG_H
  #define NEW_W YOLO_INPUT
  #define NEW_H ((IMG_H*NEW_W)/IMG_W)
  #define EXTRA_W 0
  #define EXTRA_H ((NEW_W-NEW_H)/2)
#else
  #define NEW_W ((IMG_W*NEW_H)/IMG_H)
  #define NEW_H YOLO_INPUT
  #define EXTRA_W ((NEW_H-NEW_W)/2)
  #define EXTRA_H 0
#endif

//Input network
#define NETWORK_INPUT ((YOLO_INPUT+2)*(YOLO_INPUT+2)*IMG_C)

//Layer1 (conv)
#define LAYER_1_NUM_KER 16
#define LAYER_1_KER_SIZE 3
#define DATA_LAYER_1 (YOLO_INPUT*YOLO_INPUT*LAYER_1_NUM_KER)
#define WEIGHTS_LAYER_1 (LAYER_1_NUM_KER*2+LAYER_1_NUM_KER*LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*IMG_C)

//Total
#define TOTAL_WEIGHTS (WEIGHTS_LAYER_1)
