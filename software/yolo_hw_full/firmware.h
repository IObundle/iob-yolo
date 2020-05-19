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
#define NTW_IN_W YOLO_INPUT
#define NTW_IN_H YOLO_INPUT
#define NTW_IN_C IMG_C
#define NTW_IN_PAD 1
#define NETWORK_INPUT ((NTW_IN_W+2*NTW_IN_PAD)*(NTW_IN_H+2*NTW_IN_PAD)*NTW_IN_C)
