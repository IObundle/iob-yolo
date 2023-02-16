/*
embedded_gpu.c
source file for functions related with GPU execution of the darknet
*/
#ifdef GPU

#include "cuda_runtime.h"
#include "curand.h"
#include "cublas_v2.h"

#include "embedded.h"

//gpu index
int gpu_index;

// cuda functions
void cuda_set_device(int n)
{
    gpu_index = n;
    cudaError_t status = cudaSetDevice(n);
    check_error(status);
}

int cuda_get_device()
{
    int n = 0;
    cudaError_t status = cudaGetDevice(&n);
    check_error(status);
    return n;
}


void error(const char *s)
{
    perror(s);
    assert(0);
    exit(-1);
}


void check_error(cudaError_t status)
{
    //cudaDeviceSynchronize();
    cudaError_t status2 = cudaGetLastError();
    if (status != cudaSuccess)
    {   
        const char *s = cudaGetErrorString(status);
        char buffer[256];
        printf("CUDA Error: %s\n", s);
        assert(0);
        snprintf(buffer, 256, "CUDA Error: %s", s);
        error(buffer);
    } 
    if (status2 != cudaSuccess)
    {   
        const char *s = cudaGetErrorString(status);
        char buffer[256];
        printf("CUDA Error Prev: %s\n", s);
        assert(0);
        snprintf(buffer, 256, "CUDA Error Prev: %s", s);
        error(buffer);
    } 
}

cublasHandle_t blas_handle()
{
    static int init[16] = {0};
    static cublasHandle_t handle[16];
    int i = cuda_get_device();
    if(!init[i]) {
        cublasCreate(&handle[i]);
        init[i] = 1;
    }
    return handle[i];
}

dim3 cuda_gridsize(size_t n){
    size_t k = (n-1) / BLOCK + 1;
    size_t x = k;
    size_t y = 1;
    if(x > 65535){
        x = ceil(sqrt(k));
        y = (n-1)/(x*BLOCK) + 1;
    }
    dim3 d = {x, y, 1};
    //printf("%ld %ld %ld %ld\n", n, x, y, x*y*BLOCK);
    return d;
}

float *cuda_make_array(float *x, size_t n)
{
    float *x_gpu;
    size_t size = sizeof(float)*n;
    cudaError_t status = cudaMalloc((void **)&x_gpu, size);
    check_error(status);
    if(x){
        status = cudaMemcpy(x_gpu, x, size, cudaMemcpyHostToDevice);
        check_error(status);
    } else {
        fill_gpu(n, 0, x_gpu, 1);
    }
    if(!x_gpu) error("Cuda malloc failed\n");
    return x_gpu;
}


int16_t *cuda_make_fp_array(int16_t *x, size_t n)
{
  int16_t *x_gpu;
  size_t size = sizeof(int16_t)*n;
  cudaError_t status = cudaMalloc((void**)&x_gpu, size);
  check_error(status);
  if(x){
    status = cudaMemcpy(x_gpu, x, size, cudaMemcpyHostToDevice);
    check_error(status);
  } else {
    fill_fp_gpu(n, 0, x_gpu, 1);
  }
  if(!x_gpu) error("Cuda malloc failed\n");
  return x_gpu;
}


void cuda_free(void *x_gpu)
{
    cudaError_t status = cudaFree(x_gpu);
    check_error(status);
}

void cuda_pull_array(float *x_gpu, float *x, size_t n)
{
    size_t size = sizeof(float)*n;
    cudaError_t status = cudaMemcpy(x, x_gpu, size, cudaMemcpyDeviceToHost);
    check_error(status);
}

void cuda_pull_fp_array(int16_t *x_gpu, int16_t* x, size_t n)
{
  size_t size = sizeof(int16_t)*n;
  cudaError_t status = cudaMemcpy(x, x_gpu, size, cudaMemcpyDeviceToHost);
  check_error(status);
}

void free_net_output_gpu(void** net_output_gpu, int size)
{
  int i;
  for(i=0;i<size;i++){
    if(net_output_gpu[i] != NULL)
      cuda_free(net_output_gpu[i]);
  }
  return;
}

void gemm_gpu(int TA, int TB, int M, int N, int K, float ALPHA, 
        float *A_gpu, int lda, 
        float *B_gpu, int ldb,
        float BETA,
        float *C_gpu, int ldc)
{
    cublasHandle_t handle = blas_handle();
    cudaError_t status = cublasSgemm(handle, (TB ? CUBLAS_OP_T : CUBLAS_OP_N), 
            (TA ? CUBLAS_OP_T : CUBLAS_OP_N), N, M, K, &ALPHA, B_gpu, ldb, A_gpu, lda, &BETA, C_gpu, ldc);
    check_error(status);
}


#ifdef FIXED
//convolution functions
/*
used arguments: w, h, num_ker, ker_size, batch_norm, net_output_gpu, layer_num, fp_weights, weight_pos
arguments not used: pad, nextPadding, nextStride, ignorePadding, new_output_pos, offset
*/
void forward_convolutional_layer_gpu(int w, int h, int c, int num_ker, int ker_size, int pad, int batch_norm, int nextPadding, int nextStride, int ignorePadding, unsigned int new_output_pos, unsigned int offset, void** net_output_gpu, int layer_num, int16_t* fp_weights, unsigned int* weight_pos)
{
  //aux variables
  int16_t * weights_fp_gpu = (int16_t*) net_output_gpu[NET_GPU_WEIGHTS] + *weight_pos; // Q6.10
  int16_t * biases_fp_gpu = (int16_t*) net_output_gpu[NET_GPU_WEIGHTS] + *weight_pos + (ker_size*ker_size*c*num_ker); // Q8.8
  
  int m = num_ker; //number of kernels
  int k = ker_size*ker_size*c; //size of each kernel
    
  // return (l.h + 2*l.pad - l.size) / l.stride + 1; | pad=ker_size/2, size={1,3}, stride=1 - check original make_convolutional_layer
  int out_h = h;
  int out_w = w;
  int n = out_w*out_h; //output feature map - was l.out_w*l.out_h

  // create layer output array
  net_output_gpu[layer_num] = (void*) cuda_make_fp_array(0, out_h*out_w*num_ker);

  // pointer alias
  int16_t *a = weights_fp_gpu;
  int16_t* b = (int16_t*) net_output_gpu[NET_GPU_WORKSPACE];
  int16_t* out = (int16_t*) net_output_gpu[layer_num];
  int16_t* im = (int16_t*) net_output_gpu[layer_num-1];

  if (ker_size == 1){
    b = im;
  } else {
    im2col_fp_gpu(im, c, h, w, ker_size, 1, 1, b);
  }
  gemm_fp_gpu(a, b, out, m, n, k);

  //add bias
  add_bias_fp_gpu( (int16_t*) net_output_gpu[layer_num], biases_fp_gpu, 1, num_ker, out_w*out_h, 1); //bias from Q8.8 -> Q9.7 (bias >> 1)

  if (batch_norm) {
    //batch normalization folding - only need to add bias
    //tiny only used leaky activation, when batch_norm is done
    activate_array_gpu((int16_t*) net_output_gpu[layer_num], out_w*out_h*num_ker, LEAKY );
  }

  //update weight_pos
  *weight_pos += num_ker + num_ker*c*ker_size*ker_size;

}

//YOLO layer
void forward_yolo_layer_gpu(int w, void** net_output_gpu, int layer_num, int16_t* fp_data, int yolo_layer_output_pos)
{
    //create output array
    // No yolo layer output is created, input is copied to CPU side

    //yolo output pointer
    int16_t * yolo_output_ptr = fp_data + yolo_layer_output_pos;

    //save yolo INPUT to cpu side
    cuda_pull_fp_array((int16_t*) net_output_gpu[layer_num-1], yolo_output_ptr, w*w*255);

    return;
}

//ROUTE layer
 void forward_route_layer_gpu(int inputs_num, int* input_layers, int* input_sizes, void** net_output_gpu, int layer_num)
{
    int i;
    int offset = 0, total_out_size=0;
    
    //create output array
    for(i=0; i< inputs_num;i++){
      total_out_size+=input_sizes[i];
    }
    net_output_gpu[layer_num] = (void*) cuda_make_fp_array(0, total_out_size);

    //copy all input layers to output
    for(i = 0; i < inputs_num; ++i){
        int index = input_layers[i];
        int16_t *input = (int16_t*) net_output_gpu[index];
        int input_size = input_sizes[i];

	copy_fp_gpu(input_size, input, 1, (int16_t*) net_output_gpu[layer_num] + offset, 1);       
        offset += input_size;
    }
}

//UPSAMPLE layer
void forward_upsample_layer_gpu(int w, int num_ker, void ** net_output_gpu, int layer_num)
{
  //create output array
  net_output_gpu[layer_num] = (void*) cuda_make_fp_array(0, (w*2)*(w*2)*num_ker);

  //stride = 2, scale = 1
  upsample_gpu((int16_t*) net_output_gpu[layer_num-1], w, w, num_ker, 1, 2, 1, 1, (int16_t*) net_output_gpu[layer_num]);
}




#else //Float
//convolution functions
/*
used arguments: w, h, num_ker, ker_size, batch_norm, net_output_gpu, layer_num, fp_weights, weight_pos
arguments not used: pad, nextPadding, nextStride, ignorePadding, new_output_pos, offset
*/
void forward_convolutional_layer_gpu(int w, int h, int c, int num_ker, int ker_size, int pad, int batch_norm, int nextPadding, int nextStride, int ignorePadding, unsigned int new_output_pos, unsigned int offset, void** net_output_gpu, int layer_num, float* fp_weights, unsigned int* weight_pos)
{
  //aux variables
  float * weights_gpu = (float*) net_output_gpu[NET_GPU_WEIGHTS] + *weight_pos;
  float * biases_gpu = (float*) net_output_gpu[NET_GPU_WEIGHTS] + *weight_pos + (ker_size*ker_size*c*num_ker);

  
  int m = num_ker; //number of kernels
  int k = ker_size*ker_size*c; //size of each kernel
    
  // return (l.h + 2*l.pad - l.size) / l.stride + 1; | pad=ker_size/2, size={1,3}, stride=1 - check original make_convolutional_layer
  int out_h = h;
  int out_w = w;
  int n = out_w*out_h; //output feature map - was l.out_w*l.out_h

  net_output_gpu[layer_num] = (void*) cuda_make_array(0, out_h*out_w*num_ker); //if 1st arg=0, sets fills array with 0    

  // pointer alias
  float *a = weights_gpu;
  float *b = (float*) net_output_gpu[NUM_LAYERS+1];
  float *out = (float*) net_output_gpu[layer_num];
  float *im = (float*) net_output_gpu[layer_num-1];

  if (ker_size == 1){
    b = im;
  } else {
    im2col_gpu(im, c, h, w, ker_size, 1, 1, b);
  }
  gemm_gpu(0,0,m,n,k,1,a,k,b,n,1,out,n);

  // bias is always added, batch_norm or not
  add_bias_gpu((float*) net_output_gpu[layer_num], biases_gpu, 1, num_ker, out_w*out_h);

  if (batch_norm) {
    //forward_batchnorm_layer_gpu(l, net);
    //batch normalization folding - only need to add bias
    //tiny only used leaky activation, when batch_norm is done
    activate_array_gpu((float*) net_output_gpu[layer_num], out_w*out_h*num_ker, LEAKY );
  }

  //update weight_pos
  *weight_pos += num_ker + num_ker*c*ker_size*ker_size;

}

//YOLO layer
void forward_yolo_layer_gpu(int w, void** net_output_gpu, int layer_num, float* fp_data, int yolo_layer_output_pos)
{
  //create output array
  // No yolo layer output is created, input is copied to CPU side

  //yolo output pointer
  float * yolo_output_ptr = fp_data + yolo_layer_output_pos;

  //save yolo INPUT to cpu side
  cuda_pull_array( (float*) net_output_gpu[layer_num-1], yolo_output_ptr, w*w*255);

  return;
}

//ROUTE layer
 void forward_route_layer_gpu(int inputs_num, int* input_layers, int* input_sizes, void** net_output_gpu, int layer_num)
{
    int i;
    int offset = 0, total_out_size=0;
    
    //create output array
    for(i=0; i< inputs_num;i++){
      total_out_size+=input_sizes[i];
    }
    net_output_gpu[layer_num] = (void*) cuda_make_array(0, total_out_size);

    //copy all input layers to output
    for(i = 0; i < inputs_num; ++i){
        int index = input_layers[i];
        float *input = (float*) net_output_gpu[index];
        int input_size = input_sizes[i];

	copy_gpu(input_size, input, 1, (float*) net_output_gpu[layer_num] + offset, 1);       
        offset += input_size;
    }
}


//UPSAMPLE layer
void forward_upsample_layer_gpu(int w, int num_ker, void ** net_output_gpu, int layer_num)
{
  //create output array
  net_output_gpu[layer_num] = (void*) cuda_make_array(0, (w*2)*(w*2)*num_ker);

  //stride = 2, scale = 1
  upsample_gpu( (float*) net_output_gpu[layer_num-1], w, w, num_ker, 1, 2, 1, 1, (float*) net_output_gpu[layer_num]);
}
#endif //ifdef FIXED

#endif //ifdef GPU
