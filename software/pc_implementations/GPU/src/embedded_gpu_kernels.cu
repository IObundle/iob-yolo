/*
embedded_gpu_kernels.cu

implementations of GPU kernels for inference

*/

#ifdef GPU

#include "cuda_runtime.h"
#include "curand.h"
#include "cublas_v2.h"

extern "C"{
#include "embedded.h"
}

__global__ void fill_kernel(int N, float ALPHA, float *X, int INCX)
{
    int i = (blockIdx.x + blockIdx.y*gridDim.x) * blockDim.x + threadIdx.x;
    if(i < N) X[i*INCX] = ALPHA;
}

void fill_gpu(int N, float ALPHA, float * X, int INCX)
{
    fill_kernel<<<cuda_gridsize(N), BLOCK>>>(N, ALPHA, X, INCX);
    check_error(cudaPeekAtLastError());
}

__global__ void fill_fp_kernel(int N, int16_t ALPHA, int16_t *X, int INCX)
{
    int i = (blockIdx.x + blockIdx.y*gridDim.x) * blockDim.x + threadIdx.x;
    if(i < N) X[i*INCX] = ALPHA;
}

void fill_fp_gpu(int N, int16_t ALPHA, int16_t * X, int INCX)
{
    fill_fp_kernel<<<cuda_gridsize(N), BLOCK>>>(N, ALPHA, X, INCX);
    check_error(cudaPeekAtLastError());
}

#ifdef FIXED

__global__ void copy_fp2float_array_kernel(int N, int16_t *X, float *Y, int fracFP)
{
  int i = (blockIdx.x + blockIdx.y*gridDim.x) * blockDim.x + threadIdx.x;
  if(i < N) Y[i] = (float) ((X[i]*1.0)/((int32_t)(1<<fracFP)));
}

void copy_fp2float_array_gpu(int N, int16_t *X, float* Y, int fracFP)
{
  copy_fp2float_array_kernel<<<cuda_gridsize(N), BLOCK>>>(N, X, Y, fracFP);
  check_error(cudaPeekAtLastError());
}


__global__ void copy_float2fp_array_kernel(int N, float *X, int16_t *Y, int fracFP)
{
  int i = (blockIdx.x + blockIdx.y*gridDim.x) * blockDim.x + threadIdx.x;
  if(i < N) Y[i] = (int16_t) (X[i]*((int32_t)(1<<fracFP)));
}

void copy_float2fp_array_gpu(int N, float *X, int16_t* Y, int fracFP)
{
  copy_float2fp_array_kernel<<<cuda_gridsize(N), BLOCK>>>(N, X, Y, fracFP);
  check_error(cudaPeekAtLastError());
}

//im2col_gpu implementation
// src: https://github.com/BVLC/caffe/blob/master/src/caffe/util/im2col.cu
// You may also want to read: https://github.com/BVLC/caffe/blob/master/LICENSE

__global__ void im2col_fp_gpu_kernel(const int n, const int16_t* data_im,
        const int height, const int width, const int ksize,
        const int pad,
        const int stride,
        const int height_col, const int width_col,
        int16_t *data_col) {
    int index = blockIdx.x*blockDim.x+threadIdx.x;
    for(; index < n; index += blockDim.x*gridDim.x){
        int w_out = index % width_col;
        int h_index = index / width_col;
        int h_out = h_index % height_col;
        int channel_in = h_index / height_col;
        int channel_out = channel_in * ksize * ksize;
        int h_in = h_out * stride - pad;
        int w_in = w_out * stride - pad;
        int16_t* data_col_ptr = data_col;
        data_col_ptr += (channel_out * height_col + h_out) * width_col + w_out;
        const int16_t* data_im_ptr = data_im;
        data_im_ptr += (channel_in * height + h_in) * width + w_in;
        for (int i = 0; i < ksize; ++i) {
            for (int j = 0; j < ksize; ++j) {
                int h = h_in + i;
                int w = w_in + j;

                *data_col_ptr = (h >= 0 && w >= 0 && h < height && w < width) ?
                    data_im_ptr[i * width + j] : 0;

                //*data_col_ptr = data_im_ptr[ii * width + jj];

                data_col_ptr += height_col * width_col;
            }
        }
    }
}

void im2col_fp_gpu(int16_t *im,
         int channels, int height, int width,
         int ksize, int stride, int pad, int16_t *data_col){
    // We are going to launch channels * height_col * width_col kernels, each
    // kernel responsible for copying a single-channel grid.
    int height_col = (height + 2 * pad - ksize) / stride + 1;
    int width_col = (width + 2 * pad - ksize) / stride + 1;
    int num_kernels = channels * height_col * width_col;
    im2col_fp_gpu_kernel<<<(num_kernels+BLOCK-1)/BLOCK,
        BLOCK>>>(
                num_kernels, im, height, width, ksize, pad,
                stride, height_col,
                width_col, data_col);
}


__global__ void add_bias_fp_kernel(int16_t *output, int16_t *biases, int batch, int n, int size, int bias_shift)
{
    int index = (blockIdx.x + blockIdx.y*gridDim.x) * blockDim.x + threadIdx.x;
    if (index >= n*size*batch) return;
    int i = index % size;
    index /= size;
    int j = index % n;
    index /= n;
    int k = index;

    output[(k*n+j)*size + i] += (biases[j] >> bias_shift); //Q9.7 + Q9.7
}

void add_bias_fp_gpu(int16_t *output, int16_t *biases, int batch, int n, int size, int bias_shift)
{
    int num = n*size*batch;

    add_bias_fp_kernel<<<cuda_gridsize(num), BLOCK>>>(output, biases, batch, n, size, bias_shift);
    check_error(cudaPeekAtLastError());
}


 
__global__ void add_bias_kernel(int16_t *output, int16_t *biases, int batch, int n, int size)
{
    int index = (blockIdx.x + blockIdx.y*gridDim.x) * blockDim.x + threadIdx.x;
    if (index >= n*size*batch) return;
    int i = index % size;
    index /= size;
    int j = index % n;
    index /= n;
    int k = index;

    output[(k*n+j)*size + i] += biases[j];
}

void add_bias_gpu(int16_t *output, int16_t *biases, int batch, int n, int size)
{
    int num = n*size*batch;

    add_bias_kernel<<<cuda_gridsize(num), BLOCK>>>(output, biases, batch, n, size);
    check_error(cudaPeekAtLastError());
}

__device__ int16_t leaky_activate_kernel(int16_t x)
{
  //return (x>=0) ? x : (x >> 3); //Same as x*0.125
  return (x>=0) ? x : ((x >> 4) + (x >> 5) + (x >> 7)); //Same as x*0.1015625
}

__device__ int16_t logistic_activate_kernel(int16_t x)
{
  //linear sigmoid approximation
  // fp2375 = 2.375 in Q9.7, fp084375 = 0.84375 in Q9.7 ...
  int16_t fp2375 = 0x130, fp084375 = 0x6C, fp0625 = 0x50, fp05 = 0x40; //Q9.7
  int16_t fp5 = 0x280, fp1 = 0x080; //Q9.7
  int16_t val_in = x;

  if(val_in < 0) x = ~val_in +1; //emulates multiplying by -1
  
  if(x >= fp5) x = fp1; //saturated activation
  else if(x >= fp2375) x = fp084375 + (x >> 5); //0.84375 + x*2^(-5)
  else if(x >= fp1) x = fp0625 + (x >> 3); //0.625 + x*2^(-3)
  else x = fp05 + (x >> 2); //0.5 + x*2^(-2)

  return (val_in > 0)? (x) : (fp1 - x);
}

__global__ void activate_array_kernel(int16_t *x, int n, int a)
{
    int i = (blockIdx.x + blockIdx.y*gridDim.x) * blockDim.x + threadIdx.x;
    switch(a){
    case LEAKY:
      if(i < n) x[i] = leaky_activate_kernel(x[i]);
      break;
    case LOGISTIC:
      if(i < n) x[i] = logistic_activate_kernel(x[i]);
      break;
    default:
      break;
    }
}

void activate_array_gpu(int16_t *x, int n, int a) 
{
    activate_array_kernel<<<cuda_gridsize(n), BLOCK>>>(x, n, a);
    check_error(cudaPeekAtLastError());
}

__global__ void forward_maxpool_layer_kernel(int n, int in_h, int in_w, int in_c, int stride, int size, int pad, int16_t *input, int16_t *output, int *indexes)
{
    int h = (in_h + pad - size)/stride + 1;
    int w = (in_w + pad - size)/stride + 1;
    int c = in_c;

    int id = (blockIdx.x + blockIdx.y*gridDim.x) * blockDim.x + threadIdx.x;
    if(id >= n) return;

    int j = id % w;
    id /= w;
    int i = id % h;
    id /= h;
    int k = id % c;
    id /= c;
    int b = id;

    int w_offset = -pad/2;
    int h_offset = -pad/2;

    int out_index = j + w*(i + h*(k + c*b));
    int16_t max = MIN_VAL_INT16;
    int max_i = -1;
    int l, m;
    for(l = 0; l < size; ++l){
        for(m = 0; m < size; ++m){
            int cur_h = h_offset + i*stride + l;
            int cur_w = w_offset + j*stride + m;
            int index = cur_w + in_w*(cur_h + in_h*(k + b*in_c));
            int valid = (cur_h >= 0 && cur_h < in_h &&
                    cur_w >= 0 && cur_w < in_w);
            int16_t val = (valid != 0) ? input[index] : MIN_VAL_INT16;
            max_i = (val > max) ? index : max_i;
            max   = (val > max) ? val   : max;
        }
    }
    output[out_index] = max;
    //indexes[out_index] = max_i;
}

/*
used arguments: w, h, num_ker, downsample, net_output_gpu, layer_num
unused arguments: ignorePadding, new_output_pos

*/
void forward_maxpool_layer_gpu(int w, int h, int num_ker, int downsample, int ignorePadding, unsigned int new_output_pos, void** net_output_gpu, int layer_num)
{
  /*
    stride = downsample + 1
    size = 2
    padding = size - 1 = 2 - 1 = 1
  */
    int out_h = (h+1-2)/(downsample+1) + 1; //(h + padding - size)/stride + 1 
    int out_w = (w+1-2)/(downsample+1) + 1; //(w + padding - size)/stride + 1 
    int c = num_ker;

    size_t n = out_h*out_w*c; //output size

    //create output array
    net_output_gpu[layer_num] = (void*) cuda_make_fp_array(0, n);
    
    forward_maxpool_layer_kernel<<<cuda_gridsize(n), BLOCK>>>(n, h, w, num_ker, (downsample+1), 2, 1, (int16_t*) net_output_gpu[layer_num-1], (int16_t*) net_output_gpu[layer_num], 0);
    check_error(cudaPeekAtLastError());

}

__global__ void copy_fp_kernel(int N, int16_t *X, int OFFX, int INCX, int16_t *Y, int OFFY, int INCY)
{
  int i = (blockIdx.x + blockIdx.y*gridDim.x) * blockDim.x + threadIdx.x;
  if(i < N) Y[i*INCY + OFFY] = X[i*INCX + OFFX];
}

void copy_fp_gpu(int N, int16_t * X, int INCX,  int16_t * Y, int INCY)
{
  copy_fp_kernel<<<cuda_gridsize(N), BLOCK>>>(N, X, 0, INCX, Y, 0, INCY);
  check_error(cudaPeekAtLastError());

}



__global__ void upsample_kernel(size_t N, int16_t *x, int w, int h, int c, int batch, int stride, int forward, int scale, int16_t *out)
{
    size_t i = (blockIdx.x + blockIdx.y*gridDim.x) * blockDim.x + threadIdx.x;
    if(i >= N) return;
    int out_index = i;
    int out_w = i%(w*stride);
    i = i/(w*stride);
    int out_h = i%(h*stride);
    i = i/(h*stride);
    int out_c = i%c;
    i = i/c;
    int b = i%batch;

    int in_w = out_w / stride;
    int in_h = out_h / stride;
    int in_c = out_c;

    int in_index = b*w*h*c + in_c*w*h + in_h*w + in_w;


    if(forward) out[out_index] += (int16_t) scale * x[in_index];
}

void upsample_gpu(int16_t *in, int w, int h, int c, int batch, int stride, int forward, int scale, int16_t *out)
{
    size_t size = w*h*c*batch*stride*stride;
    upsample_kernel<<<cuda_gridsize(size), BLOCK>>>(size, in, w, h, c, batch, stride, forward, scale, out);
    check_error(cudaPeekAtLastError());
}


//Shared version
#define SHMEM_SIZE (16*16)

/*
  Matrix multiplication kernel that leverages shared memory to reduce global GPU memory accesses
  Each input value is only accessed once.
  Each thread computes 1 output value.
  The output computation is divided into submatrices of size 16x16
*/
__global__ void gemm_fp_kernel(int16_t *a, int16_t *b, int16_t *c, int m, int n, int k){
	// shared memory
	__shared__ int A[SHMEM_SIZE];
	__shared__ int B[SHMEM_SIZE];

	// get global thread id
	int row = blockIdx.x * blockDim.x + threadIdx.x;
	int col = blockIdx.y * blockDim.y + threadIdx.y;

	// Extract some builtin values to simplify code
	int tx = threadIdx.x;
	int ty = threadIdx.y;
	int dim = blockDim.x;

	//number of blocks along k dimension
	int num_k_blocks = (k + dim - 1)/dim;
	int idx_a, idx_b;
	
	// move the tile across the length of the grid
	int tmp = 0;
	// accumulate along k blocks
	for (int i = 0; i < num_k_blocks; i++){
	  // load values into shared memory
	  // Each thread in a block loads 1 value for each A and B
	  
	  // calculate matrix indexes
	  idx_a = (row*k) + (i*dim) + ty;
	  idx_b = (i*dim*n) + (tx*n) + col;
	  
	  // check input matrices boundaries
	  A[tx * dim + ty] = ( (row < m) && (i*dim+ ty < k)) ? (int) a[idx_a] : 0;
	  B[tx * dim + ty] = ( (i*dim+tx < k) && (col < n)) ? (int) b[idx_b] : 0;

	  __syncthreads();

	  //only perform muladd inside output boundaries
	  if(row < m && col < n){
	    //Accumulate the partial results
	    for (int j = 0; j < dim; j++){
	      tmp += A[tx*dim + j] * B[j*dim + ty]; //Q6.10 * Q9.7 = Q15.17
	    }
	  }
	  __syncthreads();
	}

	// write back result to main memory
	c[row*n + col] = (int16_t) (tmp >> 10); //Q15.17 -> Q9.7
}

// Naive Matrix Multiplication implementation for rectangular matrices
__global__ void matrixMulRect_kernel(int16_t *a, int16_t *b, int16_t *c, int M, int N, int K){
	// get thread global id
	int row  = blockIdx.x * blockDim.x + threadIdx.x;
	int col  = blockIdx.y * blockDim.y + threadIdx.y;

	// check range
	if(row < M && col < N){

		// Accumulate partial result
		int tmp = 0;

		for (int i = 0; i < K; i++){
		  tmp += ((int) a[row*K + i])* ((int) b[i*N + col]); // Q6.10*Q9.7 = Q15.17	
		}

		//Write back the result
		c[row*N + col] = (int16_t) (tmp>>10); //Q15.17 -> Q9.7
	}
}


void gemm_fp_gpu(int16_t *A_gpu, int16_t *B_gpu, int16_t *C_gpu, int M, int N, int K)
{
  //Create CTA and Grid dimentions
  int threads = 16; // 2D => 16 + 16 = 256
  int blocks_row = (M + threads - 1) / threads;
  int blocks_col = (N + threads - 1) / threads;

  // Setup our kernel launch parameters
  dim3 THREADS(threads, threads);
  dim3 BLOCKS(blocks_row, blocks_col);

  // naive matrix multiplication kernel - only accesses global memory
  matrixMulRect_kernel<<<BLOCKS, THREADS>>>(A_gpu, B_gpu, C_gpu, M, N, K);

  // matrix multiplication kernel that uses shared memory
  // gemm_fp_kernel<<<BLOCKS, THREADS>>>(A_gpu, B_gpu, C_gpu, M, N, K);
  check_error(cudaPeekAtLastError());
}



#else //Float
//im2col_gpu implementation
// src: https://github.com/BVLC/caffe/blob/master/src/caffe/util/im2col.cu
// You may also want to read: https://github.com/BVLC/caffe/blob/master/LICENSE

__global__ void im2col_gpu_kernel(const int n, const float* data_im,
        const int height, const int width, const int ksize,
        const int pad,
        const int stride,
        const int height_col, const int width_col,
        float *data_col) {
    int index = blockIdx.x*blockDim.x+threadIdx.x;
    for(; index < n; index += blockDim.x*gridDim.x){
        int w_out = index % width_col;
        int h_index = index / width_col;
        int h_out = h_index % height_col;
        int channel_in = h_index / height_col;
        int channel_out = channel_in * ksize * ksize;
        int h_in = h_out * stride - pad;
        int w_in = w_out * stride - pad;
        float* data_col_ptr = data_col;
        data_col_ptr += (channel_out * height_col + h_out) * width_col + w_out;
        const float* data_im_ptr = data_im;
        data_im_ptr += (channel_in * height + h_in) * width + w_in;
        for (int i = 0; i < ksize; ++i) {
            for (int j = 0; j < ksize; ++j) {
                int h = h_in + i;
                int w = w_in + j;

                *data_col_ptr = (h >= 0 && w >= 0 && h < height && w < width) ?
                    data_im_ptr[i * width + j] : 0;

                //*data_col_ptr = data_im_ptr[ii * width + jj];

                data_col_ptr += height_col * width_col;
            }
        }
    }
}

void im2col_gpu(float *im,
         int channels, int height, int width,
         int ksize, int stride, int pad, float *data_col){
    // We are going to launch channels * height_col * width_col kernels, each
    // kernel responsible for copying a single-channel grid.
    int height_col = (height + 2 * pad - ksize) / stride + 1;
    int width_col = (width + 2 * pad - ksize) / stride + 1;
    int num_kernels = channels * height_col * width_col;
    im2col_gpu_kernel<<<(num_kernels+BLOCK-1)/BLOCK,
        BLOCK>>>(
                num_kernels, im, height, width, ksize, pad,
                stride, height_col,
                width_col, data_col);
}

__global__ void add_bias_kernel(float *output, float *biases, int batch, int n, int size)
{
    int index = (blockIdx.x + blockIdx.y*gridDim.x) * blockDim.x + threadIdx.x;
    if (index >= n*size*batch) return;
    int i = index % size;
    index /= size;
    int j = index % n;
    index /= n;
    int k = index;

    output[(k*n+j)*size + i] += biases[j];
}

void add_bias_gpu(float *output, float *biases, int batch, int n, int size)
{
    int num = n*size*batch;

    add_bias_kernel<<<cuda_gridsize(num), BLOCK>>>(output, biases, batch, n, size);
    check_error(cudaPeekAtLastError());
}

//__device__ float leaky_activate_kernel(float x){return (x>0) ? x : .125f*x;}
__device__ float leaky_activate_kernel(float x){return (x>0) ? x : .1015625f*x;}
__device__ float logistic_activate_kernel(float x){return 1.f/(1.f + expf(-x));}

__global__ void activate_array_kernel(float *x, int n, int a)
{
    int i = (blockIdx.x + blockIdx.y*gridDim.x) * blockDim.x + threadIdx.x;
    //tiny only used leaky activation, when batch_norm is done
    switch(a){
    case LEAKY:
      if(i < n) x[i] = leaky_activate_kernel(x[i]);
      break;
    case LOGISTIC:
      if(i < n) x[i] = logistic_activate_kernel(x[i]);
      break;
    default:
      break;
    }
}

void activate_array_gpu(float *x, int n, int a) 
{
    activate_array_kernel<<<cuda_gridsize(n), BLOCK>>>(x, n, a);
    check_error(cudaPeekAtLastError());
}

__global__ void forward_maxpool_layer_kernel(int n, int in_h, int in_w, int in_c, int stride, int size, int pad, float *input, float *output, int *indexes)
{
    int h = (in_h + pad - size)/stride + 1;
    int w = (in_w + pad - size)/stride + 1;
    int c = in_c;

    int id = (blockIdx.x + blockIdx.y*gridDim.x) * blockDim.x + threadIdx.x;
    if(id >= n) return;

    int j = id % w;
    id /= w;
    int i = id % h;
    id /= h;
    int k = id % c;
    id /= c;
    int b = id;

    int w_offset = -pad/2;
    int h_offset = -pad/2;

    int out_index = j + w*(i + h*(k + c*b));
    float max = -INFINITY;
    int max_i = -1;
    int l, m;
    for(l = 0; l < size; ++l){
        for(m = 0; m < size; ++m){
            int cur_h = h_offset + i*stride + l;
            int cur_w = w_offset + j*stride + m;
            int index = cur_w + in_w*(cur_h + in_h*(k + b*in_c));
            int valid = (cur_h >= 0 && cur_h < in_h &&
                    cur_w >= 0 && cur_w < in_w);
            float val = (valid != 0) ? input[index] : -INFINITY;
            max_i = (val > max) ? index : max_i;
            max   = (val > max) ? val   : max;
        }
    }
    output[out_index] = max;
    //indexes[out_index] = max_i;
}


/*
used arguments: w, h, num_ker, downsample, net_output_gpu, layer_num
unused arguments: ignorePadding, new_output_pos

*/
void forward_maxpool_layer_gpu(int w, int h, int num_ker, int downsample, int ignorePadding, unsigned int new_output_pos, void** net_output_gpu, int layer_num)
{
  /*
    stride = downsample + 1
    size = 2
    padding = size - 1 = 2 - 1 = 1
  */
    int out_h = (h+1-2)/(downsample+1) + 1; //(h + padding - size)/stride + 1 
    int out_w = (w+1-2)/(downsample+1) + 1; //(w + padding - size)/stride + 1 
    int c = num_ker;

    size_t n = out_h*out_w*c; //output size

    //create output array
    net_output_gpu[layer_num] = (void*) cuda_make_array(0, n);
    
    //    forward_maxpool_layer_kernel<<<cuda_gridsize(n), BLOCK>>>(n, layer.h, layer.w, layer.c, layer.stride, layer.size, layer.pad, net.input_gpu, layer.output_gpu, layer.indexes_gpu);
    // indexes is not used for anything?
    forward_maxpool_layer_kernel<<<cuda_gridsize(n), BLOCK>>>(n, h, w, num_ker, (downsample+1), 2, 1, (float*) net_output_gpu[layer_num-1], (float*) net_output_gpu[layer_num], 0);
    check_error(cudaPeekAtLastError());

}

__global__ void upsample_kernel(size_t N, float *x, int w, int h, int c, int batch, int stride, int forward, float scale, float *out)
{
    size_t i = (blockIdx.x + blockIdx.y*gridDim.x) * blockDim.x + threadIdx.x;
    if(i >= N) return;
    int out_index = i;
    int out_w = i%(w*stride);
    i = i/(w*stride);
    int out_h = i%(h*stride);
    i = i/(h*stride);
    int out_c = i%c;
    i = i/c;
    int b = i%batch;

    int in_w = out_w / stride;
    int in_h = out_h / stride;
    int in_c = out_c;

    int in_index = b*w*h*c + in_c*w*h + in_h*w + in_w;


    if(forward) out[out_index] += scale * x[in_index];
    else atomicAdd(x+in_index, scale * out[out_index]);
}

void upsample_gpu(float *in, int w, int h, int c, int batch, int stride, int forward, float scale, float *out)
{
    size_t size = w*h*c*batch*stride*stride;
    upsample_kernel<<<cuda_gridsize(size), BLOCK>>>(size, in, w, h, c, batch, stride, forward, scale, out);
    check_error(cudaPeekAtLastError());
}


#endif //ifdef FIXED

__global__ void copy_kernel(int N,  float *X, int OFFX, int INCX, float *Y, int OFFY, int INCY)
{
    int i = (blockIdx.x + blockIdx.y*gridDim.x) * blockDim.x + threadIdx.x;
    if(i < N) Y[i*INCY + OFFY] = X[i*INCX + OFFX];
}


void copy_gpu(int N, float * X, int INCX, float * Y, int INCY)
{
    copy_kernel<<<cuda_gridsize(N), BLOCK>>>(N, X, 0, INCX, Y, 0, INCY);
    check_error(cudaPeekAtLastError());
}

#endif //ifdef GPU
