/*    Sample program to test custom kernel for
      Matrix multiplication algorithm
      adapted from the youtube channel CoffeeBeforeArch

      compile with:
         > make cuda
*/


#include <cstdlib>
#include <cassert>
#include <iostream>
#include <time.h>
#include <cuda_profiler_api.h>
#include <cuda.h>
#include <cuda_runtime.h>

#include <cublas_v2.h>


using namespace std;

#define BLOCK (512)

void error(const char *s)
{
  cout << s << endl;
  assert(0);
  exit(-1);
}

dim3 cuda_gridsize(size_t n){
    size_t k = (n-1) / BLOCK + 1;
    size_t x = k;
    size_t y = 1;
    if(x > 65535){
        x = ceil(sqrt(k));
        y = (n-1)/(x*BLOCK) + 1;
    }
    dim3 d(x, y, 1);
    //printf("%ld %ld %ld %ld\n", n, x, y, x*y*BLOCK);
    return d;
}


void check_error(cudaError_t status)
{
    //cudaDeviceSynchronize();
    cudaError_t status2 = cudaGetLastError();
    if (status != cudaSuccess)
    {   
        const char *s = cudaGetErrorString(status);
        cout << "CUDA Error:" << s << endl;
        assert(0);
    } 
    if (status2 != cudaSuccess)
    {   
        const char *s = cudaGetErrorString(status);
        cout << "CUDA Error Prev: " <<s << endl;
        assert(0);
    } 
}

__global__ void fill_kernel(int N, int ALPHA, float *X, int INCX)
{
    int i = (blockIdx.x + blockIdx.y*gridDim.x) * blockDim.x + threadIdx.x;
    if(i < N) X[i*INCX] = ALPHA;
}

void fill_gpu(int N, int ALPHA, float * X, int INCX)
{
    fill_kernel<<<cuda_gridsize(N), BLOCK>>>(N, ALPHA, X, INCX);
    check_error(cudaPeekAtLastError());
}


float *cuda_make_array(float *x, size_t n)
{
  float *x_gpu;
  size_t size = sizeof(float)*n;
  cudaError_t status = cudaMalloc((void**)&x_gpu, size);
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

void cuda_free(void *x_gpu)
{
    cudaError_t status = cudaFree(x_gpu);
    check_error(status);
}

void cuda_pull_array(float *x_gpu, float* x, size_t n)
{
  size_t size = sizeof(float)*n;
  cudaError_t status = cudaMemcpy(x, x_gpu, size, cudaMemcpyDeviceToHost);
  check_error(status);
}


void cuda_pull_fp_array(int *x_gpu, int* x, size_t n){
  size_t size = sizeof(int)*n;
  cudaError_t status = cudaMemcpy(x, x_gpu, size, cudaMemcpyDeviceToHost);
  check_error(status);
}


__global__ void copy_fp2float_array_kernel(int N, int *X, float *Y, int fracFP)
{
  int i = (blockIdx.x + blockIdx.y*gridDim.x) * blockDim.x + threadIdx.x;
  if(i < N) Y[i] = (float) ((X[i]*1.0)/((int)(1<<fracFP)));
}

void copy_fp2float_array_gpu(int N, int *X, float* Y, int fracFP)
{
  copy_fp2float_array_kernel<<<cuda_gridsize(N), BLOCK>>>(N, X, Y, fracFP);
  check_error(cudaPeekAtLastError());
}


__global__ void copy_float2fp_array_kernel(int N, float *X, int *Y, int fracFP)
{
  int i = (blockIdx.x + blockIdx.y*gridDim.x) * blockDim.x + threadIdx.x;
  if(i < N) Y[i] = (int) (X[i]*((int)(1<<fracFP)));
}

void copy_float2fp_array_gpu(int N, float *X, int* Y, int fracFP)
{
  copy_float2fp_array_kernel<<<cuda_gridsize(N), BLOCK>>>(N, X, Y, fracFP);
  check_error(cudaPeekAtLastError());
}


//////////////////////////////////

// Naive Matrix Multiplication implementation for square matrices
__global__ void matrixMul(float *a, float *b, float *c, int N){
	// get thread global id
	int row  = blockIdx.x * blockDim.x + threadIdx.x;
	int col  = blockIdx.y * blockDim.y + threadIdx.y;

	// check range
	if(row < N && col < N){

		// Accumulate partial result
		float tmp = 0;

		for (int i = 0; i < N; i++){
			tmp += a[row*N + i]*b[i*N + col];	
		}

		//Write back the result
		c[row*N + col] = tmp;
	}
}


// Naive Matrix Multiplication implementation for rectangular matrices
__global__ void matrixMulRect(float *a, float *b, float *c, int M, int N, int K){
	// get thread global id
	int row  = blockIdx.x * blockDim.x + threadIdx.x;
	int col  = blockIdx.y * blockDim.y + threadIdx.y;

	// check range
	if(row < M && col < N){

		// Accumulate partial result
		float tmp = 0;

		for (int i = 0; i < K; i++){
			tmp += a[row*K + i]*b[i*N + col];	
		}

		//Write back the result
		c[row*N + col] = tmp;
	}
}

// Naive Matrix Multiplication implementation for rectangular matrices
__global__ void matrixMulRect_fp(int16_t *a, int16_t *b, int16_t *c, int M, int N, int K){
	// get thread global id
	int row  = blockIdx.x * blockDim.x + threadIdx.x;
	int col  = blockIdx.y * blockDim.y + threadIdx.y;

	// check range
	if(row < M && col < N){

		// Accumulate partial result
		int tmp = 0;

		for (int i = 0; i < K; i++){
		  tmp += ((int) a[row*K + i])*((int)b[i*N + col]);	
		}

		//Write back the result
		c[row*N + col] = (int16_t) (tmp >> 10);
	}
}


////////////////////////////////

//Shared version
// statically define shared memory
#define SHMEM_SIZE (16*16)

__global__ void matrixMulShared(float *a, float *b, float *c, int m, int n, int k){
	// shared memory
	__shared__ float A[SHMEM_SIZE];
	__shared__ float B[SHMEM_SIZE];

	// get global thread id
	int row = blockIdx.x * blockDim.x + threadIdx.x;
	int col = blockIdx.y * blockDim.y + threadIdx.y;

	// Extract some builtin values to simplify code
	int tx = threadIdx.x;
	int ty = threadIdx.y;
	int dim = blockDim.x;

	//number of blocks along k dimention
	int num_k_blocks = (k + dim - 1)/dim;
	int idx_a, idx_b;
	
	// move the tile across the length of the grid
	float tmp = 0;
	// accumulate along k blocks
	for (int i = 0; i < num_k_blocks; i++){
	  // load values into shared memory
	  // Each thread in a block loads 1 value for each A and B
	  
	  // calculate matrix indexes
	  idx_a = (row*k) + (i*dim) + ty;
	  idx_b = (i*dim*n) + (tx*n) + col;

	  // check input matrices boundaries
	  A[tx * dim + ty] = (idx_a < m*k) ? a[idx_a] : 0;
	  B[tx * dim + ty] = (idx_b < k*n) ? b[idx_b] : 0;

	  __syncthreads();

	  //only perform muladd inside output boundaries
	  if(row < m && col < n){
	    //Accumulate the partial results
	    for (int j = 0; j < dim; j++){
	      tmp += A[tx*dim + j] * B[j*dim + ty];
	    }
	  }
	  __syncthreads();
	}

	// write back result to main memory
	c[row*n + col] = tmp;
}


//Shared version
// statically define shared memory
#define SHMEM_SIZE (16*16)

__global__ void matrixMul_fp_Shared(int *a, int *b, int *c, int m, int n, int k){
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

	//number of blocks along k dimention
	int num_k_blocks = (k + dim - 1)/dim;
	int idx_a, idx_b;

	int tmp = 0;
	// accumulate along k blocks
	for (int i = 0; i < num_k_blocks; i++){
	  // load values into shared memory
	  // Each thread in a block loads 1 value for each A and B
	  
	  // calculate matrix indexes
	  idx_a = (row*k) + (i*dim) + ty;
	  idx_b = (i*dim*n) + (tx*n) + col;

	  // check index boundaries
	  A[tx * dim + ty] = (idx_a < m*k) ? a[idx_a] : 0;
	  B[tx * dim + ty] = (idx_b < k*n) ? b[idx_b] : 0;

	  __syncthreads();

	  //only perform muladd inside output boundaries
	  if(row < m && col < n){
	    //Accumulate the partial results
	    for (int j = 0; j < dim; j++){
	      tmp += A[tx*dim + j] * B[j*dim + ty];
	    }
	  }
	  __syncthreads();
	}

	// write back result to main memory
	c[row*n + col] = tmp;
}



///////////////////////////


///////////////////////////
// im2col_gpu

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



__global__ void im2col_fp_gpu_kernel(const int n, const int* data_im,
        const int height, const int width, const int ksize,
        const int pad,
        const int stride,
        const int height_col, const int width_col,
        int *data_col) {
    int index = blockIdx.x*blockDim.x+threadIdx.x;
    for(; index < n; index += blockDim.x*gridDim.x){
        int w_out = index % width_col;
        int h_index = index / width_col;
        int h_out = h_index % height_col;
        int channel_in = h_index / height_col;
        int channel_out = channel_in * ksize * ksize;
        int h_in = h_out * stride - pad;
        int w_in = w_out * stride - pad;
        int* data_col_ptr = data_col;
        data_col_ptr += (channel_out * height_col + h_out) * width_col + w_out;
        const int* data_im_ptr = data_im;
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

void im2col_fp_gpu(int *im,
         int channels, int height, int width,
         int ksize, int stride, int pad, int *data_col){
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



///////////////////////////

///////////////////////////
int cuda_get_device()
{
    int n = 0;
    cudaError_t status = cudaGetDevice(&n);
    check_error(status);
    return n;
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


// gemm_gpu()
void gemm_gpu(int TA, int TB, int M, int N, int K, float ALPHA, 
        float *A_gpu, int lda, 
        float *B_gpu, int ldb,
        float BETA,
        float *C_gpu, int ldc)
{
    cublasHandle_t handle = blas_handle();
    cublasStatus_t status = cublasSgemm(handle, (TB ? CUBLAS_OP_T : CUBLAS_OP_N), 
            (TA ? CUBLAS_OP_T : CUBLAS_OP_N), N, M, K, &ALPHA, B_gpu, ldb, A_gpu, lda, &BETA, C_gpu, ldc);
    //check_error(status);
}



///////////////////////////




//Create CPU side matrix - rows x cols
float* create_matrix(int rows, int cols){
  //create matrix on CPU side
  float* m = (float*) malloc(rows*cols*sizeof(float));
  if(m == 0){
    cout << "Failed to allocate memory" << endl;
    exit(1);
  }
  
  return m;
}


//Create CPU side matrix - rows x cols
int* create_fp_matrix(int rows, int cols){
  //create matrix on CPU side
  int* m = (int*) malloc(rows*cols*sizeof(int));
  if(m == 0){
    cout << "Failed to allocate memory" << endl;
    exit(1);
  }
  
  return m;
}


float* create_3D_array(int w, int h, int c){

  float * array = (float*) malloc(sizeof(float)*w*h*c);
  if(array == 0){
    cout << "Failed to allocate 3D array" << endl;
    exit(1);
  }
  
  return array;
}

int* create_3D_fp_array(int w, int h, int c){

  int * array = (int*) malloc(sizeof(int)*w*h*c);
  if(array == 0){
    cout << "Failed to allocate 3D array" << endl;
    exit(1);
  }
  
  return array;
}


// Initialize matrices - 0 to 100
void init_matrix(int* matrix, int rows, int cols ){

  for (int i = 0; i < rows*cols; i++){
    matrix[i] = i+1; //rand() % 100;
  }

}

void print_matrix(float* matrix, int rows, int cols){
  int i, j, idx=0;
  for(i=0;i<rows;i++){
    cout << "\t";
    for(j=0;j<cols;j++){
      cout << matrix[idx] << " \t";
      idx++;
    }
    cout << endl;
  }

}

void print_fp_matrix(int* matrix, int rows, int cols){
  int i, j, idx=0;
  for(i=0;i<rows;i++){
    cout << "\t";
    for(j=0;j<cols;j++){
      cout << matrix[idx] << " \t";
      idx++;
    }
    cout << endl;
  }

}

// Print 100 evenly spaced positions of the matrix
void print_sample_matrix(float* matrix, int rows, int cols){
  int idx, n_prints=100;
  int step = (rows*cols)/n_prints;

  /*check for smaller matrices < 100 positions*/
  if(rows*cols < n_prints){
    step = 1;
  }

  cout << "\nSampling matrix[" << rows << "][" << cols << "]:" << endl;
  for(idx=0;idx<rows*cols;idx+=step){
    cout << idx << ": " << matrix[idx] << endl;
  }
}

void print_sample_side_by_side(float *a, float* b, int rows, int cols){
  int idx, n_prints=100;
  int step = (rows*cols)/n_prints;

  /*check for smaller matrices < 100 positions*/
  if(rows*cols < n_prints){
    step = 1;
  }

  cout << "\nSampling matrices[" << rows << "][" << cols << "]:" << endl;
  for(idx=0;idx<rows*cols;idx+=step){
    cout << idx << ": " << a[idx] << "\t\t"<< b[idx] << "\tdiff:" << a[idx]-b[idx] << endl;
  }
}


// Initialize 3D array
void init_3D_array(float *im, int w, int h, int c){

  int i, j, k, idx=0;
  for(i=0;i<c;i++){ // channels
    for(j=0;j<h;j++){ //rows
      for(k=0;k<w;k++){ //cols
	im[idx] = rand()%100; //idx+1;
	idx++;
      }
    }
  }
}

// Initialize 3D array
void init_3D_fp_array(int *im, int w, int h, int c){

  int i, j, k, idx=0;
  for(i=0;i<c;i++){ // channels
    for(j=0;j<h;j++){ //rows
      for(k=0;k<w;k++){ //cols
	im[idx] = rand()%100; //idx+1;
	idx++;
      }
    }
  }
}


// Identity kernel
void init_3D_kernel(float *kernel, int w, int h, int c, int num_ker){

  int i, j, k, l, idx=0;
  // int factor = 0;
  // int mid = w/2;
  for(l=0;l<num_ker;l++){ //3D kernel
    // factor = l+1;
    for(i=0;i<c;i++){ // channels
      for(j=0;j<h;j++){ //rows
	for(k=0;k<w;k++){ //cols
	  kernel[idx] = rand()%100;//factor*(j==mid && k==mid);
	  idx++;
	}
      }
    }
  }
}

// Identity kernel
void init_3D_fp_kernel(int *kernel, int w, int h, int c, int num_ker){

  int i, j, k, l, idx=0;
  // int factor = 0;
  // int mid = w/2;
  for(l=0;l<num_ker;l++){ //3D kernel
    // factor = l+1;
    for(i=0;i<c;i++){ // channels
      for(j=0;j<h;j++){ //rows
	for(k=0;k<w;k++){ //cols
	  kernel[idx] = rand()%100;//factor*(j==mid && k==mid);
	  idx++;
	}
      }
    }
  }
}


// Print 3D array
void print_3D_array(float * a, int w, int h, int c){
  
  int i, j, k, idx=0;
  for(i=0;i<c;i++){ // channels
    cout << "c = " << i << endl;
    for(j=0;j<h;j++){ //rows
      cout << "\t";
      for(k=0;k<w;k++){ //cols
	cout << a[idx] << " \t";
	idx++;
      }
      cout << endl;
    }
    cout << "\n" << endl;
  }  
}

// Print 3D array
void print_3D_fp_array(int * a, int w, int h, int c){
  
  int i, j, k, idx=0;
  for(i=0;i<c;i++){ // channels
    cout << "c = " << i << endl;
    for(j=0;j<h;j++){ //rows
      cout << "\t";
      for(k=0;k<w;k++){ //cols
	cout << a[idx] << " \t";
	idx++;
      }
      cout << endl;
    }
    cout << "\n" << endl;
  }  
}


// Verify result of CPU
void verify_result(int *a, int *b, int *c, int c_rows, int c_cols, int a_cols){
  int tmp;
  for (int i = 0; i < c_rows; i++){ // row
    for (int j = 0; j < c_cols; j++){ //col
      // calculate 1 output
      tmp = 0;
      for (int k = 0; k < a_cols; k++){
	tmp += a[i*a_cols + k] * b[k*c_cols + j];
      }

      // Check each result
      assert(tmp == c[i*c_cols + j]);

      // cout << "C[" << i <<"][" << j <<"]:" << tmp << endl; 
      // cout << "C_gpu[" << i <<"][" << j <<"]:" << c[i*c_cols + j] << endl;
    }
  }

}

int main(){
  // input image 4x4
  int w=416, h=416, c=3;
  int ksize=3, num_ker=16;

  
  // convolution input (3D: C x H x W)

  float *im = create_3D_array(w, h, c);

  init_3D_array(im, w, h, c);

  // cout << "Image in:" << endl;
  // print_3D_array(im, w, h, c);

  // im2col unrolled image: (Ksize x Ksize x C) x (H x W)
  float *out = create_matrix(ksize*ksize*c, w*h);


  // weights (Num_ker) x (Ksize x Ksize x C)
  float *weights = create_matrix(num_ker, ksize*ksize*c);
  init_3D_kernel(weights, ksize, ksize, c, num_ker);

  // cout << "Weights:" << endl;

  // print_3D_array(weights, ksize, ksize, c);
  // print_matrix(weights, num_ker, ksize*ksize*c);

  // gemm output (Num_ker x H x W)
  float *gemm_out = create_3D_array(w, h, num_ker);

  //GPU side
  float *im_gpu, *out_gpu;
  im_gpu = cuda_make_array(im, w*h*c);
  out_gpu = cuda_make_array(0, w*h*ksize*ksize*c);
  float* weights_gpu = cuda_make_array(weights, ksize*ksize*c*num_ker);
  float* gemm_out_gpu = cuda_make_array(0, w*h*num_ker);

  im2col_gpu(im_gpu, c, h, w, ksize, 1, 1, out_gpu);

  cuda_pull_array(out_gpu, out, w*h*ksize*ksize*c);

  // cout << "im2col output" << endl;

  // print_sample_matrix(out, ksize*ksize*c, w*h);

  // print_matrix(out, ksize*ksize*c, w*h);

  //gemm
  /* gemm_gpu(0,0,m,n,k,1,a,k,b,n,0,out,n); */
  gemm_gpu(0,0,num_ker,w*h,ksize*ksize*c, 1, weights_gpu, ksize*ksize*c, out_gpu, w*h, 0, gemm_out_gpu, w*h );
  
  cuda_pull_array(gemm_out_gpu, gemm_out, w*h*num_ker);


  // cout << "gemm output:" << endl;
  // print_3D_array(gemm_out, w, h, num_ker);
  // print_sample_matrix(gemm_out, num_ker, w*h);

  // custom kernel
  float * mm_out = create_matrix(num_ker, w*h);
  float * mm_out_gpu = cuda_make_array(0,num_ker*w*h);

  // M, N, K variables
  int M = num_ker;
  int N = w*h;
  int K = ksize*ksize*c;

  //Create CTA and Grid dimentions
  int threads = 16; // 2D => 16 + 16 = 256
  int blocks_row = (M + threads - 1) / threads;
  int blocks_col = (N + threads - 1) / threads;

  // Setup our kernel launch parameters
  dim3 THREADS(threads, threads);
  dim3 BLOCKS(blocks_row, blocks_col);

  clock_t start, end;
  double kernel_time = 0;


  //start time count
  start = clock();

  // Launch kernel
  // Naive implementation
  // matrixMulRect<<<BLOCKS, THREADS, 0, 0>>>(weights_gpu, out_gpu, mm_out_gpu, M, N, K);
  // cudaDeviceSynchronize();

  // Launch kernel
  // Shared memory kernel implementation
  matrixMulShared<<<BLOCKS, THREADS, 0, 0>>>(weights_gpu, out_gpu, mm_out_gpu, M, N, K);
  cudaDeviceSynchronize();


  //end time count
  end = clock();
  kernel_time = ((double) (end-start)) / CLOCKS_PER_SEC;
  cout.precision(2);
  cout << "KERNEL TIME: " << fixed << kernel_time << "seconds" << endl;

  //copy result to cpu
  cuda_pull_array(mm_out_gpu, mm_out, M*N);

  // cout << "Custom kernel output:" << endl;
  // print_3D_array(mm_out, w, h, num_ker);
  // print_sample_matrix(mm_out, M, N);

  cout << "cublasGemm \t\tMatrixMulRect" << endl;
  print_sample_side_by_side(gemm_out, mm_out, num_ker, w*h);

  //free memory
  free(im); free(weights); free(out); free(gemm_out); free(mm_out);
  cuda_free(im_gpu); cuda_free(weights_gpu); cuda_free(out_gpu); cuda_free(gemm_out_gpu); cuda_free(mm_out_gpu);

  // // Verify the result in CPU
  // verify_result(a, b, c, M, N, K);

  cout << "PROGRAM COMPLETELY SUCCESSFULLY!" << endl;

  return 0;
}
