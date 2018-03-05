// Created for Low Level Parallel Programming 2017
//
// Implements the heatmap functionality. 
//
//#include "heatmap.h"
#include "ped_model.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <stdlib.h>
#include <math.h>


// Sets up the heatmap
void Ped::Model::setupHeatmapSeq()
{
	int *hm = (int*)calloc(SIZE*SIZE, sizeof(int));
	int *shm = (int*)malloc(SCALED_SIZE*SCALED_SIZE*sizeof(int));
	int *bhm = (int*)malloc(SCALED_SIZE*SCALED_SIZE*sizeof(int));

	heatmap = (int**)malloc(SIZE*sizeof(int*));

	scaled_heatmap = (int**)malloc(SCALED_SIZE*sizeof(int*));
	blurred_heatmap = (int**)malloc(SCALED_SIZE*sizeof(int*));

	for (int i = 0; i < SIZE; i++)
	{
		heatmap[i] = hm + SIZE*i;
	}
	for (int i = 0; i < SCALED_SIZE; i++)
	{
		scaled_heatmap[i] = shm + SCALED_SIZE*i;
		blurred_heatmap[i] = bhm + SCALED_SIZE*i;
	}

	//Copy SCALED_SIZE to device
	int temp_SCALED_SIZE = SCALED_SIZE;
	cudaMalloc((void**)&d_SCALED_SIZE, sizeof(int));
	cudaMemcpy(d_SCALED_SIZE, &temp_SCALED_SIZE, sizeof(int), cudaMemcpyHostToDevice);

	//Allocate and init heatmap for device
	cudaMalloc((void**)&d_heatmap, SIZE*SIZE*sizeof(int));
	enum cudaError memcpy_status = cudaMemcpy(d_heatmap, heatmap[0], SIZE*SIZE*sizeof(int), cudaMemcpyHostToDevice);

	//Allocate and init scaled heatmap for device
	cudaMalloc((void**)&d_scaled_heatmap, SCALED_SIZE*SCALED_SIZE*sizeof(int));
	cudaMemset(d_scaled_heatmap, 0, SCALED_SIZE*SCALED_SIZE*sizeof(int));

	//Allocate and init blurred heatmap for device
	cudaMalloc((void**)&d_blurred_heatmap, SCALED_SIZE*SCALED_SIZE*sizeof(int));
	cudaMemset(d_blurred_heatmap, 0, SCALED_SIZE*SCALED_SIZE*sizeof(int));

	//Blur filter definition and copy to device
	const int w[5][5] = {
		{ 1, 4, 7, 4, 1 },
		{ 4, 16, 26, 16, 4 },
		{ 7, 26, 41, 26, 7 },
		{ 4, 16, 26, 16, 4 },
		{ 1, 4, 7, 4, 1 }
	};
	cudaMalloc((void**)&d_blur_filter, 5*5*sizeof(int));
	cudaMemcpy(d_blur_filter, w, 5*5*sizeof(int), cudaMemcpyHostToDevice);

	//Get and copy number of agents to device
	no_agents = agentCollection->size();
	cudaMalloc(&d_agents, sizeof(int));
	cudaMemcpyAsync(d_agents, &no_agents, sizeof(int), cudaMemcpyHostToDevice);

	//Allocate the desired vectors for cuda
	cudaMalloc(&d_desiredXs, no_agents*sizeof(float));
	cudaMalloc(&d_desiredYs, no_agents*sizeof(float));

	//Create the streams
	//cudaStream_t* stream1 = (cudaStream_t*)s1;
	//cudaStreamCreate((cudaStream_t*)s1);
	//cudaStream_t* stream2 = (cudaStream_t*)s2;
	//cudaStreamCreate(stream2);
	//cudaStream_t* stream3 = (cudaStream_t*)s3;
	//cudaStreamCreate(stream3);
}

void Ped::Model::cleanupCuda(){
	cudaFree(d_SCALED_SIZE);
	cudaFree(d_heatmap);
	cudaFree(d_scaled_heatmap);
	cudaFree(d_blurred_heatmap);
	cudaFree(d_blur_filter);
	cudaFree(d_agents);
	cudaFree(d_desiredXs);
	cudaFree(d_desiredYs);
//	cudaStreamDestroy();

}

/*
* Fades the blocks before each tick. 
*/
__global__
void fade(int* d_heatmap){
	long tid = threadIdx.x + blockIdx.x * blockDim.x;
	d_heatmap[tid] = d_heatmap[tid] * 0.80;
}

/*
* Increase the red color for the block for each agent that wants to access it.
*/
__global__
void locationContention(int* d_heatmap, float* d_desiredXs, float* d_desiredYs, int* d_agents){
	long tid = threadIdx.x + blockIdx.x * blockDim.x;
	if (tid < *d_agents){
		int desX = d_desiredXs[tid];
		int desY = d_desiredYs[tid];
		int inc = 40;
		atomicAdd(&d_heatmap[desY*SIZE + desX], inc);
	}
}

/*
* Make sure the color code isn't over 255(0xFF).
*/
__global__
void ceiling(int* d_heatmap){
	long tid = threadIdx.x + blockIdx.x * blockDim.x;
	int ceiling = 255;
	if (d_heatmap[tid] > ceiling)
		d_heatmap[tid] = ceiling;
}

__global__
void scale(int* d_scaled_heatmap, int* d_SCALED_SIZE, int* d_heatmap){
	long tid = threadIdx.x + blockIdx.x * blockDim.x;
	int s_row = tid / (*d_SCALED_SIZE); 
	int s_col = tid; //Column is either thread id if row == 0. Otherwise thread id - row * rowsize
	if (s_row)
		s_col = tid - s_row * (*d_SCALED_SIZE);

	//Scale down row and column to heatmap index
	int row = s_row / CELLSIZE;
	int col = s_col / CELLSIZE;
	int index = col + row * SIZE;

	d_scaled_heatmap[tid] = d_heatmap[index];
}



__global__
void gauss(int* d_scaled_heatmap, int* d_blurred_heatmap, int* d_SCALED_SIZE, int* d_blur_filter){
	long tid = ((blockIdx.y * blockDim.y + threadIdx.y) * (gridDim.x * blockDim.x)) + (blockIdx.x * blockDim.x) + threadIdx.x;
	int global_row_length = *d_SCALED_SIZE;
	int heatmap_row = tid / global_row_length;
	int heatmap_col = tid;
	if (heatmap_row)
		heatmap_col = tid - heatmap_row * global_row_length;

	__shared__ int s[BLOCKSIZE];
	int shared_index = threadIdx.x + threadIdx.y * blockDim.x;
	s[shared_index] = d_scaled_heatmap[tid];
	__syncthreads();

	//printf("Idx: %d, bIdx: %d, bIdy: %d, tid: %d\n", threadIdx.x, blockIdx.x, blockIdx.y, tid);

	if (heatmap_row < 2 || heatmap_col < 2 || heatmap_row >= global_row_length - 2 || heatmap_col >= global_row_length - 2)
		return;//To close the window/field border

	int heatmap_index = 0;
	int heatmap_row_index = 0;
	int filter_index = 0;
	int filter_row = 0;
	int sum = 0;
	int heatmap_value = 0;
	int shared_row = 0;
	bool sharableThread = (threadIdx.x >= 2 && threadIdx.x < BLOCKROW - 2 && threadIdx.y >= 2 && threadIdx.y < BLOCKROW-2);



	//Summarize filtered values from neighbors
	for (int k = -2; k < 3; k++)
	{
		filter_row = (2 + k) * 5;// filter row length == 5
		heatmap_row_index = (heatmap_row + k) * global_row_length;
		shared_row = (threadIdx.y + k) * blockDim.x;
		for (int l = -2; l < 3; l++)
		{
			filter_index = 2 + l + filter_row;
			heatmap_index = l + heatmap_col + heatmap_row_index;
			shared_index = l + threadIdx.x + shared_row;

			//Get heatmap value from shared memory if possible
			if (sharableThread)
				heatmap_value = s[shared_index];
			else
				heatmap_value = d_scaled_heatmap[heatmap_index];

			sum += d_blur_filter[filter_index] * heatmap_value;
		}
	}

	int value = sum / 273;// WEIGHTSUM = 273;
	value = 0x00FF0000 | value << 24;
	d_blurred_heatmap[tid] = value;
}

// Updates the heatmap according to the agent positions
void Ped::Model::updateHeatmapStart()
{
	//Init the stream for asynch exec.
	//*stream1;// , stream2, stream3;
	cudaStream_t stream1;// = (cudaStream_t*)s1;
	cudaStreamCreate(&stream1);

	//Block sizes for different heatmaps
	int threads = 1024;
	dim3 heatmap_blocks(SIZE*SIZE / threads);
	dim3 scaled_heatmap_blocks(SCALED_SIZE*SCALED_SIZE / threads);
	int threadsGauss = BLOCKSIZE;
	int threadDim = BLOCKROW;
	int blockDim = sqrt(SCALED_SIZE*SCALED_SIZE / threadsGauss);
	dim3 gauss_heatmap_blocks(blockDim, blockDim);
	dim3 gauss_heatmap_threads(threadDim, threadDim);

	//Desireds Y's
	vector<float> tempx = agentCollection->getDesiredX();
	float* h_desiredXs = &(tempx[0]);

	//Desireds X's 
	vector<float> tempy = agentCollection->getDesiredY();
	float* h_desiredYs = &(tempy[0]);

	//Fade heatmap
	fade << <heatmap_blocks, threads, 0, stream1 >> >(d_heatmap);
	
	//Copy desired X's to device
	cudaMemcpyAsync(d_desiredXs, h_desiredXs, no_agents*sizeof(float), cudaMemcpyHostToDevice, stream1);
	

	//Copy desired Y's to device
	cudaMemcpyAsync(d_desiredYs, h_desiredYs, no_agents*sizeof(float), cudaMemcpyHostToDevice, stream1);


	//Set location contention based on desired
	locationContention << <heatmap_blocks, threads, 0, stream1 >> >(d_heatmap, d_desiredXs, d_desiredYs, d_agents);


	//Cut of values at max 255
	ceiling << <heatmap_blocks, threads, 0, stream1 >> >(d_heatmap);


	// Scale the data for visual representation
	scale << <scaled_heatmap_blocks, threads, 0, stream1 >> >(d_scaled_heatmap, d_SCALED_SIZE, d_heatmap);
	

	//Do the gaussing and get a nice blurr. Retrun the blurred array to host memory
	gauss << <gauss_heatmap_blocks, gauss_heatmap_threads, 0, stream1 >> >(d_scaled_heatmap, d_blurred_heatmap, d_SCALED_SIZE, d_blur_filter);
	cudaMemcpyAsync(blurred_heatmap[0], d_blurred_heatmap, SCALED_SIZE*SCALED_SIZE*sizeof(int), cudaMemcpyDeviceToHost, stream1);
}


void Ped::Model::waitCuda(){
	cudaDeviceSynchronize();
}

int Ped::Model::getHeatmapSize() const {
	return SCALED_SIZE;
}

