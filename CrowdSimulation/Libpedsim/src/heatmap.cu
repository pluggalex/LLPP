// Created for Low Level Parallel Programming 2017
//
// Implements the heatmap functionality. 
//
//#include "heatmap.h"
#include "ped_model.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <stdio.h>
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

	//Copy SIZE to device
	int temp_SIZE = SIZE;
	cudaMalloc((void**)&d_SIZE, sizeof(int));
	cudaMemcpy(d_SIZE, &temp_SIZE, sizeof(int), cudaMemcpyHostToDevice);

	//Copy CELLSIZE to device
	int temp_CELLSIZE = CELLSIZE;
	cudaMalloc((void**)&d_CELLSIZE, sizeof(int));
	cudaMemcpy(d_CELLSIZE, &temp_CELLSIZE, sizeof(int), cudaMemcpyHostToDevice);

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


	cudaMalloc(&d_desiredXs, no_agents*sizeof(float));
	cudaMalloc(&d_desiredYs, no_agents*sizeof(float));

}


/*
* Fades the blocks before each tick. 
*/
__global__
void fade(int* d_heatmap){
	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	d_heatmap[tid] = d_heatmap[tid] * 0.80;
}

/*
* Increase the red color for the block for each agent that wants to access it.
*/
__global__
void locationContention(int* d_heatmap, float* d_desiredXs, float* d_desiredYs, int* d_agents, int* d_SIZE){
	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	if (tid < *d_agents){
		int desX = d_desiredXs[tid];
		int desY = d_desiredYs[tid];
		int inc = 40;
		atomicAdd(&d_heatmap[desY*(*d_SIZE) + desX], inc);
	}
}

/*
* Make sure the color code isn't over 255(0xFF).
*/
__global__
void ceiling(int* d_heatmap){
	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	int ceiling = 255;
	if (d_heatmap[tid] > ceiling)
		d_heatmap[tid] = ceiling;
}

__global__
void scale(int* d_scaled_heatmap, int* d_SCALED_SIZE, int* d_heatmap, int* d_SIZE, int* d_CELLSIZE){
	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	int s_row = tid / (*d_SCALED_SIZE); 
	int s_col = tid; //Column is either thread id if row == 0. Otherwise thread id - row * rowsize
	if (s_row)
		s_col = tid - s_row * (*d_SCALED_SIZE);

	//Scale down row and column to heatmap index
	int row = s_row / *d_CELLSIZE; 
	int col = s_col / *d_CELLSIZE;
	int index = col + row * (*d_SIZE);

	d_scaled_heatmap[tid] = d_heatmap[index];
}



__global__
void gauss(int* d_scaled_heatmap, int* d_blurred_heatmap, int* d_SCALED_SIZE, int* d_blur_filter){
	if (blockIdx.x >= 2 - gridDim.x || threadIdx.x >= 2 - blockDim.x || blockIdx.x < 2 || threadIdx.x < 2)
		return;//To close the window/field border

	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	int heatmap_index = 0;
	int heatmap_row_index = 0;
	int filter_index = 0;
	int filter_row = 0;
	int sum = 0;

	int heatmap_row = tid / *d_SCALED_SIZE;
	int heatmap_col = tid;
	if (heatmap_row)
		heatmap_col = tid - heatmap_row * (*d_SCALED_SIZE);

	//Summarize filtered values from neighbors
	for (int k = -2; k < 3; k++)
	{
		filter_row = (2 + k) * 5;// filter row length == 5
		heatmap_row_index = (heatmap_row + k) * (*d_SCALED_SIZE);
		for (int l = -2; l < 3; l++)
		{
			heatmap_index = l + heatmap_col + heatmap_row_index;
			filter_index = 2 + l + filter_row;
			sum += d_blur_filter[filter_index] * d_scaled_heatmap[heatmap_index];
		}
	}
	int value = sum / 273;// WEIGHTSUM = 273;
	value = 0x00FF0000 | value << 24;
	d_blurred_heatmap[tid] = value;
}

// Updates the heatmap according to the agent positions
void Ped::Model::updateHeatmapSeq()
{
	cudaStream_t stream1;
	cudaStreamCreate(&stream1);
	int threadsPerBlock = 1024;
	int heatmap_blocks = SIZE*SIZE / threadsPerBlock;
	

	//Fade heatmap
	fade << <heatmap_blocks, threadsPerBlock, 0, stream1 >> >(d_heatmap);
		
	//Get and copy desired X's to device
	vector<float> tempx = agentCollection->getDesiredX();
	float* h_desiredXs = &(tempx[0]);
	cudaMemcpyAsync(d_desiredXs, h_desiredXs, no_agents*sizeof(float), cudaMemcpyHostToDevice);
	

	//Get and copy desired Y's to device
	vector<float> tempy = agentCollection->getDesiredY();
	float* h_desiredYs = &(tempy[0]);
	cudaMemcpyAsync(d_desiredYs, h_desiredYs, no_agents*sizeof(float), cudaMemcpyHostToDevice);


	//Set location contention based on desired
	locationContention << <heatmap_blocks, threadsPerBlock,  0, stream1 >> >(d_heatmap, d_desiredXs, d_desiredYs, d_agents, d_SIZE);


	//Cut of values at max 255
	ceiling << <heatmap_blocks, threadsPerBlock, 0, stream1 >> >(d_heatmap);


	// Scale the data for visual representation
	int scaled_heatmap_blocks = SCALED_SIZE*SCALED_SIZE / threadsPerBlock;
	scale << <scaled_heatmap_blocks, threadsPerBlock, 0, stream1 >> >(d_scaled_heatmap, d_SCALED_SIZE, d_heatmap, d_SIZE, d_CELLSIZE);
	

	//Do the gaussing and get a nice blurr. Retrun the blurred array to host memory
	gauss << <scaled_heatmap_blocks, threadsPerBlock, 0, stream1 >> >(d_scaled_heatmap, d_blurred_heatmap, d_SCALED_SIZE, d_blur_filter);
	cudaMemcpy(blurred_heatmap[0], d_blurred_heatmap, SCALED_SIZE*SCALED_SIZE*sizeof(int), cudaMemcpyDeviceToHost);
}



int Ped::Model::getHeatmapSize() const {
	return SCALED_SIZE;
}

