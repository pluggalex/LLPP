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

	//CUDA init
	//enum cudaError malloc_status = cudaMallocPitch((void**)&d_heatmap, pitch, SIZE*sizeof(int), SIZE*sizeof(int));
	//enum cudaError memcpy_status = cudaMemcpy(d_heatmap, heatmap, SIZE*SIZE*sizeof(int), cudaMemcpyHostToDevice);

	enum cudaError malloc_status = cudaMalloc((void**)&d_heatmap, SIZE*SIZE*sizeof(int));
	malloc_status = cudaMalloc((void**)&d_heatmap_row_size, sizeof(int));
	enum cudaError memset_status = cudaMemset(d_heatmap_row_size, SIZE, sizeof(int));

	malloc_status = cudaMalloc((void**)&d_scaled_heatmap, SIZE*SIZE*CELLSIZE*CELLSIZE*sizeof(int));
	memset_status = cudaMemset(d_scaled_heatmap, 0, SIZE*SIZE*CELLSIZE*CELLSIZE*sizeof(int));

	malloc_status = cudaMalloc((void**)&d_scaled_heatmap_row_size, sizeof(int));
	memset_status = cudaMemset(d_scaled_heatmap_row_size, SIZE*CELLSIZE, sizeof(int));

	//Blurred heatmap
	malloc_status = cudaMalloc((void**)&d_blurred_heatmap, SIZE*SIZE*CELLSIZE*CELLSIZE*sizeof(int));
	memset_status = cudaMemset(d_blurred_heatmap, 0, SIZE*SIZE*CELLSIZE*CELLSIZE*sizeof(int));

	//Blur filter
	// Weights
	const int w[5][5] = {
		{ 1, 4, 7, 4, 1 },
		{ 4, 16, 26, 16, 4 },
		{ 7, 26, 41, 26, 7 },
		{ 4, 16, 26, 16, 4 },
		{ 1, 4, 7, 4, 1 }
	};
	malloc_status = cudaMalloc((void**)&d_blur_filter, 5*5*sizeof(int));
	enum cudaError memcpy_status = cudaMemcpy(d_blur_filter, w, 5*5*sizeof(int), cudaMemcpyHostToDevice);

}

/*
for (int x = 0; x < SIZE; x++)
{
for (int y = 0; y < SIZE; y++)
{
// heat fades
heatmap[y][x] = (int)round(heatmap[y][x] * 0.80);
}
}
*/
__global__
void fade(int* d_heatmap){
	// heat fades
	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	d_heatmap[tid] = __double2int_rd(d_heatmap[tid] * 0.80);
}

__global__
void locationContention(int* d_heatmap, float* d_desiredXs, float* d_desiredYs, int* d_agents){
	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	if (tid < *d_agents){
		int desX = d_desiredXs[tid];
		int desY = d_desiredYs[tid];
		int inc = 40;
		atomicAdd(&d_heatmap[desY*1024 + desX], inc);
		//d_heatmap[y][x] += 40 ;
	}
}

__global__
void ceiling(int* d_heatmap){
	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	int ceiling = 255;
	if (d_heatmap[tid] > ceiling)
		d_heatmap[tid] = ceiling;
}

__global__
void scale(int* d_scaled_heatmap, int* d_scaled_heatmap_row_size, int* d_heatmap, int* d_heatmap_row_size){
	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	int cellsize = 5;
	int s_row_size = 1024 * 5;
	int s_row = __double2int_rd(tid / s_row_size);//__double2int_rd(blockIdx.x / cellsize);
	int s_col = tid;
	if (s_row)
		s_col = tid - s_row * s_row_size;

	int row = s_row / cellsize;
	int col = s_col / cellsize;
	int index = col + row * 1024;// (*d_heatmap_row_size);
	d_scaled_heatmap[tid] = d_heatmap[index];

	//if (tid == 1024)
	//	printf("s_row: %d, s_col: %d, row: %d, col:%d, s_row_size:%d\n", s_row, s_col, row, col, s_row_size);
}


/*
int sum = 0;
for (int k = -2; k < 3; k++)
{
	for (int l = -2; l < 3; l++)
	{
		sum += w[2 + k][2 + l] * scaled_heatmap[i + k][j + l];
	}
}
int value = sum / WEIGHTSUM;
blurred_heatmap[i][j] = 0x00FF0000 | value << 24;
*/
__global__
void gauss(int* d_scaled_heatmap, int* d_blurred_heatmap, int* d_blur_filter){
	if (blockIdx.x >= 2 - gridDim.x || threadIdx.x >= 2 - blockDim.x || blockIdx.x < 2 || threadIdx.x < 2)
		return;//To close the window/field border

	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	int i_heatmap = 0;
	int h_row_index = 0;
	int i_filter = 0;
	int filter_row = 0;
	int sum = 0;

	int h_row = tid / (1024 * 5); //row length == (1024 * 5);
	int h_col = tid;
	if (h_row)
		h_col = tid - h_row * 1024 * 5;

	for (int k = -2; k < 3; k++)
	{
		filter_row = (2 + k) * 5;// row length == 5
		h_row_index = (h_row + k) * 1024*5; //scaled heatmap row length = 1024*5
		for (int l = -2; l < 3; l++)
		{
			i_heatmap = l + h_col + h_row_index;
			i_filter = 2 + l + filter_row;
			sum += d_blur_filter[i_filter] * d_scaled_heatmap[i_heatmap];
		}
	}
	int value = sum / 273;// WEIGHTSUM = 273;
	value = 0x00FF0000 | value << 24;
	d_blurred_heatmap[tid] = value;
}

// Updates the heatmap according to the agent positions
void Ped::Model::updateHeatmapSeq()
{
	//INIT d_heatmap
	enum cudaError memcpy_status = cudaMemcpy(d_heatmap, heatmap[0], SIZE*SIZE*sizeof(int), cudaMemcpyHostToDevice);
	int threadsPerBlock = 512;
	int blocks = SIZE*SIZE / threadsPerBlock;
	
	//Fade heatmap
	fade << <blocks, threadsPerBlock >> >(d_heatmap);
	enum cudaError sync_status = cudaDeviceSynchronize();
	
	//Init desireds for cuda
	vector<float> tempx = agentCollection->getDesiredX();
	vector<float> tempy = agentCollection->getDesiredY();
	float* Xs = &(tempx[0]);
	float* Ys = &(tempy[0]);
	int no_agents = agentCollection->size();
	
	int* d_size;
	cudaMalloc(&d_size, sizeof(int));
	cudaMemcpy(d_size, &no_agents, sizeof(int), cudaMemcpyHostToDevice);

	float* d_desiredXs;// = &(Xs[0]);
	cudaMalloc(&d_desiredXs, no_agents*sizeof(float));
	cudaMemcpy(d_desiredXs, Xs, no_agents*sizeof(float), cudaMemcpyHostToDevice);
	
	float* d_desiredYs;// = &(Ys[0]);	
	cudaMalloc(&d_desiredYs, no_agents*sizeof(float));
	cudaMemcpy(d_desiredYs, Ys, no_agents*sizeof(float), cudaMemcpyHostToDevice);

	//Set location contention based on desired
	locationContention <<<blocks, threadsPerBlock>>>(d_heatmap, d_desiredXs, d_desiredYs, d_size);
	sync_status = cudaDeviceSynchronize();
	
	//Cut of values to max 255
	ceiling<< <blocks, threadsPerBlock >> >(d_heatmap);
	cudaMemcpy(heatmap[0], d_heatmap, SIZE*SIZE*sizeof(int), cudaMemcpyDeviceToHost);
	sync_status = cudaDeviceSynchronize();

	// Scale the data for visual representation
	blocks = SIZE*SIZE*CELLSIZE*CELLSIZE / threadsPerBlock;
	scale << <blocks, threadsPerBlock >> >(d_scaled_heatmap, d_scaled_heatmap_row_size, d_heatmap, d_heatmap_row_size);
	cudaMemcpy(scaled_heatmap[0], d_scaled_heatmap, SIZE*SIZE*CELLSIZE*CELLSIZE*sizeof(int), cudaMemcpyDeviceToHost);
	sync_status = cudaDeviceSynchronize();
	
	/*// Weights for blur filter
	const int w[5][5] = {
		{ 1, 4, 7, 4, 1 },
		{ 4, 16, 26, 16, 4 },
		{ 7, 26, 41, 26, 7 },
		{ 4, 16, 26, 16, 4 },
		{ 1, 4, 7, 4, 1 }
	};
#define WEIGHTSUM 273
	// Apply gaussian blurfilter		       
	for (int i = 2; i < SCALED_SIZE - 2; i++)
	{
		for (int j = 2; j < SCALED_SIZE - 2; j++)
		{
			int sum = 0;
			for (int k = -2; k < 3; k++)
			{
				for (int l = -2; l < 3; l++)
				{
					sum += w[2 + k][2 + l] * scaled_heatmap[i + k][j + l];
				}
			}
			int value = sum / WEIGHTSUM;
			blurred_heatmap[i][j] = 0x00FF0000 | value << 24;
		}
	}
	*/
	blocks = SCALED_SIZE*SCALED_SIZE / threadsPerBlock;
	gauss << <blocks, threadsPerBlock >> >(d_scaled_heatmap, d_blurred_heatmap, d_blur_filter);
	cudaMemcpy(scaled_heatmap[0], d_scaled_heatmap, SIZE*SIZE*CELLSIZE*CELLSIZE*sizeof(int), cudaMemcpyDeviceToHost);
	cudaMemcpy(blurred_heatmap[0], d_blurred_heatmap, SIZE*SIZE*CELLSIZE*CELLSIZE*sizeof(int), cudaMemcpyDeviceToHost);
	sync_status = cudaDeviceSynchronize();
}



int Ped::Model::getHeatmapSize() const {
	return SCALED_SIZE;
}

