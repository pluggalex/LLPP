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
	int xy = threadIdx.x + blockIdx.x * blockDim.x;
	d_heatmap[xy] = __double2int_rd(d_heatmap[xy] * 0.80);
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

// Updates the heatmap according to the agent positions
void Ped::Model::updateHeatmapSeq()
{
	enum cudaError malloc_status = cudaMalloc((void**)&d_heatmap, SIZE*SIZE*sizeof(int));
	enum cudaError memcpy_status = cudaMemcpy(d_heatmap, heatmap[0], SIZE*SIZE*sizeof(int), cudaMemcpyHostToDevice);
	int threadsPerBlock = 512;
	int blocks = SIZE*SIZE / threadsPerBlock;
	
	fade << <blocks, threadsPerBlock >> >(d_heatmap);
	enum cudaError sync_status = cudaDeviceSynchronize();
	cudaMemcpy(heatmap[0], d_heatmap, SIZE*SIZE*sizeof(int), cudaMemcpyDeviceToHost);
	
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

	locationContention <<<blocks, threadsPerBlock>>>(d_heatmap, d_desiredXs, d_desiredYs, d_size);
	cudaMemcpy(heatmap[0], d_heatmap, SIZE*SIZE*sizeof(int), cudaMemcpyDeviceToHost);
	sync_status = cudaDeviceSynchronize();
	//Barier need to wait for the mess up stairs to clear before fixing the max heat color bellow

	for (int x = 0; x < SIZE; x++)
	{
		for (int y = 0; y < SIZE; y++)
		{
			heatmap[y][x] = heatmap[y][x] < 255 ? heatmap[y][x] : 255;
		}
	}

	// Scale the data for visual representation
	for (int y = 0; y < SIZE; y++)
	{
		for (int x = 0; x < SIZE; x++)
		{
			int value = heatmap[y][x];
			//if (value != 0)
			//	printf("heatmap[%d][%d] = %ld\n", y, x, value);
			for (int cellY = 0; cellY < CELLSIZE; cellY++)
			{
				for (int cellX = 0; cellX < CELLSIZE; cellX++)
				{
					scaled_heatmap[y * CELLSIZE + cellY][x * CELLSIZE + cellX] = value;
				}
			}
		}
	}

	// Weights for blur filter
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
}



int Ped::Model::getHeatmapSize() const {
	return SCALED_SIZE;
}

