/*
 * Copyright 2020 Naval Postgraduate School
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <nps_uw_multibeam_sonar/sonar_calculation_cuda.cuh>

// #include <math.h>
#include <assert.h>

// For complex numbers
#include <thrust/complex.h>
#include <cuComplex.h>

// For rand() function
#include <unistd.h>
#include <curand.h>
#include <curand_kernel.h>

// For FFT
#include <cufft.h>
#include <cufftw.h>
#include <thrust/device_vector.h>
#include <list>

#include <chrono>

#define BLOCK_SIZE 32

static inline void _safe_cuda_call(cudaError err, const char *msg,
                                   const char *file_name, const int line_number)
{
  if (err != cudaSuccess)
  {
    fprintf(stderr, "%s\n\nFile: %s\n\nLine Number: %d\n\nReason: %s\n",
            msg, file_name, line_number, cudaGetErrorString(err));
    std::cin.get();
    exit(EXIT_FAILURE);
  }
}

#define SAFE_CALL(call, msg) _safe_cuda_call((call), (msg), __FILE__, __LINE__)

///////////////////////////////////////////////////////////////////////////
// Incident Angle Calculation Function
// incidence angle is target's normal angle accounting for the ray's azimuth
// and elevation
__device__ float compute_incidence(float azimuth, float elevation, float *normal)
{
  // ray normal from camera azimuth and elevation
  float camera_x = cosf(-azimuth) * cosf(elevation);
  float camera_y = sinf(-azimuth) * cosf(elevation);
  float camera_z = sinf(elevation);
  float ray_normal[3] = {camera_x, camera_y, camera_z};

  // target normal with axes compensated to camera axes
  float target_normal[3] = {normal[2], -normal[0], -normal[1]};

  // dot product
  float dot_product = ray_normal[0] * target_normal[0]
                      + ray_normal[1] * target_normal[1]
                      + ray_normal[2] * target_normal[2];

  return M_PI - acosf(dot_product);
}

///////////////////////////////////////////////////////////////////////////
__device__ __host__ float unnormalized_sinc(float t)
{
  if (abs(t) < 1E-8)
    return 1.0;
  else
    return sin(t) / t;
}

///////////////////////////////////////////////////////////////////////////
template <typename T>
__global__ void column_sums_reduce(const T *__restrict__ in, T *__restrict__ out, size_t width, size_t height)
{

  __shared__ T sdata[BLOCK_SIZE][BLOCK_SIZE + 1];
  size_t idx = threadIdx.x + blockDim.x * blockIdx.x;
  size_t width_stride = gridDim.x * blockDim.x;
  size_t full_width = (width & (~((unsigned long long)(BLOCK_SIZE - 1)))) + ((width & (BLOCK_SIZE - 1)) ? BLOCK_SIZE : 0); // round up to next block
  for (size_t w = idx; w < full_width; w += width_stride)
  { // grid-stride loop across matrix width
    sdata[threadIdx.y][threadIdx.x] = 0;
    size_t in_ptr = w + threadIdx.y * width;
    for (size_t h = threadIdx.y; h < height; h += BLOCK_SIZE)
    { // block-stride loop across matrix height
      sdata[threadIdx.y][threadIdx.x] += (w < width) ? in[in_ptr] : 0;
      in_ptr += width * BLOCK_SIZE;
    }
    __syncthreads();
    T my_val = sdata[threadIdx.x][threadIdx.y];
    for (int i = warpSize >> 1; i > 0; i >>= 1) // warp-wise parallel sum reduction
      my_val += __shfl_xor_sync(0xFFFFFFFFU, my_val, i);
    __syncthreads();
    if (threadIdx.x == 0)
      sdata[0][threadIdx.y] = my_val;
    __syncthreads();
    if ((threadIdx.y == 0) && ((w) < width))
      out[w] = sdata[0][threadIdx.x];
  }
}

__global__ void gpu_matrix_mult(float *a, float *b, float *c, int m, int n, int k)
{
  int row = blockIdx.y * blockDim.y + threadIdx.y;
  int col = blockIdx.x * blockDim.x + threadIdx.x;
  float sum = 0;
  if (col < k && row < m)
  {
    for (int i = 0; i < n; i++)
    {
      sum += a[row * n + i] * b[i * k + col];
    }
    c[row * k + col] = sum;
  }
}

__global__ void gpu_diag_matrix_mult(float *Val, int *RowPtr, float *diagVals, int total_rows)
{
  const int row = threadIdx.x + blockIdx.x * blockDim.x;
  if (row < total_rows)
  {
    for (int i = RowPtr[row]; i < RowPtr[row + 1]; i++)
    {
      Val[i] = diagVals[row] * Val[i];
    }
  }
}

///////////////////////////////////////////////////////////////////////////
// Sonar Claculation Function
__global__ void sonar_calculation(thrust::complex<float> *P_Beams,
                                  float *depth_image,
                                  float *normal_image,
                                  int width,
                                  int height,
                                  int depth_image_step,
                                  int normal_image_step,
                                  float *rand_image,
                                  int rand_image_step,
                                  float *reflectivity_image,
                                  int reflectivity_image_step,
                                  float hPixelSize,
                                  float vPixelSize,
                                  float hFOV,
                                  float vFOV,
                                  float beam_azimuthAngleWidth,
                                  float beam_elevationAngleWidth,
                                  float ray_azimuthAngleWidth,
                                  float *ray_elevationAngles,
                                  float ray_elevationAngleWidth,
                                  float soundSpeed,
                                  float sourceTerm,
                                  int nBeams, int nRays,
                                  int raySkips,
                                  float sonarFreq, float delta_f,
                                  int nFreq, float bandwidth,
                                  float maxDistance,
                                  float attenuation,
                                  float area_scaler)
{
  // 2D Index of current thread
  const int beam = blockIdx.x * blockDim.x + threadIdx.x;
  const int ray = blockIdx.y * blockDim.y + threadIdx.y;

  //Only valid threads perform memory I/O
  if ((beam < width) && (ray < height) && (ray % raySkips == 0))
  {
    // Location of the image pixel
    const int depth_index = ray * depth_image_step / sizeof(float) + beam;
    const int normal_index = ray * normal_image_step / sizeof(float) + (3 * beam);
    const int rand_index = ray * rand_image_step / sizeof(float) + (2 * beam);
    const int reflectivity_index = ray * reflectivity_image_step / sizeof(float) + beam;

    // Input parameters for ray processing
    float distance = depth_image[depth_index] * 1.0f;
    float normal[3] = {normal_image[normal_index],
                      normal_image[normal_index + 1],
                      normal_image[normal_index + 2]};

    // Beam pattern
    // only one column of rays for each beam at beam center, interference calculated later
    float azimuthBeamPattern = 1.0;
    float elevationBeamPattern = 1.0;
    // float elevationBeamPattern = abs(unnormalized_sinc(M_PI * 0.884
    //    				                  / (beam_elevationAngleWidth) * sin(ray_elevationAngles[ray])));

    // printf("angles %f", ray_elevationAngles[ray]);

    // incidence angle (taking that of normal_image)
    float incidence = acos(normal[2]); // compute_incidence(ray_azimuthAngle, ray_elevationAngle, normal);

    // ----- Point scattering model ------ //
    // Gaussian noise generated using opencv RNG
    float xi_z = rand_image[rand_index];
    float xi_y = rand_image[rand_index + 1];

    // Calculate amplitude
    thrust::complex<float> randomAmps = thrust::complex<float>(xi_z / sqrt(2.0), xi_y / sqrt(2.0));
    thrust::complex<float> lambert_sqrt =
        thrust::complex<float>(sqrt(reflectivity_image[reflectivity_index]) * cos(incidence), 0.0);
    thrust::complex<float> beamPattern =
        thrust::complex<float>(azimuthBeamPattern * elevationBeamPattern, 0.0);
    thrust::complex<float> targetArea_sqrt = thrust::complex<float>(sqrt(distance * area_scaler), 0.0);
    thrust::complex<float> propagationTerm =
        thrust::complex<float>(1.0 / pow(distance, 2.0) * exp(-2.0 * attenuation * distance), 0.0);
    thrust::complex<float> amplitude = randomAmps * thrust::complex<float>(sourceTerm, 0.0)
                                     * propagationTerm * beamPattern * lambert_sqrt * targetArea_sqrt;

    // Max distance cut-off
    if (distance > maxDistance)
      amplitude = thrust::complex<float>(0.0, 0.0);

    // Summation of Echo returned from a signal (frequency domain)
    for (size_t f = 0; f < nFreq; f++)
    {
      float freq;
      if (nFreq % 2 == 0)
        freq = delta_f * (-nFreq / 2.0 + f*1.0f + 1.0);
      else
        freq = delta_f * (-(nFreq - 1) / 2.0 + f*1.0f + 1.0);
      float kw = 2.0 * M_PI * freq / soundSpeed; // wave vector

      // Transmit spectrum, frequency domain
      thrust::complex<float> kernel = exp(thrust::complex<float>(0.0f, 2.0f * distance * kw)) * amplitude;
      P_Beams[beam * nFreq * (int)(nRays / raySkips) + (int)(ray / raySkips) * nFreq + f] =
          thrust::complex<float>(kernel.real() , kernel.imag());
    }
  }
}

///////////////////////////////////////////////////////////////////////////
namespace NpsGazeboSonar
{

  // CUDA Device Checker Wrapper
  void check_cuda_init_wrapper(void)
  {
    // Check CUDA device
    cudaDeviceSynchronize();
    cudaError_t error = cudaGetLastError();
    if (error != cudaSuccess)
    {
      fprintf(stderr, "ERROR: %s\n", cudaGetErrorString(error));
      exit(-1);
    }
  }

  // Sonar Claculation Function Wrapper
  CArray2D sonar_calculation_wrapper(const cv::Mat &depth_image,
                                     const cv::Mat &normal_image,
                                     const cv::Mat &rand_image,
                                     double _hPixelSize,
                                     double _vPixelSize,
                                     double _hFOV,
                                     double _vFOV,
                                     double _beam_azimuthAngleWidth,
                                     double _beam_elevationAngleWidth,
                                     double _ray_azimuthAngleWidth,
                                     float *_ray_elevationAngles,
                                     double _ray_elevationAngleWidth,
                                     double _soundSpeed,
                                     double _maxDistance,
                                     double _sourceLevel,
                                     int _nBeams, int _nRays,
                                     int _raySkips,
                                     double _sonarFreq,
                                     double _bandwidth,
                                     int _nFreq,
                                     const cv::Mat &reflectivity_image,
                                     double _attenuation,
                                     float *window,
                                     float **beamCorrector,
                                     float beamCorrectorSum,
                                     bool debugFlag)
  {
    auto start = std::chrono::high_resolution_clock::now();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    if (debugFlag)
      start = std::chrono::high_resolution_clock::now();

    // ----  Allocation of properties parameters  ---- //
    const float hPixelSize = (float)_hPixelSize;
    const float vPixelSize = (float)_vPixelSize;
    const float hFOV = (float)_hFOV;
    const float vFOV = (float)_vFOV;
    const float beam_elevationAngleWidth = (float)_beam_elevationAngleWidth;
    const float beam_azimuthAngleWidth = (float)_beam_azimuthAngleWidth;
    const float ray_elevationAngleWidth = (float)_ray_elevationAngleWidth;
    const float ray_azimuthAngleWidth = (float)_ray_azimuthAngleWidth;
    const float soundSpeed = (float)_soundSpeed;
    const float maxDistance = (float)_maxDistance;
    const float sonarFreq = (float)_sonarFreq;
    const float bandwidth = (float)_bandwidth;
    const float attenuation = (float)_attenuation;
    const int nBeams = _nBeams;
    const int nRays = _nRays;
    const int nFreq = _nFreq;
    const int raySkips = _raySkips;

    //#######################################################//
    //###############    Sonar Calculation   ################//
    //#######################################################//
    // ---------   Calculation parameters   --------- //
    const float max_distance = maxDistance;
    // Signal
    const float delta_f = bandwidth/nFreq;
    // Precalculation
    const float area_scaler = ray_azimuthAngleWidth * ray_elevationAngleWidth;
    const float sourceLevel = (float)_sourceLevel;                     // db re 1 muPa;
    const float pref = 1e-6;                                           // 1 micro pascal (muPa);
    const float sourceTerm = sqrt(pow(10, (sourceLevel / 10))) * pref; // source term

    // ---------   Allocate GPU memory for image   --------- //
    //Calculate total number of bytes of input and output image
    const int depth_image_Bytes = depth_image.step * depth_image.rows;
    const int normal_image_Bytes = normal_image.step * normal_image.rows;
    const int rand_image_Bytes = rand_image.step * rand_image.rows;
    const int reflectivity_image_Bytes = reflectivity_image.step * reflectivity_image.rows;
    const int ray_elevationAngles_Bytes = sizeof(float) * nRays;

    //Allocate device memory
    float *d_depth_image, *d_normal_image, *d_rand_image, *d_reflectivity_image, *ray_elevationAngles, *d_ray_elevationAngles;
    SAFE_CALL(cudaMalloc((void **)&d_depth_image, depth_image_Bytes), "CUDA Malloc Failed");
    SAFE_CALL(cudaMalloc((void **)&d_normal_image, normal_image_Bytes), "CUDA Malloc Failed");
    SAFE_CALL(cudaMalloc((void **)&d_rand_image, rand_image_Bytes), "CUDA Malloc Failed");
    SAFE_CALL(cudaMalloc((void **)&d_reflectivity_image, reflectivity_image_Bytes), "CUDA Malloc Failed");
    cudaMallocHost((void **)&ray_elevationAngles, ray_elevationAngles_Bytes);
    SAFE_CALL(cudaMalloc((void **)&d_ray_elevationAngles, ray_elevationAngles_Bytes), "CUDA Malloc Failed");
    for (size_t ray = 0; ray < nRays; ray ++)
      ray_elevationAngles[ray] = _ray_elevationAngles[ray];

    //Copy data from OpenCV input image to device memory
    SAFE_CALL(cudaMemcpy(d_depth_image, depth_image.ptr(), depth_image_Bytes,
                  cudaMemcpyHostToDevice), "CUDA Memcpy Failed");
    SAFE_CALL(cudaMemcpy(d_normal_image, normal_image.ptr(), normal_image_Bytes,
                  cudaMemcpyHostToDevice),"CUDA Memcpy Failed");
    SAFE_CALL(cudaMemcpy(d_rand_image, rand_image.ptr(), rand_image_Bytes,
                  cudaMemcpyHostToDevice),"CUDA Memcpy Failed");
    SAFE_CALL(cudaMemcpy(d_reflectivity_image, reflectivity_image.ptr(), reflectivity_image_Bytes,
                  cudaMemcpyHostToDevice), "CUDA Memcpy Failed");
    SAFE_CALL(cudaMemcpy(d_ray_elevationAngles, ray_elevationAngles, ray_elevationAngles_Bytes,
                  cudaMemcpyHostToDevice), "CUDA Memcpy Failed");

    //Specify a reasonable block size
    const dim3 block(BLOCK_SIZE, BLOCK_SIZE);

    //Calculate grid size to cover the whole image
    const dim3 grid((depth_image.cols + block.x - 1) / block.x,
                    (depth_image.rows + block.y - 1) / block.y);

    // Beam data array
    thrust::complex<float> *P_Beams;
    thrust::complex<float> *d_P_Beams;
    const int P_Beams_N = nBeams * (int)(nRays / raySkips) * (nFreq + 1);
    const int P_Beams_Bytes = sizeof(thrust::complex<float>) * P_Beams_N;
    SAFE_CALL(cudaMallocHost((void **)&P_Beams, P_Beams_Bytes), "CUDA Malloc Failed");
    SAFE_CALL(cudaMalloc((void **)&d_P_Beams, P_Beams_Bytes), "CUDA Malloc Failed");

    //Launch the beamor conversion kernel
    sonar_calculation<<<grid, block>>>(d_P_Beams,
                                       d_depth_image,
                                       d_normal_image,
                                       normal_image.cols,
                                       normal_image.rows,
                                       depth_image.step,
                                       normal_image.step,
                                       d_rand_image,
                                       rand_image.step,
                                       d_reflectivity_image,
                                       reflectivity_image.step,
                                       hPixelSize,
                                       vPixelSize,
                                       hFOV,
                                       vFOV,
                                       beam_azimuthAngleWidth,
                                       beam_elevationAngleWidth,
                                       ray_azimuthAngleWidth,
                                       d_ray_elevationAngles,
                                       ray_elevationAngleWidth,
                                       soundSpeed,
                                       sourceTerm,
                                       nBeams, nRays,
                                       raySkips,
                                       sonarFreq, delta_f,
                                       nFreq, bandwidth,
                                       max_distance,
                                       attenuation,
                                       area_scaler);

    //Synchronize to check for any kernel launch errors
    SAFE_CALL(cudaDeviceSynchronize(), "Kernel Launch Failed");

    //Copy back data from destination device meory to OpenCV output image
    SAFE_CALL(cudaMemcpy(P_Beams, d_P_Beams, P_Beams_Bytes,
                         cudaMemcpyDeviceToHost), "CUDA Memcpy Failed");

    // Free GPU memory
    cudaFree(d_depth_image);
    cudaFree(d_normal_image);
    cudaFree(d_rand_image);
    cudaFree(d_reflectivity_image);
    cudaFree(d_P_Beams);
    cudaFree(d_ray_elevationAngles);
    cudaFreeHost(ray_elevationAngles);

    // For calc time measure
    if (debugFlag)
    {
      stop = std::chrono::high_resolution_clock::now();
      duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
      printf("GPU Sonar Computation Time %lld/100 [s]\n",
              static_cast<long long int>(duration.count() / 10000));
      start = std::chrono::high_resolution_clock::now();
    }

    //########################################################//
    //#########   Summation, Culling and windowing   #########//
    //########################################################//
    // Preallocate an array for return
    CArray2D P_Beams_F(CArray(nFreq), nBeams);
    // GPU grids and rows
    unsigned int grid_rows, grid_cols;
    dim3 dimBlock(BLOCK_SIZE, BLOCK_SIZE);

    // GPU Ray summation using column sum
    float *P_Ray_real, *P_Ray_imag;
    float *d_P_Ray_real, *d_P_Ray_imag;
    const int P_Ray_N = (int)(nRays / raySkips) * (nFreq);
    const int P_Ray_Bytes = sizeof(float) * P_Ray_N;
    float *P_Ray_F_real, *P_Ray_F_imag;
    float *d_P_Ray_F_real, *d_P_Ray_F_imag;
    const int P_Ray_F_N = (nFreq)*1;
    const int P_Ray_F_Bytes = sizeof(float) * P_Ray_F_N;
    cudaMallocHost((void **)&P_Ray_real, P_Ray_Bytes);
    cudaMallocHost((void **)&P_Ray_imag, P_Ray_Bytes);
    cudaMallocHost((void **)&P_Ray_F_real, P_Ray_F_Bytes);
    cudaMallocHost((void **)&P_Ray_F_imag, P_Ray_F_Bytes);
    SAFE_CALL(cudaMalloc((void **)&d_P_Ray_real, P_Ray_Bytes), "CUDA Malloc Failed");
    SAFE_CALL(cudaMalloc((void **)&d_P_Ray_imag, P_Ray_Bytes), "CUDA Malloc Failed");
    SAFE_CALL(cudaMalloc((void **)&d_P_Ray_F_real, P_Ray_F_Bytes), "CUDA Malloc Failed");
    SAFE_CALL(cudaMalloc((void **)&d_P_Ray_F_imag, P_Ray_F_Bytes), "CUDA Malloc Failed");

    dim3 dimGrid_Ray((nFreq + BLOCK_SIZE - 1) / BLOCK_SIZE);

    for (size_t beam = 0; beam < nBeams; beam ++)
    {
      for (size_t ray = 0; ray < (int)(nRays / raySkips); ray++)
      {
        for (size_t f = 0; f < nFreq; f++)
        {
          P_Ray_real[ray * nFreq + f] =
              P_Beams[beam * nFreq * (int)(nRays / raySkips) + ray * nFreq + f].real();
          P_Ray_imag[ray * nFreq + f] =
              P_Beams[beam * nFreq * (int)(nRays / raySkips) + ray * nFreq + f].imag();
        }
      }

      SAFE_CALL(cudaMemcpy(d_P_Ray_real, P_Ray_real, P_Ray_Bytes, cudaMemcpyHostToDevice),
                "CUDA Memcpy Failed");
      SAFE_CALL(cudaMemcpy(d_P_Ray_imag, P_Ray_imag, P_Ray_Bytes, cudaMemcpyHostToDevice),
                "CUDA Memcpy Failed");

      column_sums_reduce<<<dimGrid_Ray, dimBlock>>>(d_P_Ray_real, d_P_Ray_F_real,
                                                    nFreq, (int)(nRays / raySkips));
      column_sums_reduce<<<dimGrid_Ray, dimBlock>>>(d_P_Ray_imag, d_P_Ray_F_imag,
                                                    nFreq, (int)(nRays / raySkips));

      SAFE_CALL(cudaMemcpy(P_Ray_F_real, d_P_Ray_F_real, P_Ray_F_Bytes,
                           cudaMemcpyDeviceToHost), "CUDA Memcpy Failed");
      SAFE_CALL(cudaMemcpy(P_Ray_F_imag, d_P_Ray_F_imag, P_Ray_F_Bytes,
                           cudaMemcpyDeviceToHost), "CUDA Memcpy Failed");
      SAFE_CALL(cudaDeviceSynchronize(), "Kernel Launch Failed");

      for (size_t f = 0; f < nFreq; f++)
        P_Beams_F[beam][f] = Complex(P_Ray_F_real[f], P_Ray_F_imag[f]);
    }

    // free memory
    cudaFreeHost(P_Beams);
    cudaFreeHost(P_Ray_real);
    cudaFreeHost(P_Ray_imag);
    cudaFreeHost(P_Ray_F_real);
    cudaFreeHost(P_Ray_F_imag);
    cudaFree(d_P_Ray_real);
    cudaFree(d_P_Ray_imag);
    cudaFree(d_P_Ray_F_real);
    cudaFree(d_P_Ray_F_imag);

    if (debugFlag)
    {
      stop = std::chrono::high_resolution_clock::now();
      duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
      printf("Sonar Ray Summation %lld/100 [s]\n",
            static_cast<long long int>(duration.count() / 10000));
      start = std::chrono::high_resolution_clock::now();
    }

    // -------------- Beam culling correction -----------------//
    // beamCorrector and beamCorrectorSum is precalculated at parent cpp
    float *P_Beams_Cor_real, *P_Beams_Cor_imag;
    // float *P_Beams_Cor_F_real, *P_Beams_Cor_F_imag;
    float *P_Beams_Cor_real_tmp, *P_Beams_Cor_imag_tmp;
    float *d_P_Beams_Cor_real, *d_P_Beams_Cor_imag;
    float *d_P_Beams_Cor_F_real, *d_P_Beams_Cor_F_imag;
    const int P_Beams_Cor_N = nBeams * nFreq;
    const int P_Beams_Cor_Bytes = sizeof(float) * P_Beams_Cor_N;
    cudaMallocHost((void **)&P_Beams_Cor_real, P_Beams_Cor_Bytes);
    cudaMallocHost((void **)&P_Beams_Cor_imag, P_Beams_Cor_Bytes);
    cudaMallocHost((void **)&P_Beams_Cor_real_tmp, P_Beams_Cor_Bytes);
    cudaMallocHost((void **)&P_Beams_Cor_imag_tmp, P_Beams_Cor_Bytes);
    // cudaMallocHost((void **)&P_Beams_Cor_F_real, P_Beams_Cor_Bytes);
    // cudaMallocHost((void **)&P_Beams_Cor_F_imag, P_Beams_Cor_Bytes);
    SAFE_CALL(cudaMalloc((void **)&d_P_Beams_Cor_real, P_Beams_Cor_Bytes), "CUDA Malloc Failed");
    SAFE_CALL(cudaMalloc((void **)&d_P_Beams_Cor_imag, P_Beams_Cor_Bytes), "CUDA Malloc Failed");
    SAFE_CALL(cudaMalloc((void **)&d_P_Beams_Cor_F_real, P_Beams_Cor_Bytes), "CUDA Malloc Failed");
    SAFE_CALL(cudaMalloc((void **)&d_P_Beams_Cor_F_imag, P_Beams_Cor_Bytes), "CUDA Malloc Failed");

    float *beamCorrector_lin, *d_beamCorrector_lin;
    const int beamCorrector_lin_N = nBeams * nBeams;
    const int beamCorrector_lin_Bytes = sizeof(float) * beamCorrector_lin_N;
    cudaMallocHost((void **)&beamCorrector_lin, beamCorrector_lin_Bytes);
    SAFE_CALL(cudaMalloc((void **)&d_beamCorrector_lin, beamCorrector_lin_Bytes), "CUDA Malloc Failed");

    // (nfreq x nBeams) * (nBeams x nBeams) = (nfreq x nBeams)
    for (size_t beam = 0; beam < nBeams; beam ++)
    {
      for (size_t f = 0; f < nFreq; f++)
      {
        P_Beams_Cor_real[f * nBeams + beam] = P_Beams_F[beam][f].real() * 1.0f;
        P_Beams_Cor_imag[f * nBeams + beam] = P_Beams_F[beam][f].imag() * 1.0f;
      }
      for (size_t beam_other = 0; beam_other < nBeams; beam_other ++)
        beamCorrector_lin[beam_other * nBeams + beam] = beamCorrector[beam][beam_other];
    }

    SAFE_CALL(cudaMemcpy(d_P_Beams_Cor_real, P_Beams_Cor_real, P_Beams_Cor_Bytes,
                         cudaMemcpyHostToDevice),
              "CUDA Memcpy Failed");
    SAFE_CALL(cudaMemcpy(d_P_Beams_Cor_imag, P_Beams_Cor_imag, P_Beams_Cor_Bytes,
                         cudaMemcpyHostToDevice),
              "CUDA Memcpy Failed");
    SAFE_CALL(cudaMemcpy(d_beamCorrector_lin, beamCorrector_lin, beamCorrector_lin_Bytes,
                         cudaMemcpyHostToDevice),
              "CUDA Memcpy Failed");

    grid_rows = (nFreq + BLOCK_SIZE - 1) / BLOCK_SIZE;
    grid_cols = (nBeams + BLOCK_SIZE - 1) / BLOCK_SIZE;
    dim3 dimGrid_Beam(grid_cols, grid_rows);

    gpu_matrix_mult<<<dimGrid_Beam, dimBlock>>>(d_P_Beams_Cor_real, d_beamCorrector_lin,
                                                d_P_Beams_Cor_F_real, nFreq, nBeams, nBeams);
    SAFE_CALL(cudaDeviceSynchronize(), "Kernel Launch Failed");

    gpu_matrix_mult<<<dimGrid_Beam, dimBlock>>>(d_P_Beams_Cor_imag, d_beamCorrector_lin,
                                                d_P_Beams_Cor_F_imag, nFreq, nBeams, nBeams);
    SAFE_CALL(cudaDeviceSynchronize(), "Kernel Launch Failed");

    //Copy back data from destination device meory
    SAFE_CALL(cudaMemcpy(P_Beams_Cor_real_tmp, d_P_Beams_Cor_F_real, P_Beams_Cor_Bytes,
                         cudaMemcpyDeviceToHost),
              "CUDA Memcpy Failed");
    SAFE_CALL(cudaMemcpy(P_Beams_Cor_imag_tmp, d_P_Beams_Cor_F_imag, P_Beams_Cor_Bytes,
                         cudaMemcpyDeviceToHost),
              "CUDA Memcpy Failed");
    SAFE_CALL(cudaDeviceSynchronize(), "Kernel Launch Failed");

    // Return
    for (size_t beam = 0; beam < nBeams; beam ++)
      for (size_t f = 0; f < nFreq; f++)
        P_Beams_F[beam][f] =
            Complex(P_Beams_Cor_real_tmp[f * nBeams + beam] / beamCorrectorSum,
                    P_Beams_Cor_imag_tmp[f * nBeams + beam] / beamCorrectorSum);

    // Free memory
    cudaFree(d_P_Beams_Cor_imag);
    cudaFree(d_P_Beams_Cor_real);
    cudaFree(d_P_Beams_Cor_F_imag);
    cudaFree(d_P_Beams_Cor_F_real);
    cudaFree(d_beamCorrector_lin);
    cudaFreeHost(P_Beams_Cor_real);
    cudaFreeHost(P_Beams_Cor_imag);
    cudaFreeHost(P_Beams_Cor_real_tmp);
    cudaFreeHost(P_Beams_Cor_imag_tmp);
    cudaFreeHost(beamCorrector_lin);

    // For calc time measure
    if (debugFlag)
    {
      stop = std::chrono::high_resolution_clock::now();
      duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
      printf("GPU Window & Correction %lld/100 [s]\n",
            static_cast<long long int>(duration.count() / 10000));
      start = std::chrono::high_resolution_clock::now();
    }

    //#################################################//
    //###################   FFT   #####################//
    //#################################################//
    SAFE_CALL(cudaDeviceSynchronize(), "Kernel Launch Failed");
    const int DATASIZE = nFreq;
    const int BATCH = nBeams;
    // --- Host side input data allocation and initialization
    cufftComplex *hostInputData = (cufftComplex *)malloc(
        DATASIZE * BATCH * sizeof(cufftComplex));
    for (int beam = 0; beam < BATCH; beam++)
    {
      for (int f = 0; f < DATASIZE; f++)
      {
        if (f < nFreq)
          hostInputData[beam * DATASIZE + f] =
              make_cuComplex(P_Beams_F[beam][f].real() * 1.0f,
                             P_Beams_F[beam][f].imag() * 1.0f);
        else
          hostInputData[beam * DATASIZE + f] =
              (make_cuComplex(0.f, 0.f)); // zero padding
      }
    }

    // --- Device side input data allocation and initialization
    cufftComplex *deviceInputData;
    SAFE_CALL(cudaMalloc((void **)&deviceInputData,
                         DATASIZE * BATCH * sizeof(cufftComplex)),
                         "FFT CUDA Malloc Failed");
    SAFE_CALL(cudaMemcpy(deviceInputData, hostInputData,
                         DATASIZE * BATCH * sizeof(cufftComplex),
                         cudaMemcpyHostToDevice),
                         "FFT CUDA Memcopy Failed");

    // --- Host side output data allocation
    cufftComplex *hostOutputData =
        (cufftComplex *)malloc(DATASIZE * BATCH * sizeof(cufftComplex));

    // --- Device side output data allocation
    cufftComplex *deviceOutputData;
    cudaMalloc((void **)&deviceOutputData,
               DATASIZE * BATCH * sizeof(cufftComplex));

    // --- Batched 1D FFTs
    cufftHandle handle;
    int rank = 1;         // --- 1D FFTs
    int n[] = {DATASIZE}; // --- Size of the Fourier transform
    // --- Distance between two successive input/output elements
    int istride = 1, ostride = 1;
    int idist = DATASIZE, odist = DATASIZE; // --- Distance between batches
    // --- Input/Output size with pitch (ignored for 1D transforms)
    int inembed[] = {0};
    int onembed[] = {0};
    int batch = BATCH; // --- Number of batched executions
    cufftPlanMany(&handle, rank, n,
                  inembed, istride, idist,
                  onembed, ostride, odist, CUFFT_C2C, batch);

    cufftExecC2C(handle, deviceInputData, deviceOutputData, CUFFT_FORWARD);

    // --- Device->Host copy of the results
    SAFE_CALL(cudaMemcpy(hostOutputData, deviceOutputData,
                         DATASIZE * BATCH * sizeof(cufftComplex),
                         cudaMemcpyDeviceToHost),
                         "FFT CUDA Memcopy Failed");

    cufftDestroy(handle);
    cudaFree(deviceOutputData);
    cudaFree(deviceInputData);
    free(hostInputData);
    free(hostOutputData);


    for (int beam = 0; beam < BATCH; beam++)
    {
      for (int f = 0; f < nFreq; f++)
      {
        P_Beams_F[beam][f] =
            Complex(hostOutputData[beam * DATASIZE + f].x * delta_f,
                    hostOutputData[beam * DATASIZE + f].y * delta_f);
      }
    }

    // For calc time measure
    if (debugFlag)
    {
      stop = std::chrono::high_resolution_clock::now();
      duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
      printf("GPU FFT Calc Time %lld/100 [s]\n",
            static_cast<long long int>(duration.count() / 10000));
    }

    return P_Beams_F;
  }
} // namespace NpsGazeboSonar
