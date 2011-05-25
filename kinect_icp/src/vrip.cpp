#include "kinect_icp/vrip.h"
#include <stdbool.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>

using namespace kinect_icp;

#define MAX_SOURCE_SIZE (0x100000)
#define BLOCK_SIZE = 1024

//#define TEST_LEAKS

#define CHECK(value) if (value != 0) {std::cerr << "An Error(" << value << ") occurred at " << __FILE__ << ":" << __LINE__ << std::endl; exit(1);}

#define SEP printf("-----------------------------------------------------------\n")

int device_stats(cl_device_id device_id)
{

  int err;
  size_t returned_size;

  // Report the device vendor and device name
  //
  cl_char vendor_name[1024] = {0};
  cl_char device_name[1024] = {0};
  cl_char device_profile[1024] = {0};
  cl_char device_extensions[1024] = {0};
  cl_device_local_mem_type local_mem_type;

  cl_ulong global_mem_size, global_mem_cache_size;
  cl_ulong max_mem_alloc_size;

  cl_uint clock_frequency, vector_width, max_compute_units;

  size_t max_work_item_dims, max_work_group_size, max_work_item_sizes[3];

  cl_uint vector_types[] = {CL_DEVICE_PREFERRED_VECTOR_WIDTH_CHAR, CL_DEVICE_PREFERRED_VECTOR_WIDTH_SHORT, CL_DEVICE_PREFERRED_VECTOR_WIDTH_INT, CL_DEVICE_PREFERRED_VECTOR_WIDTH_LONG, CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT, CL_DEVICE_PREFERRED_VECTOR_WIDTH_DOUBLE};
  const char *vector_type_names[] = {"char", "short", "int", "long", "float", "double"};

  err = clGetDeviceInfo(device_id, CL_DEVICE_VENDOR, sizeof (vendor_name), vendor_name, &returned_size);
  err |= clGetDeviceInfo(device_id, CL_DEVICE_NAME, sizeof (device_name), device_name, &returned_size);
  err |= clGetDeviceInfo(device_id, CL_DEVICE_PROFILE, sizeof (device_profile), device_profile, &returned_size);
  err |= clGetDeviceInfo(device_id, CL_DEVICE_EXTENSIONS, sizeof (device_extensions), device_extensions, &returned_size);
  err |= clGetDeviceInfo(device_id, CL_DEVICE_LOCAL_MEM_TYPE, sizeof (local_mem_type), &local_mem_type, &returned_size);

  err |= clGetDeviceInfo(device_id, CL_DEVICE_GLOBAL_MEM_SIZE, sizeof (global_mem_size), &global_mem_size, &returned_size);
  err |= clGetDeviceInfo(device_id, CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE, sizeof (global_mem_cache_size), &global_mem_cache_size, &returned_size);
  err |= clGetDeviceInfo(device_id, CL_DEVICE_MAX_MEM_ALLOC_SIZE, sizeof (max_mem_alloc_size), &max_mem_alloc_size, &returned_size);

  err |= clGetDeviceInfo(device_id, CL_DEVICE_MAX_CLOCK_FREQUENCY, sizeof (clock_frequency), &clock_frequency, &returned_size);

  err |= clGetDeviceInfo(device_id, CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof (max_work_group_size), &max_work_group_size, &returned_size);

  err |= clGetDeviceInfo(device_id, CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS, sizeof (max_work_item_dims), &max_work_item_dims, &returned_size);

  err |= clGetDeviceInfo(device_id, CL_DEVICE_MAX_WORK_ITEM_SIZES, sizeof (max_work_item_sizes), max_work_item_sizes, &returned_size);

  err |= clGetDeviceInfo(device_id, CL_DEVICE_MAX_COMPUTE_UNITS, sizeof (max_compute_units), &max_compute_units, &returned_size);

  printf("Vendor: %s\n", vendor_name);
  printf("Device Name: %s\n", device_name);
  printf("Profile: %s\n", device_profile);
  printf("Supported Extensions: %s\n\n", device_extensions);

  printf("Local Mem Type (Local=1, Global=2): %i\n", (int) local_mem_type);
  printf("Global Mem Size (MB): %i\n", (int) global_mem_size / (1024 * 1024));
  printf("Global Mem Cache Size (Bytes): %i\n", (int) global_mem_cache_size);
  printf("Max Mem Alloc Size (MB): %ld\n", (long int) max_mem_alloc_size / (1024 * 1024));

  printf("Clock Frequency (MHz): %i\n\n", clock_frequency);

  for (int i = 0; i < 6; i++)
  {
    err |= clGetDeviceInfo(device_id, vector_types[i], sizeof (clock_frequency), &vector_width, &returned_size);
    printf("Vector type width for: %s = %i\n", vector_type_names[i], vector_width);
  }

  printf("\nMax Work Group Size: %lu\n", (unsigned long) max_work_group_size);
  //printf("Max Work Item Dims: %lu\n",max_work_item_dims);
  //for(size_t i=0;i<max_work_item_dims;i++)
  //	printf("Max Work Items in Dim %lu: %lu\n",(long unsigned)(i+1),(long unsigned)max_work_item_sizes[i]);

  printf("Max Compute Units: %i\n", max_compute_units);
  printf("\n");

  return CL_SUCCESS;
}

const int Volume_Size = 16;
const float d_max = 0.05;
const float d_min = -d_max;
const int blockSize = 1024;

Vrip::Vrip()
  : imageSize_(Volume_Size*Volume_Size * 4)
  , volumeSize_(Volume_Size*Volume_Size*Volume_Size)
{
  // Get platform and device information
  cl_platform_id platform_id = NULL;
  cl_uint ret_num_devices;
  cl_uint ret_num_platforms;
  CHECK(clGetPlatformIDs(1, &platform_id, &ret_num_platforms));
  CHECK(clGetDeviceIDs(platform_id, CL_DEVICE_TYPE_ALL, 1,
    &device_id_, &ret_num_devices));

  device_stats(device_id_);

  cl_int ret;

  // Create an OpenCL context
  context_ = clCreateContext(NULL, 1, &device_id_, NULL, NULL, &ret);
  CHECK(ret);

  // Create a command queue
  command_queue_ = clCreateCommandQueue(context_, device_id_, 0, &ret);
  CHECK(ret);

  // Create memory buffers on the device for each vector
  volume_mem_obj_ = clCreateBuffer(context_, CL_MEM_READ_WRITE,
    volumeSize_ * 2 * sizeof (float), NULL, &ret);

  CHECK(ret);

  march_mem_obj_ = clCreateBuffer(context_, CL_MEM_READ_WRITE,
    volumeSize_ * sizeof (int), NULL, &ret);

  CHECK(ret);

  image_mem_obj_ = clCreateBuffer(context_, CL_MEM_READ_ONLY,
    imageSize_ * sizeof (float), NULL, &ret);

  CHECK(ret);

  float* volume = (float*) malloc(volumeSize_ * 2 * sizeof (float));
  int i = 0;
  while (i < volumeSize_ * 2)
  {
    volume[i++] = d_max;
    volume[i++] = 0;
  }

  // Copy the lists A and B to their respective memory buffers
  CHECK(clEnqueueWriteBuffer(command_queue_, volume_mem_obj_, CL_TRUE, 0,
    volumeSize_ * 2 * sizeof (float), volume, 0, NULL, NULL));

  free(volume);

  cl_kernel * kernels[3] = {&fuse_kernel_, &preMarching_, &mainMarching_};
  const char* names[3] = {"fuse", "precube", "cube"};
  loadKernel("fuse.cl", 3, kernels, names);

  cl_kernel * kernels2[3] = {&scanLargeArrays_, &blockAddition_, &prefixSum_};
  const char* names2[3] = {"ScanLargeArrays", "blockAddition", "prefixSum"};
  loadKernel("ScanLargeArrays_Kernels.cl", 3, kernels2, names2);

#ifdef TEST_LEAKS
  testScan();
  Cleanup();
  exit(1);
#endif
}

void Vrip::loadKernel(const char* filename, int num_kernels, cl_kernel* kernels[],
  const char* kernel_names[])
{
  // Load the kernel source code into the array source_str
  FILE *fp;
  char *source_str = (char*) malloc(MAX_SOURCE_SIZE);
  size_t source_size;

  fp = fopen(filename, "r");
  CHECK(!fp);

  source_size = fread(source_str, 1, MAX_SOURCE_SIZE, fp);
  fclose(fp);

  // Create a program from the kernel source
  cl_int ret;
  cl_program program;
  program = clCreateProgramWithSource(context_, 1,
    (const char **) &source_str, &source_size, &ret);
  CHECK(ret);

  // Build the program
  CHECK(clBuildProgram(program, 1, &device_id_, NULL, NULL, NULL));

  char buffer[2048];
  size_t len;
  clGetProgramBuildInfo(program, device_id_, CL_PROGRAM_BUILD_LOG, sizeof (buffer), buffer, &len);

  std::cout << buffer << std::endl;

  for (int i = 0; i < num_kernels; i++)
  {
    // Create the OpenCL kernel
    *kernels[i] = clCreateKernel(program, kernel_names[i], &ret);
    printf("Loading kernel %s...\n", kernel_names[i]);
    CHECK(ret);
  }

  // Release program
  CHECK(clReleaseProgram(program));

  free(source_str);
}

void Vrip::Cleanup()
{
  // Clean up
  CHECK(clFlush(command_queue_));
  CHECK(clFinish(command_queue_));

  CHECK(clReleaseKernel(fuse_kernel_));
  CHECK(clReleaseKernel(preMarching_));
  CHECK(clReleaseKernel(mainMarching_));
  CHECK(clReleaseKernel(scanLargeArrays_));
  CHECK(clReleaseKernel(blockAddition_));
  CHECK(clReleaseKernel(prefixSum_));

  CHECK(clReleaseMemObject(volume_mem_obj_));
  CHECK(clReleaseMemObject(march_mem_obj_));
  CHECK(clReleaseMemObject(image_mem_obj_));
  CHECK(clReleaseCommandQueue(command_queue_));
  CHECK(clReleaseContext(context_));
}

Vrip::~Vrip()
{
  Cleanup();
}

void Vrip::fuseCloud(const PCloud::ConstPtr& new_point_cloud)
{
  float* image = (float*) malloc(imageSize_ * sizeof (float));
  int index = 0;
  for (int i = 0; i < Volume_Size; ++i)
  {
    for (int j = 0; j < Volume_Size; ++j)
    {
      float ii = (float) i / (float) Volume_Size;
      float jj = (float) j / (float) Volume_Size;
      image[index++] = ii;
      image[index++] = jj;
      image[index++] = (ii - 0.5f)*(ii - 0.5f) + (jj - 0.5f)*(jj - 0.5f) + 0.25;

      //RBG actually not used but will be in the data
      image[index++] = 0;
    }
  }

  CHECK(clEnqueueWriteBuffer(command_queue_, image_mem_obj_, CL_TRUE, 0,
    imageSize_ * sizeof (float), image, 0, NULL, NULL));

  clFinish(command_queue_);

  free(image);

  // Set the arguments of the kernel
  CHECK(clSetKernelArg(fuse_kernel_, 0, sizeof (cl_mem), (void *) &volume_mem_obj_));
  CHECK(clSetKernelArg(fuse_kernel_, 1, sizeof (cl_mem), (void *) &image_mem_obj_));
  CHECK(clSetKernelArg(fuse_kernel_, 2, sizeof (int), (void *) &Volume_Size));
  CHECK(clSetKernelArg(fuse_kernel_, 3, sizeof (float), (void *) &d_min));
  CHECK(clSetKernelArg(fuse_kernel_, 4, sizeof (float), (void *) &d_max));

  // Execute the OpenCL kernel on the list
  size_t localWorkSize[] = {16, 16};
  size_t globalWorkSize[] = {Volume_Size, Volume_Size};
  CHECK(clEnqueueNDRangeKernel(command_queue_, fuse_kernel_, 2, NULL,
    globalWorkSize, localWorkSize, 0, NULL, NULL));

  /*float* volume = (float*) malloc(volumeSize_ * 2 *sizeof (float));
  CHECK(clEnqueueReadBuffer(command_queue_, volume_mem_obj_, CL_TRUE, 0,
    volumeSize_ * 2 * sizeof (float), volume, 0, NULL, NULL);

  int i = 0;
  for(int x = 0; x < Volume_Size; ++x)
  {
    for(int y = 0; y < Volume_Size; ++y)
    {
      for(int z = 0; z < Volume_Size; ++z)
      {
        std::cout << volume[i++] << " ";
        std::cout << volume[i++] << " - ";
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }*/

  marchingCubes();
}

void Vrip::testScan()
{
  cl_int ret;
  cl_mem inputBuffer;

  const int length = 2048;
  cl_int* input;

  /* input buffer size */
  cl_uint sizeBytes = length * sizeof (cl_int);

  /* allocate memory for input arrray */
  input = (cl_int*) malloc(sizeBytes);

  /* error check */
  CHECK(!input);

  for (int i = 0; i < length; i++)
  {
    input[i] = i*2+1;
  }


  inputBuffer = clCreateBuffer(
    context_,
    CL_MEM_READ_WRITE,
    sizeof (cl_int) * length,
    0,
    &ret);
  CHECK(ret);


  /* Enqueue Input Buffer */
  CHECK(clEnqueueWriteBuffer(command_queue_,
    inputBuffer,
    1,
    0,
    length * sizeof (cl_int),
    input,
    0,
    0,
    NULL));


  /* Wait for write to finish */
  clFinish(command_queue_);

  preFixSum(inputBuffer, length);

  cl_int* output;
  /* allocate memory for output buffer */
  output = (cl_int*) malloc(sizeBytes);

  /* Read the final result */
  CHECK(clEnqueueReadBuffer(command_queue_,
    inputBuffer,
    1,
    0,
    length * sizeof (cl_int),
    output,
    0,
    0,
    NULL));

  clFinish(command_queue_);

  int sum = 0;
  for (int i = 0; i < length; i++)
  {
    int tmp = input[i];
    input[i] = sum;
    sum += tmp;
  }

  std::cout << "Output: " << std::endl;
  for (int i = 0; i < length; i++)
  {
    if (i < 10 || i > length - 10)
    {
      std::cout << "I[" << i << "] => GPU/CPU: " << output[i] << "/" << output[i] << std::endl;
    }

    if (output[i] != input[i])
    {
      std::cout << "ERROR!" << std::endl;
      exit(1);
    }
  }

  free(input);
  free(output);
  clReleaseMemObject(inputBuffer);
  std::cout << "Test PASSED!" << std::endl;
}

void Vrip::marchingCubes()
{
  std::cout << "pre Marching cubes started" << std::endl;

  // Set the arguments of the kernel
  CHECK(clSetKernelArg(preMarching_, 0, sizeof (cl_mem), (void *) &volume_mem_obj_));
  CHECK(clSetKernelArg(preMarching_, 1, sizeof (cl_mem), (void *) &march_mem_obj_));
  CHECK(clSetKernelArg(preMarching_, 2, sizeof (int), (void *) &Volume_Size));

  size_t localWorkSize3D[] = {16, 16, 1};
  size_t globalWorkSize3D[] = {Volume_Size, Volume_Size, Volume_Size};
  CHECK(clEnqueueNDRangeKernel(command_queue_, preMarching_, 3, NULL,
    globalWorkSize3D, localWorkSize3D, 0, NULL, NULL));

  std::cout << "pre Marching cubes finished" << std::endl;

  int memoryToAllocate = preFixSum(march_mem_obj_, 1024);

  std::cout << "Marching cubes started" << std::endl;

  cl_int ret;
  cl_mem out_mem_obj = clCreateBuffer(context_, CL_MEM_WRITE_ONLY,
    memoryToAllocate * sizeof (float), NULL, &ret);

  CHECK(ret);

  // Set the arguments of the kernel
  CHECK(clSetKernelArg(mainMarching_, 0, sizeof (cl_mem), (void *) &volume_mem_obj_));
  CHECK(clSetKernelArg(mainMarching_, 1, sizeof (cl_mem), (void *) &march_mem_obj_));
  CHECK(clSetKernelArg(mainMarching_, 2, sizeof (cl_mem), (void *) &out_mem_obj));
  CHECK(clSetKernelArg(mainMarching_, 3, sizeof (int), (void *) &Volume_Size));

  CHECK(clEnqueueNDRangeKernel(command_queue_, mainMarching_, 3, NULL,
    globalWorkSize3D, localWorkSize3D, 0, NULL, NULL));

  float* hostOut = (float*) malloc(memoryToAllocate * sizeof (float));

  CHECK(clEnqueueReadBuffer(command_queue_, out_mem_obj, CL_TRUE, 0,
    memoryToAllocate * sizeof (float), hostOut, 0, NULL, NULL));

  clFinish(command_queue_);

  std::cout << "Marching cubes finished" << std::endl;

  //Write Marching Cube Surface
  std::ofstream File("test.off");
  File << "OFF" << std::endl;

  memoryToAllocate /= 3; //3 indices per vertex
  File << memoryToAllocate << " " << memoryToAllocate / 3 << " " << 0 << std::endl;

  for (int i = 0; i < memoryToAllocate; ++i)
  {
    File << hostOut[3 * i] << " " << hostOut[3 * i + 1] << " " << hostOut[3 * i + 2] << std::endl;
  }

  memoryToAllocate /= 3; //3 vertices per face
  for (int i = 0; i < memoryToAllocate; ++i)
  {
    File << 3 << " " << i * 3 << " " << i * 3 + 1 << " " << i * 3 + 2 << std::endl;
  }

  CHECK(clReleaseMemObject(out_mem_obj));

  free(hostOut);
}

int Vrip::preFixSum(cl_mem inputBuffer, int input_length)
{
  cl_uint pass;
  cl_uint length = input_length;

  /* Calculate number of passes required */
  float t = log((float) length) / log((float) blockSize);
  pass = (cl_uint) t;

  // If t is equal to pass
  if (fabs(t - (float) pass) < 1e-7)
  {
    pass--;
  }

  cl_mem *outputBuffer; /**< Array of output buffers */
  cl_mem *blockSumBuffer; /**< Array of block sum buffers */
  cl_int ret;
  cl_mem tempBuffer;

  /* Allocate output buffers */
  outputBuffer = (cl_mem*) malloc(pass * sizeof (cl_mem));
  outputBuffer[0] = inputBuffer;

  for (int i = 1; i < (int) pass; i++)
  {
    int size = (int) (length / pow((float) blockSize, (float) i));
    outputBuffer[i] = clCreateBuffer(
      context_,
      CL_MEM_READ_WRITE,
      sizeof (cl_int) * size,
      0,
      &ret);

    CHECK(ret);
  }

  /* Allocate blockSumBuffers */
  blockSumBuffer = (cl_mem*) malloc(pass * sizeof (cl_mem));

  for (int i = 0; i < (int) pass; i++)
  {
    int size = (int) (length / pow((float) blockSize, (float) (i + 1)));
    blockSumBuffer[i] = clCreateBuffer(
      context_,
      CL_MEM_READ_WRITE,
      sizeof (cl_int) * size,
      0,
      &ret);

    CHECK(ret);
  }

  /* Create a tempBuffer on device */
  int tempLength = (int) (length / pow((float) blockSize, (float) pass));

  tempBuffer = clCreateBuffer(context_,
    CL_MEM_READ_WRITE,
    sizeof (cl_int) * tempLength,
    0,
    &ret);
  CHECK(ret);

  /* Do block-wise sum */
  bScan(length, &inputBuffer, &outputBuffer[0], &blockSumBuffer[0]);

  for (int i = 1; i < (int) pass; i++)
  {
    bScan((cl_uint) (length / pow((float) blockSize, (float) i)),
      &blockSumBuffer[i - 1],
      &outputBuffer[i],
      &blockSumBuffer[i]);
  }

  /* Do scan to tempBuffer */
  pScan(tempLength, &blockSumBuffer[pass - 1], &tempBuffer);

  /* Do block-addition on outputBuffers */
  bAddition((cl_uint) (length / pow((float) blockSize, (float) (pass - 1))),
    &tempBuffer, &outputBuffer[pass - 1]);

  for (int i = pass - 1; i > 0; i--)
  {
    bAddition((cl_uint) (length / pow((float) blockSize, (float) (i - 1))),
      &outputBuffer[i], &outputBuffer[i - 1]);
  }

  CHECK(clFinish(command_queue_));

  CHECK(clReleaseMemObject(tempBuffer));

  for (int i = 0; i < (int) pass; i++)
  {
    if (i != 0)
      CHECK(clReleaseMemObject(outputBuffer[i]));

    CHECK(clReleaseMemObject(blockSumBuffer[i]));
  }

  free(blockSumBuffer);
  free(outputBuffer);

  // TODO free everything!
  // TODO return correct value
  return -1;
}

void Vrip::bScan(cl_uint len,
  cl_mem *inputBuffer,
  cl_mem *outputBuffer,
  cl_mem * blockSumBuffer)
{
  /* set the block size*/
  size_t globalThreads[1] = {len / 2};
  size_t localThreads[1] = {blockSize / 2};

  /* Set appropriate arguments to the kernel */
  /* 1st argument to the kernel - outputBuffer */
  CHECK(clSetKernelArg(
    scanLargeArrays_,
    0,
    sizeof (cl_mem),
    (void *) outputBuffer));

  /* 2nd argument to the kernel - inputBuffer */
  CHECK(clSetKernelArg(
    scanLargeArrays_,
    1,
    sizeof (cl_mem),
    (void *) inputBuffer));

  /* 3rd argument to the kernel - local memory */
  CHECK(clSetKernelArg(
    scanLargeArrays_,
    2,
    blockSize * sizeof (cl_int),
    NULL));


  /* 4th argument to the kernel - blockSize  */
  CHECK(clSetKernelArg(
    scanLargeArrays_,
    3,
    sizeof (cl_int),
    &blockSize));

  /* 5th argument to the kernel - length  */
  CHECK(clSetKernelArg(
    scanLargeArrays_,
    4,
    sizeof (cl_int),
    &len));

  /* 6th argument to the kernel - sum of blocks  */
  CHECK(clSetKernelArg(
    scanLargeArrays_,
    5,
    sizeof (cl_mem),
    blockSumBuffer));

  // TODO maybe check if sufficient local memory


  /* Enqueue a kernel run call.*/
  CHECK((clEnqueueNDRangeKernel(
    command_queue_,
    scanLargeArrays_,
    1,
    NULL,
    globalThreads,
    localThreads,
    0,
    NULL,
    NULL)));
}

void Vrip::pScan(cl_uint len,
  cl_mem *inputBuffer,
  cl_mem * outputBuffer)
{
  size_t globalThreads[1] = {len / 2};
  size_t localThreads[1] = {len / 2};

  /* Set appropriate arguments to the kernel */
  /* 1st argument to the kernel - outputBuffer */
  CHECK(clSetKernelArg(
    prefixSum_,
    0,
    sizeof (cl_mem),
    (void *) outputBuffer));

  /* 2nd argument to the kernel - inputBuffer */
  CHECK(clSetKernelArg(
    prefixSum_,
    1,
    sizeof (cl_mem),
    (void *) inputBuffer));

  /* 3rd argument to the kernel - local memory */
  CHECK(clSetKernelArg(
    prefixSum_,
    2,
    len * sizeof (cl_int),
    NULL));

  /* 4th argument to the kernel - length */
  CHECK(clSetKernelArg(
    prefixSum_,
    3,
    sizeof (cl_int),
    (void*) &len));

  /* Enqueue a kernel run call.*/
  CHECK(clEnqueueNDRangeKernel(
    command_queue_,
    prefixSum_,
    1,
    NULL,
    globalThreads,
    localThreads,
    0,
    NULL,
    NULL));
}

void Vrip::bAddition(cl_uint len,
  cl_mem *inputBuffer,
  cl_mem *outputBuffer)
{
  /* set the block size*/
  size_t globalThreads[1] = {len};
  size_t localThreads[1] = {blockSize};

  /*** Set appropriate arguments to the kernel ***/
  /* 1st argument to the kernel - inputBuffer */
  CHECK(clSetKernelArg(
    blockAddition_,
    0,
    sizeof (cl_mem),
    (void*) inputBuffer));

  /* 2nd argument to the kernel - outputBuffer */
  CHECK(clSetKernelArg(
    blockAddition_,
    1,
    sizeof (cl_mem),
    (void *) outputBuffer));


  /* Enqueue a kernel run call.*/
  CHECK(clEnqueueNDRangeKernel(
    command_queue_,
    blockAddition_,
    1,
    NULL,
    globalThreads,
    localThreads,
    0,
    NULL,
    NULL));
}
