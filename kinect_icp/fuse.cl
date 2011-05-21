__kernel void fuse(__global float *Volume, __global float *Image, int N) 
{ 
    // Get the index of the current element
    int ix = get_global_id(0);
    int iy = get_global_id(1);
    
    for(int iz = 0; iz < N; ++iz)
    {
      float depth = Image[4*(ix + iy*N) + 2];
      float voxelDepth = (float)iz/(float)N;
      float dist = max(-0.125,min(0.125, voxelDepth-depth));
      int index = 2*(ix + iy*N + iz*N*N);
      Volume[index] = dist;
      Volume[index+1] = 1.0;
    }

}
