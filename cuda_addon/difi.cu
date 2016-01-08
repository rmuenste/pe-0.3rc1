#include <stdio.h>
#include <common.h>
#include <perftimer.h>
#include <cfloat>
#include <difi.cuh>
#include <aabb3.h>
#include <boundingvolumetree3.h>

#define cudaCheckErrors(msg) cudaCheckError(msg,__FILE__, __LINE__)

void cudaCheckError(const char *message, const char *file, const int line)
{
  cudaError err = cudaGetLastError();
  if (cudaSuccess != err)
  {
    fprintf(stderr, "cudaCheckError() failed at %s:%i : %s User error message: %s\n",
        file, line, cudaGetErrorString(err), message);
    exit(-1);
  }

}

int g_triangles;
int g_verticesGrid;

const int NN = 100 * 1000;

Model3D *g_model;

triangle *d_triangles;
vector3 *d_vertices;
vector3 *d_vertices_grid;

AABB3f *d_boundingBoxes;

vector3 *d_vertexCoords;
vector3 *d_normals;
vector3 *d_contactPoints;      

float *d_distance_map;

int *d_inout;
real *d_distance;

#include "bvh.cuh"
#include "intersection.cuh"
#include "distance.cuh"
#include "unit_tests.cuh"


#include "distancemap.cuh"
#include "uniformgrid.cuh"

BVHNode<float> *d_nodes;
DMap *d_map;
DistanceMap<float,gpu> *d_map_gpu;
UniformGrid<float,ElementCell,VertexTraits<float>,gpu> *d_unigrid_gpu;


__device__ float machine_eps_flt() {
  typedef union {
    int i32;
    float f32;
  } flt_32;

  flt_32 s;

  s.f32 = 1.;
  s.i32++;
  return (s.f32 - 1.);
}

__global__ void test_eps()
{
  float eps = machine_eps_flt();
  printf("CUDA = %g, CPU = %g\n", eps, FLT_EPSILON);
}

__global__ void d_test_points(vector3 *vertices_grid, triangle *triangles, vector3 *vertices, int *traits)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;


  //if( idx < d_nVertices_grid)
#ifdef TESTING
  if (idx == DEBUG_IVT)
#else
    if( idx < d_nVertices_grid)
#endif
    {
      //vector3 dir(0.9f,0.1f,0.2f);
      vector3 &query = vertices_grid[idx];
      vector3 dir(1.0f, 0.0f, 0.0f);
      int maxComp;
      if (fabs(query.x) > fabs(query.y))
        maxComp = 0;
      else
        maxComp = 1;

      if (fabs(query.m_dCoords[maxComp]) < fabs(query.z))
        maxComp = 2;

      if (query.m_dCoords[maxComp] < 0.0f)
      {
        dir = vector3(0.f, 0.f, 0.f);
        dir.m_dCoords[maxComp] = -1.0f;
      }
      else
      {
        dir = vector3(0.f, 0.f, 0.f);
        dir.m_dCoords[maxComp] = 1.0f;
      }

      //dir.normalize();
      int nIntersections = 0;
      int nTriangles = d_nTriangles;
      for(int i = 0; i < nTriangles; i++)
      {
        vector3 &v0 = vertices[triangles[i].idx0];
        vector3 &v1 = vertices[triangles[i].idx1];
        vector3 &v2 = vertices[triangles[i].idx2];
        if (intersection_tri(query, dir, v0, v1, v2,i))
        {
          nIntersections++;
#ifdef TESTING
          printf("Point [%f,%f,%f] hit with triangle %i \n",query.x,query.y,query.z,i);
#endif
        }
#ifdef TESTING
        else if (i == DEBUG_IDX)
        {
          printf("Point [%f,%f,%f] no hit with triangle %i \n", query.x, query.y, query.z, i);
        }

#endif
      }
      if(nIntersections%2!=0)
        traits[idx] = 1;
    }
}

void all_points_test(UnstructuredGrid<Real, DTraits> &grid)
{

  int *intersect = new int[grid.nvt_];

  cudaMemset(d_inout, 0, grid.nvt_ * sizeof(int));
  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  cudaEventRecord(start, 0);
  d_test_points<<< (grid.nvt_ + 1023)/1024 , 1024 >>> (d_vertices_grid, d_triangles, d_vertices, d_inout);
  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  float elapsed_time;
  cudaEventElapsedTime(&elapsed_time, start, stop);
  cudaDeviceSynchronize();
  printf("GPU time event: %3.8f [ms]\n", elapsed_time);
  cudaMemcpy(intersect, d_inout, sizeof(int) * grid.nvt_, cudaMemcpyDeviceToHost);
  int id = 0;
  for(id=0; id < grid.nvt_; id++)
  {
    if (intersect[id])
    {
      grid.m_myTraits[id].iTag = 1;
    }
    else
    {
      grid.m_myTraits[id].iTag = 0;
    }
  }
  delete[] intersect;
}

__global__ void d_points_dist(vector3 *vertices_grid, triangle *triangles, vector3 *vertices, AABB3f *boxes, real *distance)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if (idx < d_nVertices_grid)
  {
    vector3 &query = vertices_grid[idx];
    int nTriangles = d_nTriangles;
    real min_dist = FLT_MAX;
    for (int i = 0; i < nTriangles; i++)
    {
      vector3 &v0 = vertices[triangles[i].idx0];
      vector3 &v1 = vertices[triangles[i].idx1];
      vector3 &v2 = vertices[triangles[i].idx2];
      if (boxes[i].minDistanceSqr(query) < min_dist)
      {
        real dist = distance_triangle<real>(query, v0, v1, v2);
        if (dist < min_dist)
        {
          min_dist = dist;
        }
      }
    }
    distance[idx] = min_dist;
  }
}

__global__ void hello_kernel()
{
  printf("Hello\n");
}

void hello_launcher()
{
  hello_kernel<<<1,1>>>();
  cudaDeviceSynchronize();
}

void all_points_dist(UnstructuredGrid<Real, DTraits> &grid)
{

  hello_kernel<<<1,1>>>();
  cudaDeviceSynchronize();
  exit(0);

  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  cudaEventRecord(start, 0);
  d_points_dist << < (grid.nvt_ + 1023) / 1024, 1024 >> > (d_vertices_grid, d_triangles, d_vertices, d_boundingBoxes, d_distance);
  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  float elapsed_time;
  cudaEventElapsedTime(&elapsed_time, start, stop);
  cudaDeviceSynchronize();
  printf("GPU time distance: %3.8f [ms]\n", elapsed_time);
  real *mydist = new real[grid.nvt_];
  cudaMemcpy(mydist, d_distance, grid.nvt_ * sizeof(real), cudaMemcpyDeviceToHost);
  int id = 0;
  for (id = 0; id < grid.nvt_; id++)
  {
    if (grid.m_myTraits[id].iTag)
    {
      grid.m_myTraits[id].distance = -1.0f * sqrtf(mydist[id]);
      grid.m_myTraits[id].distance = sqrtf(mydist[id]);
    }
    else
      grid.m_myTraits[id].distance = sqrtf(mydist[id]);
  }
  hello_kernel<<<1,1>>>();
  cudaDeviceSynchronize();

}

__global__ void test_distmap(DMap *map, vector3 *vertices)
{

  //  printf("cells = %i %i %i\n",map->cells_[0],map->cells_[1],map->cells_[2]);
  //  printf("dim = %i %i \n",map->dim_[0],map->dim_[1]);
  //  

  //  printf("vertex = %f %f %f\n", map->vertices_[1].x, map->vertices_[1].y, map->vertices_[1].z);
  //
  //  printf("normals = %f %f %f\n", map->normals_[1].x, map->normals_[1].y, map->normals_[1].z);
  //
  //  printf("contactPoints = %f %f %f\n", map->contactPoints_[1].x, map->contactPoints_[1].y, map->contactPoints_[1].z);
  //
  //  printf("distance_ = %f \n", map->distance_[1]);

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if(idx < map->dim_[0]*map->dim_[1])
  {

    //    printf("center = %f %f %f\n", map->bv_.center_.x, map->bv_.center_.y, map->bv_.center_.z);
    //    printf("extends = %f %f %f\n", map->bv_.extents_[0], map->bv_.extents_[1], map->bv_.extents_[2]);
    vector3 query = vertices[idx];
    query += vector3(0.1,0,0); 
    vector3 cp(0,0,0);
    float dist=0;
    map->queryMap(query,dist,cp);

    //    if(blockIdx.x==0 && threadIdx.x==0)
    //    {
    //      map->queryMap(query,dist,cp);
    //      printf("dist_gpu = %f\n", dist);
    //      printf("dist[0] = %f\n", map->distance_[0]);
    //      printf("dist[100] = %f\n", map->distance_[100]);
    //      printf("query = %f %f %f\n", query.x, query.y, query.z);
    //      printf("cp = %f %f %f\n", cp.x, cp.y, cp.z);
    //    }
  }

  //  printf("vertex = %f %f %f\n", query.x, query.y, query.z);
  //
  //  printf("contactPoints = %f %f %f\n", cp.x, cp.y, cp.z);
  //
  //  printf("distance_ = %f \n", dist);
  //
  //  printf("mesh vertices = %i \n", d_nVertices);

}

__global__ void test_distmap(DistanceMap<float,gpu> *map)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if(idx < map->dim_[0]*map->dim_[1])
  {
    vector3 query = map->vertexCoords_[idx];
    query += vector3(0.1,0,0); 
    vector3 cp(0,0,0);
    float dist=0;
    dist=dist+1;
    map->queryMap(query,dist,cp);
  }

}

__global__ void test_dist(DistanceMap<float,gpu> *map, vector3 *vectors, float *res)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if(idx < NN)
  {
    vector3 query = vectors[idx];
    vector3 cp(0,0,0);
    float dist=0.0;
    map->queryMap(query,dist,cp);
    res[idx] = dist;
  }

}

__global__ void test_kernel(DistanceMap<float,gpu> *map, vector3 *vertices)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  //if(idx < d_nVertices)
  {
    printf("center = %f %f %f\n", map->boundingBox_.center_.x, map->boundingBox_.center_.y, map->boundingBox_.center_.z);
    printf("extends = %f %f %f\n", map->boundingBox_.extents_[0], map->boundingBox_.extents_[1], map->boundingBox_.extents_[2]);
    printf("mesh vertices =--------------------------------  \n");
    printf("map size = %i %i\n", map->dim_[0],map->dim_[1]);
    printf("vertex = %f %f %f\n", vertices[0].x, vertices[0].y, vertices[0].z);
  }

}

__global__ void test_kernel(DistanceMap<float,gpu> *map)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  //if(idx < d_nVertices)
  {
    printf("center = %f %f %f\n", map->boundingBox_.center_.x, map->boundingBox_.center_.y, map->boundingBox_.center_.z);
    printf("extends = %f %f %f\n", map->boundingBox_.extents_[0], map->boundingBox_.extents_[1], map->boundingBox_.extents_[2]);
    printf("mesh vertices =--------------------------------  \n");
    printf("map size = %i %i\n", map->dim_[0],map->dim_[1]);
    printf("vertexCoords = %f %f %f\n", map->vertexCoords_[0].x, map->vertexCoords_[0].y, map->vertexCoords_[0].z);
  }

}

inline float frand()
{
  return rand() / (float)RAND_MAX;
}

__global__ void sphere_gpu(UniformGrid<float,ElementCell,VertexTraits<float>,gpu> *g,Sphere<float> *spheres,int s)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if(idx < s)
  {
    g->queryVertex(spheres[idx]);
  }

}

__global__ void queryGrid(UniformGrid<float,ElementCell,VertexTraits<float>,gpu> *g, int j)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if(g->traits_.fbmVertices_[j])
    printf("fbm_vertex = %i %i \n", j, g->traits_.fbmVertices_[j]);

}

__global__ void copyFBM(UniformGrid<float,ElementCell,VertexTraits<float>,gpu> *g, int *d, int size)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if(idx < size)
  {
    d[idx] = g->traits_.fbmVertices_[idx];
  }

}

void sphere_test(RigidBody *body, UniformGrid<Real,ElementCell,VertexTraits<Real>> &grid)
{

  int size = body->map_->dim_[0] * body->map_->dim_[1];

  vector3 *testVectors = new vector3[NN];
  vector3 *d_testVectors;

  Real *distance_res = new Real[NN];
  float *d_distance_res;
  float *distance_gpu = new float[NN];

  std::vector<Sphere<float>> spheres;

  for(int i=0; i < NN; i++)
  {
    vector3 vr(0,0,0);
    vr.x = -body->map_->boundingBox_.extents_[0] + frand() * (2.0 * body->map_->boundingBox_.extents_[0]); 
    vr.y = -body->map_->boundingBox_.extents_[1] + frand() * (2.0 * body->map_->boundingBox_.extents_[1]); 
    vr.z = -body->map_->boundingBox_.extents_[2] + frand() * (2.0 * body->map_->boundingBox_.extents_[2]); 
    testVectors[i] = vr;
  }

  cudaMalloc((void**)&(d_testVectors), NN*sizeof(vector3));
  cudaMemcpy(d_testVectors, testVectors, NN*sizeof(vector3), cudaMemcpyHostToDevice);

  cudaMalloc((void**)&(d_distance_res), NN*sizeof(float));

  int vx = grid.traits_.cells_[0]+1;
  int vxy=vx*vx*vx;
  printf("CPU distmap for %i points\n", vxy);

  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  cudaEventRecord(start, 0);

  test_dist<<< (vxy+255)/256, 256 >>>(body->map_gpu_,d_testVectors, d_distance_res);
  cudaMemcpy(distance_gpu, d_distance_res, NN*sizeof(float), cudaMemcpyDeviceToHost);
  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  float elapsed_time;
  cudaEventElapsedTime(&elapsed_time, start, stop);
  cudaDeviceSynchronize();
  printf("Elapsed time gpu distmap: %3.8f [ms].\n", elapsed_time);
  float gpu_distmap = elapsed_time;



  CPerfTimer timer;
  timer.Start();
//  for (int i = 0; i < vxy; i++)
//  {
//    Vector3<Real> v=grid.traits_.vertexCoords_[i];
//    std::pair<Real, Vector3<Real>> res = body->map_->queryMap(v);
//    grid.traits_.fbmVertices_[i] = (res.first < 0.0) ? 1 : 0;
//  }

  
  float cpu_distmap = timer.GetTime();
  std::cout << "Elapsed time cpu distmap: " <<  cpu_distmap << " [ms]." << std::endl;

  cudaDeviceSynchronize();
  
  grid.queryVertex(body->spheres[185]);

  //for (int i = 0; i < NN; i++)
  //{
  //  printf("gpu = %3.8f cpu = %3.8f \n", distance_gpu[i], distance_res[i]);
  //}


  printf("Inner sphere test with %i spheres\n", body->spheres.size());

  timer.Start();

//  for(auto &sphere : body->spheres)
//  {
//    grid.query(sphere);
//  }

  float cpu_spheres = timer.GetTime();
  std::cout << "Elapsed time cpu spheres: " <<  cpu_spheres << " [ms]." << std::endl;

  for (int i = 0; i < vxy; i++)
  {
    grid.traits_.fbmVertices_[i] = 0;
  }

  timer.Start();

  for(auto &sphere : body->spheres)
  {
    grid.queryVertex(sphere);
  }

  float cpu_spheres_vertex = timer.GetTime();
  std::cout << "Elapsed time cpu spheres[vertex]: " <<  cpu_spheres_vertex << " [ms]." << std::endl;

  for(auto &sphere : body->spheres)
  {
    Sphere<float> s(&sphere); 
    spheres.push_back(s);
  }

  Sphere<float> *dev_spheres;
  cudaMalloc((void**)&(dev_spheres), spheres.size() * sizeof(Sphere<float>));
  cudaMemcpy(dev_spheres, spheres.data(), spheres.size()*sizeof(Sphere<float>), cudaMemcpyHostToDevice);

  cudaEvent_t start0, stop0;
  cudaEventCreate(&start0);
  cudaEventCreate(&stop0);
  cudaEventRecord(start0, 0);

  sphere_gpu<<< (spheres.size()+255)/256, 256 >>>(d_unigrid_gpu,dev_spheres,spheres.size());
  //sphere_gpu<<< 1,1  >>>(d_unigrid_gpu,dev_spheres,spheres.size());

  cudaEventRecord(stop0, 0);
  cudaEventSynchronize(stop0);
  float elapsed_time0;
  cudaEventElapsedTime(&elapsed_time0, start0, stop0);
  cudaDeviceSynchronize();
  printf("Elapsed time gpu spheres[vertex]: %3.8f [ms].\n", elapsed_time0);

//  int sphere=185;
//  int vertex=132761;
//  sphere_gpu<<< 1,1>>>(d_unigrid_gpu,dev_spheres,spheres.size());

  int size2 = (grid.m_iDimension[0]+1) * (grid.m_iDimension[1]+1) * (grid.m_iDimension[2]+1);
  int *fbmVertices = new int[size2];
  int *dev_fbmVertices;
  cudaMalloc((void**)&(dev_fbmVertices), size2 * sizeof(int));

  copyFBM<<< (size2+255)/256, 256 >>>(d_unigrid_gpu,dev_fbmVertices,size2);

  cudaMemcpy(fbmVertices, dev_fbmVertices,
             size2 * sizeof(int), cudaMemcpyDeviceToHost);
  cudaDeviceSynchronize();

  int gpuIn=0;
  int cpuIn=0;
  for (int i = 0; i < size2; i++)
  {
    if(fbmVertices[i] != grid.traits_.fbmVertices_[i])
    {
      printf("gpu = %i cpu = %i %i\n", fbmVertices[i], grid.traits_.fbmVertices_[i],i);
    }
    if(fbmVertices[i])
    {
      gpuIn++;
    }
    if(grid.traits_.fbmVertices_[i])
    {
      cpuIn++;
    }
  }

  printf("gpuIn = %i \n", gpuIn);
  printf("cpuIn = %i \n", cpuIn);

  printf("cpu_distmap = %3.8f \n", cpu_distmap);
  printf("gpu_distmap = %3.8f \n", gpu_distmap);
  printf("cpu_spheres = %3.8f \n", cpu_spheres_vertex);
  printf("gpu_distmap = %3.8f \n", elapsed_time0);

  //for (int i = 0; i < NN; i++)
  //{
  //  printf("gpu = %3.8f cpu = %3.8f \n", distance_gpu[i], distance_res[i]);
  //}


  delete[] testVectors;
  delete[] distance_res;
  delete[] distance_gpu;
  delete[] fbmVertices;

  cudaFree(d_testVectors);
  cudaFree(d_distance_res);
  cudaFree(dev_fbmVertices);

}


void dmap_test(RigidBody *body)
{

  int size = body->map_->dim_[0] * body->map_->dim_[1];

  vector3 *testVectors = new vector3[NN];
  vector3 *d_testVectors;

  Real *distance_res = new Real[NN];
  float *d_distance_res;
  float *distance_gpu = new float[NN];

  for(int i=0; i < NN; i++)
  {
    vector3 vr(0,0,0);
    vr.x = -body->map_->boundingBox_.extents_[0] + frand() * (2.0 * body->map_->boundingBox_.extents_[0]); 
    vr.y = -body->map_->boundingBox_.extents_[1] + frand() * (2.0 * body->map_->boundingBox_.extents_[1]); 
    vr.z = -body->map_->boundingBox_.extents_[2] + frand() * (2.0 * body->map_->boundingBox_.extents_[2]); 
    testVectors[i] = vr;
  }

  cudaMalloc((void**)&(d_testVectors), NN*sizeof(vector3));
  cudaMemcpy(d_testVectors, testVectors, NN*sizeof(vector3), cudaMemcpyHostToDevice);

  cudaMalloc((void**)&(d_distance_res), NN*sizeof(float));

  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  cudaEventRecord(start, 0);

  test_dist<<< (NN+255)/256, 256 >>>(body->map_gpu_,d_testVectors, d_distance_res);
  cudaMemcpy(distance_gpu, d_distance_res, NN*sizeof(float), cudaMemcpyDeviceToHost);
  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  float elapsed_time;
  cudaEventElapsedTime(&elapsed_time, start, stop);
  cudaDeviceSynchronize();
  printf("Elapsed time gpu distmap: %3.8f [ms].\n", elapsed_time);

  CPerfTimer timer;
  timer.Start();
  for (int i = 0; i < NN; i++)
  {
    Vector3<Real> v(0,0,0);
    v.x=testVectors[i].x;
    v.y=testVectors[i].y;
    v.z=testVectors[i].z;
    std::pair<Real, Vector3<Real>> res = body->map_->queryMap(v);
    distance_res[i] = res.first;
  }

  std::cout << "Elapsed time cpu distmap: " <<  timer.GetTime() << " [ms]." << std::endl;

  cudaDeviceSynchronize();

  //for (int i = 0; i < NN; i++)
  //{
  //  printf("gpu = %3.8f cpu = %3.8f \n", distance_gpu[i], distance_res[i]);
  //}

  delete[] testVectors;
  delete[] distance_res;
  delete[] distance_gpu;

  cudaFree(d_testVectors);
  cudaFree(d_distance_res);

}

__global__ void test_grid(UniformGrid<float,ElementCell,VertexTraits<float>,gpu> *g)
{

  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  {
    printf("map size gpu = %i %i\n", g->dim_[0],g->dim_[1]);
    printf("center = %f %f %f\n", g->boundingBox_.center_.x, g->boundingBox_.center_.y, g->boundingBox_.center_.z);
    printf("extends = %f %f %f\n", g->boundingBox_.extents_[0], g->boundingBox_.extents_[1], g->boundingBox_.extents_[2]);
    printf("mesh vertices =--------------------------------  \n");
    printf("vertexCoords = %f %f %f\n", g->traits_.vertexCoords_[0].x,
                                        g->traits_.vertexCoords_[0].y,
                                        g->traits_.vertexCoords_[0].z);
    printf("size gpu = %i %i %i\n", g->traits_.cells_[0],g->traits_.cells_[1],g->traits_.cells_[2]);
    int vx = g->traits_.cells_[0]+1;

    int vxy=vx*vx;

    int vxyz = vxy*vx;
    printf("size gpu = %i\n",vxyz);

    printf("vertexCoords = %f %f %f\n", g->traits_.vertexCoords_[vxyz-1].x,
                                        g->traits_.vertexCoords_[vxyz-1].y,
                                        g->traits_.vertexCoords_[vxyz-1].z);
  }

}

void transfer_uniformgrid(UniformGrid<Real,ElementCell,VertexTraits<Real>> *grid)
{

  UniformGrid<float,ElementCell,VertexTraits<float>,cpu> grid_(grid);
  printf("map size cp = %i %i\n", grid->m_iDimension[0],grid->m_iDimension[1]);

  cudaMalloc((void**)&(d_unigrid_gpu), sizeof(UniformGrid<float,ElementCell,VertexTraits<float>,gpu>));
  cudaCheckErrors("Allocate uniform grid");

  cudaMemcpy(d_unigrid_gpu, &grid_, sizeof(UniformGrid<float,ElementCell,VertexTraits<float>,cpu>), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy uniformgrid class");

  d_unigrid_gpu->transferData(grid_);

  test_grid<<<1,1>>>(d_unigrid_gpu);
  cudaDeviceSynchronize();

}

void transfer_distancemap(RigidBody *body, DistanceMap<double,cpu> *map)
{

  DistanceMap<float,cpu> map_(map);

  cudaMalloc((void**)&(body->map_gpu_), sizeof(DistanceMap<float,gpu>));
  cudaCheckErrors("Allocate dmap");

  cudaMemcpy(body->map_gpu_, &map_, sizeof(DistanceMap<float,cpu>), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy distancemap class");

  body->map_gpu_->transferData(map_);

}

void copy_distancemap(DistanceMap<double,cpu> *map)
{

  DMap map_;

  map_.dim_[0] = map->dim_[0];
  map_.dim_[1] = map->dim_[1];

  map_.cells_[0] = map->cells_[0];
  map_.cells_[1] = map->cells_[1];
  map_.cells_[2] = map->cells_[2];

  map_.cellSize_ = map->cellSize_; 

  Vector3<float> vmin, vmax;
  vmin.x = (float)map->boundingBox_.vertices_[0].x;
  vmin.y = (float)map->boundingBox_.vertices_[0].y;
  vmin.z = (float)map->boundingBox_.vertices_[0].z;

  vmax.x = (float)map->boundingBox_.vertices_[1].x;
  vmax.y = (float)map->boundingBox_.vertices_[1].y;
  vmax.z = (float)map->boundingBox_.vertices_[1].z;

  map_.bv_.init(vmin, vmax);

  cudaMalloc((void**)&d_map, sizeof(DMap));
  cudaCheckErrors("Allocate dmap");

  cudaMemcpy(d_map, &map_, sizeof(DMap), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy distancemap class");

  Vector3<float> *vertexCoords;
  Vector3<float> *normals;
  Vector3<float> *contactPoints;      

  float *distance_;

  int size = map->dim_[0] * map->dim_[1]; 

  map->outputInfo();

  vertexCoords = new Vector3<float>[size];
  normals = new Vector3<float>[size];
  contactPoints = new Vector3<float>[size];
  distance_ = new float[size];

  for (int i = 0; i < size; i++)
  {
    vertexCoords[i].x = (float)map->vertexCoords_[i].x;
    vertexCoords[i].y = (float)map->vertexCoords_[i].y;
    vertexCoords[i].z = (float)map->vertexCoords_[i].z;

    normals[i].x = (float)map->normals_[i].x;
    normals[i].y = (float)map->normals_[i].y;
    normals[i].z = (float)map->normals_[i].z;

    contactPoints[i].x = (float)map->contactPoints_[i].x;
    contactPoints[i].y = (float)map->contactPoints_[i].y;
    contactPoints[i].z = (float)map->contactPoints_[i].z;

    distance_[i] = (float)map->distance_[i];
  }

  cudaMalloc((void**)&d_vertexCoords, size * sizeof(vector3));
  cudaCheckErrors("Allocate vertices distancemap");

  cudaMemcpy(d_vertexCoords, vertexCoords, size * sizeof(vector3), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy vertices distance");

  cudaMemcpy(&d_map->vertices_ , &d_vertexCoords, sizeof(vector3*), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy vertices distance");

  cudaMalloc((void**)&d_normals, size * sizeof(vector3));
  cudaCheckErrors("Allocate vertices normals");

  cudaMemcpy(d_normals, normals, size * sizeof(vector3), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy vertices normals");

  cudaMemcpy(&d_map->normals_ , &d_normals, sizeof(vector3*), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy vertices normals");

  cudaMalloc((void**)&d_contactPoints, size * sizeof(vector3));
  cudaCheckErrors("Allocate vertices contactPoints");

  cudaMemcpy(d_contactPoints, contactPoints, size * sizeof(vector3), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy vertices contactPoints");

  cudaMemcpy(&d_map->contactPoints_ , &d_contactPoints, sizeof(vector3*), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy vertices contactPoints");

  cudaMalloc((void**)&d_distance_map, size * sizeof(float));
  cudaCheckErrors("Allocate distance");

  cudaMemcpy(d_distance_map, distance_, size * sizeof(float), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy distance");

  cudaMemcpy(&d_map->distance_ , &d_distance_map, sizeof(float*), cudaMemcpyHostToDevice);
  cudaCheckErrors("copy distance");

  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  cudaEventRecord(start, 0);
  test_distmap<<<(size+255)/256, 256 >>>(d_map, d_vertexCoords);
  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  float elapsed_time;
  cudaEventElapsedTime(&elapsed_time, start, stop);
  cudaDeviceSynchronize();
  printf("GPU distmap coll: %3.8f [ms]\n", elapsed_time);
  //test_kernel<<<1,1>>>(d_map, d_vertexCoords);

  cudaDeviceSynchronize();

  hello_kernel<<<1,1>>>();
  cudaDeviceSynchronize();
  //  std::pair<Real,Vector3<Real>> res = map->queryMap(map->vertexCoords_[0]+Vector3<Real>(0.1,0,0));
  //  std::cout << "query_cpu" << map->vertexCoords_[0] << std::endl;
  //  std::cout << "cp" << res.second << std::endl;
  //  std::cout << "dist_cpu" << res.first << std::endl;
  //  std::cout << "dist[100]" <<  map->distance_[100] << std::endl;
  //  exit(0);
  CPerfTimer timer;
  timer.Start();
  for (int i = 0; i < size; i++)
  {
    map->queryMap(map->vertexCoords_[i]);
  }

  std::cout << "Elapsed time gpu[ms]:" <<  timer.GetTime() * 1000.0 << std::endl;
  delete[] vertexCoords;
  delete[] normals;
  delete[] contactPoints;
  delete[] distance_;

}

void copy_mesh(Model3D *model){

  int nTriangles = 0;
  int nVertices = 0;

  g_model = model;

  vector3  *meshVertices;

  for (auto &mesh : model->meshes_)
  {
    nVertices += mesh.numVerts_;
    meshVertices = (vector3*)malloc(sizeof(vector3)*mesh.numVerts_);
    for (int i = 0; i < mesh.vertices_.Size(); i++)
    {
      meshVertices[i].x = (real)mesh.vertices_[i].x;
      meshVertices[i].y = (real)mesh.vertices_[i].y;
      meshVertices[i].z = (real)mesh.vertices_[i].z;
    }
  }

  printf("Number of triangles: %i\n",nVertices);
  g_triangles = nVertices;

  cudaMalloc((void**)&d_vertices, nVertices * sizeof(vector3));
  cudaCheckErrors("Allocate vertices");

  cudaMemcpy(d_vertices, meshVertices, nVertices * sizeof(vector3), cudaMemcpyHostToDevice);
  cudaCheckErrors("Copy vertices");
  cudaDeviceSynchronize();

  cudaMemcpyToSymbol(d_nVertices, &nVertices, sizeof(int));
  cudaCheckErrors("Copy number of vertices");

  free(meshVertices);

  cudaDeviceSynchronize();

}

void my_cuda_func(Model3D *model, UnstructuredGrid<Real, DTraits> &grid){

  int nTriangles = 0;
  int nVertices = 0;

  g_model = model;

  triangle *meshTriangles;
  vector3  *meshVertices;

  for (auto &mesh : model->meshes_)
  {
    nTriangles += mesh.numFaces_;
    meshTriangles=(triangle*)malloc(sizeof(triangle)*mesh.numFaces_);
    for (int i = 0; i < mesh.faces_.Size(); i++)
    {
      meshTriangles[i].idx0 = mesh.faces_[i][0];
      meshTriangles[i].idx1 = mesh.faces_[i][1];
      meshTriangles[i].idx2 = mesh.faces_[i][2];
    }

    nVertices += mesh.numVerts_;
    meshVertices = (vector3*)malloc(sizeof(vector3)*mesh.numVerts_);
    for (int i = 0; i < mesh.vertices_.Size(); i++)
    {
      meshVertices[i].x = (real)mesh.vertices_[i].x;
      meshVertices[i].y = (real)mesh.vertices_[i].y;
      meshVertices[i].z = (real)mesh.vertices_[i].z;
    }
  }

  model->meshes_[0].generateTriangleBoundingBoxes();

  AABB3f *boxes = new AABB3f[nTriangles];
  cudaMalloc((void**)&d_boundingBoxes, sizeof(AABB3f)* nTriangles);
  for (int i = 0; i < nTriangles; i++)
  {
    vector3 vmin, vmax;
    vmin.x = (real)model->meshes_[0].triangleAABBs_[i].vertices_[0].x;
    vmin.y = (real)model->meshes_[0].triangleAABBs_[i].vertices_[0].y;
    vmin.z = (real)model->meshes_[0].triangleAABBs_[i].vertices_[0].z;

    vmax.x = (real)model->meshes_[0].triangleAABBs_[i].vertices_[1].x;
    vmax.y = (real)model->meshes_[0].triangleAABBs_[i].vertices_[1].y;
    vmax.z = (real)model->meshes_[0].triangleAABBs_[i].vertices_[1].z;

    boxes[i].init(vmin, vmax);
  }

  cudaMemcpy(d_boundingBoxes, boxes, nTriangles * sizeof(AABB3f), cudaMemcpyHostToDevice);

  delete[] boxes;

  printf("Number of triangles: %i\n",nTriangles);
  g_triangles = nTriangles;
  cudaMalloc((void**)&d_triangles, nTriangles * sizeof(triangle));
  cudaCheckErrors("Allocate triangles");

  cudaMemcpy(d_triangles, meshTriangles, nTriangles * sizeof(triangle),cudaMemcpyHostToDevice);
  cudaCheckErrors("Copy triangles");

  cudaMemcpyToSymbol(d_nTriangles, &nTriangles, sizeof(int));
  cudaCheckErrors("Copy number of triangles");

  printf("CPU: Triangle[52].idx0 = %i \n", meshTriangles[52].idx0);

  cudaMalloc((void**)&d_vertices, nVertices * sizeof(vector3));
  cudaCheckErrors("Allocate vertices");

  cudaMalloc((void**)&d_inout, grid.nvt_ * sizeof(int));
  cudaCheckErrors("Allocate vertex traits");

  cudaMemcpy(d_vertices, meshVertices, nVertices * sizeof(vector3), cudaMemcpyHostToDevice);
  cudaCheckErrors("Copy vertices");
  cudaDeviceSynchronize();

  cudaMemcpyToSymbol(d_nVertices, &nVertices, sizeof(int));
  cudaCheckErrors("Copy number of vertices");

  printf("CPU: Number of vertices: %i\n", nVertices);
  printf("CPU: Vertex[52].y = %f \n", meshVertices[52].y);

  free(meshTriangles);

  free(meshVertices);

  cudaMalloc((void**)&d_vertices_grid, grid.nvt_ * sizeof(vector3));
  cudaCheckErrors("Allocate grid vertices");

  meshVertices = (vector3*)malloc(sizeof(vector3)*grid.nvt_);
  for (int i = 0; i < grid.nvt_; i++)
  {
    meshVertices[i].x = (real)grid.vertexCoords_[i].x;
    meshVertices[i].y = (real)grid.vertexCoords_[i].y;
    meshVertices[i].z = (real)grid.vertexCoords_[i].z;
  }

  cudaMemcpy(d_vertices_grid, meshVertices, grid.nvt_ * sizeof(vector3), cudaMemcpyHostToDevice);
  cudaCheckErrors("Copy grid vertices");
  cudaDeviceSynchronize();

  cudaMemcpyToSymbol(d_nVertices_grid, &grid.nvt_, sizeof(int));
  g_verticesGrid = grid.nvt_;
  free(meshVertices);

  cudaMalloc((void**)&d_distance, grid.nvt_ * sizeof(real));
  cudaCheckErrors("Allocation for distance array");

  cudaMemset(d_distance, 0, grid.nvt_ * sizeof(real));

}

__global__ void test_structure(BVHNode<float> *nodes, int *indices)
{
  printf("triangles = %i \n",nodes[1].nTriangles_);
  printf("center = %f \n", nodes[1].bv_.center_.x);

  printf("GPU: nodes[1].indices_[0] = %i \n", nodes[0].indices_[0]);
  printf("GPU: indices_[0] = %i \n", indices[0]);

}

void allocateNodes(std::list<int> *triangleIdx, AABB3f *boxes, int *pSize, int nNodes)
{

  cudaMalloc((void**)&d_nodes, nNodes * sizeof(BVHNode<float>));
  cudaCheckErrors("Allocate nodes");

  for (int i = 0; i < nNodes; i++)
  {
    cudaMemcpy(&d_nodes[i].nTriangles_, &pSize[i], sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(&(d_nodes[i].bv_), &boxes[i], sizeof(AABB3f), cudaMemcpyHostToDevice);
  }

  int **d_indices = new int*[nNodes];

  for (int i = 0; i < nNodes; i++)
  {
    cudaMalloc((void**)&d_indices[i], pSize[i] * sizeof(int));
    cudaMemcpy(&d_nodes[i].indices_, &d_indices[i], sizeof(int*), cudaMemcpyHostToDevice);
  }
  cudaDeviceSynchronize();
  printf("nodes = %i, psize[0] = %i %i \n", nNodes, pSize[0],triangleIdx[0].size());
  int indices[10000];
  int j = 0;

  for (auto &idx : triangleIdx[0])
  {
    indices[j] = idx;
    j++;
  }
  printf("CPU: nodes[1].indices_[0] = %i \n", indices[0]);

  for (int i = 0; i < nNodes; i++)
  {

    int j = 0;
    for (auto &idx : triangleIdx[i])
    {
      indices[j] = idx;
      j++;
    }
    if (i==0)
      printf("CPU: nodes[1].indices_[0] = %i \n", indices[0]);

    cudaMemcpy(d_indices[i], indices, sizeof(int) * pSize[i], cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();
  }

  test_structure << < 1, 1 >> >(d_nodes, d_indices[0]);

  printf("gpu triangles = %i \n", pSize[1]);
  printf("center = %f \n", boxes[1].center_.x);

}

void cleanGPU()
{

  cudaFree(d_triangles);
  cudaFree(d_vertices);
  cudaFree(d_vertices_grid);
  cudaFree(d_inout);
  cudaFree(d_distance);
  cudaFree(d_nodes);

}
