#include<iostream>
#include<vector>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<pcl-1.9/pcl/io/pcd_io.h>
#include<pcl-1.9/pcl/filters/voxel_grid.h>
#include<pcl-1.9/pcl/point_types.h>
#include<pcl-1.9/pcl/visualization/cloud_viewer.h>
#include<pcl-1.9/pcl/kdtree/kdtree.h>
#include<pcl-1.9/pcl/common/transforms.h>
#include<pcl-1.9/pcl/point_cloud.h>
#include<pcl-1.9/pcl/octree/octree.h>
#include<chrono>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include<boost/format.hpp>
using namespace std;
using namespace pcl;