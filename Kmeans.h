# pragma once
#include "base.h"
std::vector<vector<int> > Kmeans_cluster(pcl::PointCloud<PointXY>::Ptr db,int class_);//原始数据、聚类数量
//void Kmeans_cluster(int class,pcl::PointCloud<PointXY>::Ptr db);