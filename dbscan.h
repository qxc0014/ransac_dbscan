# pragma once
#include "base.h"
#include "knn.h"
class dbscan{
    public:
        double eps_;//设置radius搜索的半径
        int min_sample_;//核心点所需的最小采样
        Octant* root_;
        vector<int>* cluster_state;
        int cluster_id = 0;
        pcl::PointCloud<PointXYZ>::Ptr db_;
        dbscan(pcl::PointCloud<PointXYZ>::Ptr db,Octant* root,double eps,int min_sample);//初始化dbscan参数
        void run();//运行dbscan聚类
        void explore(int index,int cluster_idx);
        bool isCore(int index);
};