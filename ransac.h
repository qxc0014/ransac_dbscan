# pragma once
#include "base.h"
class Platform{
    public:
    Platform(){}
    Eigen::Vector3d calnormal(){
        return Eigen::Vector3d(a,b,c);
    }
    double a;
    double b;
    double c;
    double d;
    int num;//在范围内的点数量
};
std::pair<Eigen::Vector3d,PointXYZ> ransac(pcl::PointCloud<PointXYZ>::Ptr db,int max_iter);