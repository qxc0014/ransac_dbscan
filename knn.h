# pragma once
#include "base.h"

class Octant{
    public:
        Octant(){}
        Octant(Octant* child_root,PointXYZ center,float extent,vector<int> points_index,bool is_leaf,int depth)
        :extent_(extent),points_index_(points_index),is_leaf_(is_leaf),center_(center),depth_(depth){
           // for(int i=0;i<8;i++)
           //    child_root_[i] = new Octant();
        }
        int depth_=0;//节点的深度
        Octant* child_root_[8]={nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr};//存八个子立方体的指针
        PointXYZ center_;//当前立方体的中心坐标
        float extent_;//当前立方体的半边长
        vector<int> points_index_;//当前立方体的包含点的Index
        bool is_leaf_;//当前坐标是否为叶子
        
};
class distindex{
    public:
        distindex(float dist_,int index_):dist(dist_),index(index_){}
        float dist;
        int index;
};
class result{
    public:
    result(float worst_dis_):worst_dis(worst_dis_){only_index.resize(1);}//用于搜索一个近邻点
    result(float worst_dis_,int k):worst_dis(worst_dis_),worst_dis_cap(vector<distindex>(k,distindex(worst_dis_,-1))),size(k){ 
    }
    float worst_dis=0;
    int index;
    int num=0;
    int size;
    vector<distindex> worst_dis_cap;
    vector<int> only_index;
    void add_point(float bias,int node_index);
    void add_point_radius(const float &bias,const int &node_index);
    std::set<int> get_set(){
        std::set<int> index_set;
        for(int w=0;w<num;w++){
            index_set.insert(worst_dis_cap[w].index);
        }
        return index_set;
    }
};
Octant* build_octree(Octant* root,pcl::PointCloud<PointXYZ>::Ptr db,PointXYZ center,float extent,vector<int> points_index,int depth,int width);
bool octree_knn_search(Octant* root,pcl::PointCloud<PointXYZ>::Ptr db,PointXYZ Point,result &a);
bool octree_radius_search(Octant* root,pcl::PointCloud<PointXYZ>::Ptr db,const PointXYZ &Point,result &a);