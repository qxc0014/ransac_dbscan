#include "dbscan.h"
dbscan::dbscan(pcl::PointCloud<PointXYZ>::Ptr db,Octant* root,double eps,int min_sample):db_(db),root_(root),eps_(eps),min_sample_(min_sample){
    cluster_state = new vector<int>(db_->size(),-1);//-1表示未访问
}
void dbscan::run(){
    //随机选取未访问的点作为初始点
    //srand(time(0));
    //int index = rand()%cluster_state->size();
    //以初始点起始，半径为eps继续搜索
    //result a(eps_);
    //octree_radius_search(root_,db_,(*db_)[index],a);
    for(int i=0;i<db_->size();i++){
        if((*cluster_state)[i] != -1) continue;
        if(isCore(i)){//如果圆内数量大于min_sample
            explore(i,++cluster_id);
        }else{
            (*cluster_state)[i] = 0;//噪点
        }
    }
}
void dbscan::explore(int index,int cluster_idx){
    (*cluster_state)[index] = cluster_idx;//核心点
    result a(eps_);
    octree_radius_search(root_,db_,(*db_)[index],a);
    if(a.only_index.size() <= min_sample_) return;
    for(auto &idx:a.only_index){
        if((*cluster_state)[idx] != -1) continue;//已访问过的点跳出
        explore(idx,cluster_idx);
    }

}
bool dbscan::isCore(int index){
    result a(eps_);
    octree_radius_search(root_,db_,(*db_)[index],a);
    if(a.only_index.size()>=min_sample_){
        return true;
    }else{
        return false;
    }
}