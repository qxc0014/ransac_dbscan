#include "Kmeans.h" 
//void Kmeans(pcl::PointCloud<PointXY>::Ptr db,int class_,std::vector<vector<int>> &cluster_index){
std::vector<vector<int> > Kmeans_cluster(pcl::PointCloud<PointXY>::Ptr db,int class_){
    //随机初始化点
    srand(time(0));
    std::vector<int> center_index;
    std::vector<vector<int>> cluster_index;
    cluster_index.resize(class_);
    center_index.clear();
    for(int d =0;d<class_;d++){
        center_index.push_back(rand()%(db->size()));
    }
    pcl::PointCloud<PointXY>::Ptr center(new pcl::PointCloud<PointXY>);
    for(int i = 0;i <class_;i++){
        center->push_back((*db)[center_index[i]]);
    }
    int max_iter = 300;
    //计算每个点到中心点的距离
    while(max_iter--){
        for(int i =0;i<class_;i++)
            cluster_index[i].clear();
        for(int index=0;index<db->size();index++){
            double min_dis=2e8;
            int mindis_class=-1;//默认为不属于任何聚类模型
            for(int k = 0;k<class_;k++){
                double bias_x,bias_y,dis;
                bias_x = ((*db)[index].x - (*center)[k].x);
                bias_y = ((*db)[index].y - (*center)[k].y);
                dis = bias_x*bias_x + bias_y*bias_y;
                if(dis < min_dis){
                    min_dis = dis;
                    mindis_class = k;
                }
            }
                cluster_index[mindis_class].push_back(index);
        }
        //求每个类的中心
        for(int i = 0;i <class_;i++){
            PointXY sum;
            sum.x = 0;
            sum.y = 0;
            for(int j=0;j<cluster_index[i].size();j++){
                int w = cluster_index[i][j];
                sum.x +=  (*db)[w].x;
                sum.y +=  (*db)[w].y;
            }
            (*center)[i].x = sum.x/(double)cluster_index[i].size();
            (*center)[i].y = sum.y/(double)cluster_index[i].size();
        }
    }
    return cluster_index;
}