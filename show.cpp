#include "show.h"
void show_point_cloud(vector<pcl::PointCloud<PointXYZ>::Ptr> cluster_list,int cluster_num){
    pcl::visualization::PCLVisualizer viewer("demo");
    int v1(0);
    viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v1);
    srand(time(0));
    for(int i=0;i<cluster_num;i++){
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cluster_color(cluster_list[i],rand()%255,rand()%255,rand()%255);
        viewer.addPointCloud(cluster_list[i], cluster_color, "cloud_in_"+to_string(i), v1); 
    }
    viewer.setSize(1280, 1024);
    viewer.setBackgroundColor(255, 255, 255);
    viewer.spin();
}