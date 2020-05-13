#include "base.h"
#include "knn.h"
#include "ransac.h" 
#include "dbscan.h"
#include "show.h"
const float com_float = 2e6;
int leafsize =0; 
const float search_radius = 0.5;
const string db_list = "/home/esoman/c++code/Homework I/cluster four/cluster/data/000000.bin";
/*平面类*/


int main(int argc, char const *argv[])
{
    ifstream fin;
    fin.open(db_list,ios::binary);
    if(!fin){
		cout<<"open error!"<<endl;
		return -1;
	}
	pcl::PointCloud<PointXYZ>::Ptr points (new pcl::PointCloud<PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
	int i;
    float x_min,x_max,y_min,y_max,z_min,z_max;
    x_min = com_float;
    x_max = -com_float;
    y_min = com_float;
    y_max = -com_float;
    z_min = com_float;
    z_max = -com_float;
	for (i=0; fin.good() && !fin.eof(); i++) {
		PointXYZ point;
		fin.read((char *) &point.x, 4*sizeof(float));
		points->push_back(point);
        x_min = (x_min < point.x)?x_min:point.x;
        x_max = (x_max > point.x)?x_max:point.x;
        y_min = (y_min < point.y)?y_min:point.y;
        y_max = (y_max > point.y)?y_max:point.y;
        z_min = (z_min < point.z)?z_min:point.z;
        z_max = (z_max > point.z)?z_max:point.z;
	}
    /*降采样*/
    sor.setInputCloud(points);
    sor.setLeafSize(2.f, 2.f, 2.f);
    sor.filter(*cloud_filtered);
    /*对降采样的点云进行进行ransac*/
    int point_size = points->size()-1;
    chrono::steady_clock::time_point t5 = chrono::steady_clock::now();
    std::pair<Eigen::Vector3d,PointXYZ> plat_point;
    cout << "滤波后的点云数量：" << cloud_filtered->size() << endl;
    plat_point = ransac(cloud_filtered,300);
    /*对所有点云进行计算离平面距离,并提取对应的index*/
    std::vector<int> platform_index;//存放平面点的容器
    for(int i = 0 ;i<points->size();i++){
        Eigen::Vector3d dis_vector;
        dis_vector[0] = (*points)[i].x - plat_point.second.x;
        dis_vector[1] = (*points)[i].y - plat_point.second.y;
        dis_vector[2] = (*points)[i].z - plat_point.second.z;
        double dis;
        dis = fabs(dis_vector.dot(plat_point.first))/ (double)sqrt(plat_point.first.squaredNorm());
        if(dis < 0.28){
            platform_index.push_back(i);
        }
    }
    chrono::steady_clock::time_point t6 = chrono::steady_clock::now();
    chrono::duration<double> time_used2 = chrono::duration_cast<chrono::duration<double>>(t6 - t5)*1000;
    cout << "ransac用时 = " << time_used2.count() << " ms.    " << endl;
    pcl::PointCloud<PointXYZ>::Ptr plat_points (new pcl::PointCloud<PointXYZ>);
    int bias=0;//指针偏移 当erase掉一个point后后面的point地址会往前移一个
    for(int index:platform_index){
        //plat_points->push_back((*points)[index-bias]);
        points->erase(points->begin()+index-bias);
        bias++;
    }
    std::vector<int> points_index(points->size());
    points_index[0] = 0;
    std::partial_sum(points_index.begin(), points_index.end(), points_index.begin(), [](const int&a, int b) {return a + 1;});
    /*计算包含所有点的大立方体*/
    PointXYZ center((x_min+x_max)/2.,(y_min+y_max)/2.,(z_min+z_max)/2.);
    float extent = 0;
    extent = (y_max-y_min)<(x_max-x_min)?(x_max-x_min):(y_max-y_min);
    extent = extent<(z_max-z_min)?(z_max-z_min):extent;
    extent = ceil(extent / 2.);
    cout << "点云个数" << points_index.size() << endl;
    /*建立八叉树*/
    Octant* root;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    root = build_octree(root,points,center,extent,points_index,0,-1);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1)*1000;
    cout << "建立八叉树用时 = " << time_used.count() << " ms.    " << endl;
    /*调用八叉树库*/
    /*
    float resolution=1.0f; //分辨率
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);//初始化octree
    octree.setInputCloud(points);
    octree.addPointsFromInputCloud();
    vector<int> pointIdxVec;
    std::vector<float> pointRadiusSquaredDistance;
    chrono::steady_clock::time_point t7 = chrono::steady_clock::now();
    octree.radiusSearch((*points)[0],1,pointIdxVec,pointRadiusSquaredDistance);
    chrono::steady_clock::time_point t8 = chrono::steady_clock::now();
    chrono::duration<double> time_used4 = chrono::duration_cast<chrono::duration<double>>(t8 - t7)*1000;
    cout << "搜索用时 = " << time_used4.count() << " ms.    " << endl;
    */
    /*dbscan搜索*/
    chrono::steady_clock::time_point t3 = chrono::steady_clock::now();
    dbscan scan(points,root,search_radius,8);//radius与min_sample
    scan.run();
    chrono::steady_clock::time_point t4 = chrono::steady_clock::now();
    chrono::duration<double> time_used1 = chrono::duration_cast<chrono::duration<double>>(t4 - t3)*1000;
    cout << "dbscan用时 = " << time_used1.count() << " ms.    " << endl;
    int cluster_num = *max_element(scan.cluster_state->begin(),scan.cluster_state->end()) + 1;
    vector<pcl::PointCloud<PointXYZ>::Ptr> cluster_list;
    for(int i=0;i<cluster_num;i++){
       pcl::PointCloud<PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<PointXYZ>);
       cluster_list.push_back(cloud_ptr);
    }
    for(int index =0;index<points->size();index++){
        int reslut = (*scan.cluster_state)[index];
        cluster_list[reslut]->push_back((*points)[index]);
    }
   
    /*显示*/
    show_point_cloud(cluster_list,cluster_num);
   // }

    return 0;
}