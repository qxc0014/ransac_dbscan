#include "ransac.h"

std::pair<Eigen::Vector3d,PointXYZ> ransac(pcl::PointCloud<PointXYZ>::Ptr db,int max_iter){
    srand(time(0));
    std::vector<int> index_final;
    PointXYZ plat_point;
    Eigen::Vector3d ABC;
    index_final.clear();
    while(max_iter--){
        std::vector<int> index;
        index.clear();
        for(int k =0;k<3;k++){
            index.push_back(rand()%(db->size()));
        }
        double x1, y1,z1, x2, y2,z2, x3, y3,z3;
        auto idx = index.begin();
        x1 = (*db)[*idx].x;
        y1 = (*db)[*idx].y;
        z1 = (*db)[*idx].z;
        idx++;
        x2 = (*db)[*idx].x;
        y2 = (*db)[*idx].y;
        z2 = (*db)[*idx].z;
        idx++;
        x3 = (*db)[*idx].x;
        y3 = (*db)[*idx].y;
        z3 = (*db)[*idx].z;
        Platform p;
        p.a = (y2 - y1)*(z3 - z1) - (z2-z1)*(y3 - y1);
        p.b = (z2 - z1)*(x3 - x1) - (x2-x1)*(z3 - z1);
        p.c = (x2 - x1)*(y3 - y1) - (y2-y1)*(x3 - x1);
        p.d = -(p.a*x2 + p.b*y2 + p.c*z2);
        for(int i=0;i < db->size();i++){
            double x4 = (*db)[i].x;
            double y4 = (*db)[i].y;
            double z4 = (*db)[i].z;
            double dis = fabs((x4-x2)*p.a+(y4-y2)*p.b+(z4-z2)*p.c)/sqrt(p.a*p.a+p.b*p.b+p.c*p.c);
            if(dis<0.12){
                index.push_back(i);
            }
        }
        //更新集合
        if(index.size()>index_final.size()){
            index_final = index;
            plat_point = PointXYZ(x1,y1,z1);
            ABC = Eigen::Vector3d(p.a,p.b,p.c);
        }
    }
    return make_pair(ABC,plat_point);
}