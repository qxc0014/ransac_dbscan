#include "knn.h"
const float min_extent = 1;
void result::add_point(float bias,int node_index){
        if(bias >= worst_dis_cap[this->size-1].dist) return;//大于最大值直接跳出
        if(num != this->size) num++;//已插入值的个数
        int i = num-1;//已经插入最大值的index
        while(i>0){
             if(bias < worst_dis_cap[i-1].dist){
                this->worst_dis_cap[i] = worst_dis_cap[i-1];
                i--;
             }else{
                break;
             }
        }
        worst_dis_cap[i].dist = bias;
        worst_dis_cap[i].index = node_index;
        this->worst_dis = worst_dis_cap[this->size-1].dist;
}
void result::add_point_radius(const float &bias,const int &node_index){
        if(bias >= this->worst_dis) return;//大于最大值直接跳出
        only_index.push_back(node_index);
}
Octant* build_octree(Octant* root,pcl::PointCloud<PointXYZ>::Ptr db,PointXYZ center,float extent,vector<int> points_index,int depth,int width){
    if(points_index.size() == 0) {
        return nullptr;
    }
    if(root == nullptr){
        depth++;
       // cout << "节点深度：" << depth << "节点宽度" << width << endl;
        root = new Octant(nullptr,center,extent,points_index,true,depth);
    }
    if(extent < min_extent && points_index.size()<=1){
        root->is_leaf_ = true;//叶子节点
    }else{
        root->is_leaf_ = false;//不是叶子
        vector<vector<int>> child_point_index(8);
        for(auto point_idx:points_index){
            int Coordinate = 0;
            if((*db)[point_idx].x > center.x){
                Coordinate = Coordinate | 1;
            }
            if((*db)[point_idx].y > center.y){
                Coordinate = Coordinate | 2;
            }
            if((*db)[point_idx].z > center.z){
                Coordinate = Coordinate | 4;
            }
            child_point_index[Coordinate].push_back(point_idx);
        }
        float factor[2] = {-0.5,0.5};
        vector<PointXYZ> child_center(8);
        float child_extent=0;
        for(int i = 0;i < 8;i++){
            child_center[i].x = center.x + factor[(i&1)>0]*extent;
            child_center[i].y = center.y + factor[(i&2)>0]*extent;
            child_center[i].z = center.z + factor[(i&4)>0]*extent;
            child_extent = 0.5 *extent;
            //cout << child_extent << endl;
            root->child_root_[i] = build_octree(root->child_root_[i],db,child_center[i],child_extent,child_point_index[i],depth,i);
        }   
    }
    return root;
} 
//判断球与立方体的方位
bool overlap(Octant* root,const PointXYZ &Point,const float &worst_dis){
    //分三种情况:
    //第一种:球与立方体没有接触,只要投影的某个方向满足就可以
    float xyz[3];
    xyz[0] = fabs(root->center_.x - Point.x);
    xyz[1] = fabs(root->center_.y - Point.y);
    xyz[2] = fabs(root->center_.z - Point.z);
    float max_dis = (root->extent_+ worst_dis);
    if( xyz[0] > max_dis || xyz[1] > max_dis || xyz[2] > max_dis) return false;
    //第二种:球与立方体相交（通过投影判断）至少有两个投影面包含了圆心就可以认为是相交
    if(((xyz[0]<root->extent_)+(xyz[1]<root->extent_)+(xyz[2]<root->extent_))>=2) return true;
    //第三种:补充第二种，在边界处相交不满足第二种
    float x = (xyz[0]-root->extent_)>0?(xyz[0]-root->extent_):0;
    float y = (xyz[1]-root->extent_)>0?(xyz[1]-root->extent_):0;
    float z = (xyz[2]-root->extent_)>0?(xyz[2]-root->extent_):0;
    if(x*x+y*y+z*z<worst_dis*worst_dis) return true;
}
//判断球是否在立方体内
bool inside(Octant* root,const PointXYZ &Point,const float &worst_dis){
    float xyz[3];
    xyz[0] = fabs(root->center_.x - Point.x);
    xyz[1] = fabs(root->center_.y - Point.y);
    xyz[2] = fabs(root->center_.z - Point.z);
    float max_dis = (root->extent_ - worst_dis);
    return ((xyz[0] < max_dis) && (xyz[1] < max_dis) && (xyz[2] < max_dis));

}
bool octree_knn_search(Octant* root,pcl::PointCloud<PointXYZ>::Ptr db,PointXYZ Point,result &a){
    //先判断当前root是否为空指针
    if(root == nullptr) return false;
    //判断当前的节点是否为叶子
    if((root->is_leaf_ == true) && root->points_index_.size() == 1){
       //计算worst_dis
       //cout << "找到叶子！" << endl;
       Eigen::Vector3d radius(Point.x - (*db)[root->points_index_[0]].x,
                              Point.y - (*db)[root->points_index_[0]].y,
                              Point.z - (*db)[root->points_index_[0]].z);
       a.add_point(radius.squaredNorm(),root->points_index_[0]);
       //a.worst_dis = a.worst_dis < dis? a.worst_dis:dis;
       //判断现在的球是否在立方体内，如果在可以提前终止
       bool q = inside(root,Point,a.worst_dis);
      // cout << a.worst_dis_cap[0].dist << endl;
       return q;
    }
    //判断目标点所属象限
    int Coordinate = 0;
    if(Point.x > root->center_.x){
        Coordinate = Coordinate | 1;
    }
    if(Point.y > root->center_.y){
        Coordinate = Coordinate | 2;
    }
    if(Point.z > root->center_.z){
        Coordinate = Coordinate | 4;
    }
    //迭代寻找新的子象限
    if(octree_knn_search(root->child_root_[Coordinate],db,Point,a)) return true;
    //当发现最近的子象限都不能完全包裹最坏距离，那么就要扫描其他的子象限
    for(int i = 0;i<8;i++){
        //先排除刚才已经扫描过的象限
        if(i == Coordinate || root->child_root_[i] == nullptr) continue;
        //再排除球与立方体不相交的情况
        //cout << i << endl;
        if(false == overlap(root->child_root_[i],Point,a.worst_dis)) continue;
        //最后对这个象限进行计算worst_dis
        if(octree_knn_search(root->child_root_[i],db,Point,a)) return true;
    }

    //再次判断现在的球是否在立方体内，如果在可以提前终止
    return inside(root,Point,a.worst_dis);
}
bool octree_radius_search(Octant* root,pcl::PointCloud<PointXYZ>::Ptr db,const PointXYZ &Point,result &a){
    //先判断当前root是否为空指针
    if(root == nullptr) return false;
    //判断当前的节点是否为叶子
    if((root->is_leaf_ == true) && root->points_index_.size() == 1){
       //计算worst_dis
       //cout << "找到叶子！" << endl;
       Eigen::Vector3d radius(Point.x - (*db)[root->points_index_[0]].x,
                              Point.y - (*db)[root->points_index_[0]].y,
                              Point.z - (*db)[root->points_index_[0]].z);
       a.add_point_radius(radius.squaredNorm(),root->points_index_[0]);
       //a.worst_dis = a.worst_dis < dis? a.worst_dis:dis;
       //判断现在的球是否在立方体内，如果在可以提前终止
       return inside(root,Point,a.worst_dis);
    }
    //判断目标点所属象限
    int Coordinate = 0;
    if(Point.x > root->center_.x){
        Coordinate = Coordinate | 1;
    }
    if(Point.y > root->center_.y){
        Coordinate = Coordinate | 2;
    }
    if(Point.z > root->center_.z){
        Coordinate = Coordinate | 4;
    }
    //迭代寻找新的子象限
    if(octree_radius_search(root->child_root_[Coordinate],db,Point,a)) return true;
    //当发现最近的子象限都不能完全包裹最坏距离，那么就要扫描其他的子象限
    for(int i = 0;i<8;i++){
        //先排除刚才已经扫描过的象限
        if(i == Coordinate || root->child_root_[i] == nullptr) continue;
        //再排除球与立方体不相交的情况
        //cout << i << endl;
        if(false == overlap(root->child_root_[i],Point,a.worst_dis)) continue;
        //最后对这个象限进行计算worst_dis
        if(octree_radius_search(root->child_root_[i],db,Point,a)) return true;
    }

    //再次判断现在的球是否在立方体内，如果在可以提前终止
    return inside(root,Point,a.worst_dis);
}