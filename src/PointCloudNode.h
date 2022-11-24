#ifndef MY_GRAND_PROJECT_POINTCLOUDNODE_H
#define MY_GRAND_PROJECT_POINTCLOUDNODE_H

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include "ICPDefine.h"

typedef class PointCloudNode {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointCloudNode();
    void set_index(const int &index);
    void load_cloud(const std::string &file_name);
    void get_voxel(const float &hullvoxelsize);

    pclPtr cloud;
    pclPtr cloud_voxel;
    pclPtr cloud_target;
    std::string label;

    int index;
    bool is_fixed;
    Eigen::Matrix4f mat_to_fix;

} *cloudPtr;


#endif //MY_GRAND_PROJECT_POINTCLOUDNODE_H
