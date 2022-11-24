#ifndef MY_GRAND_PROJECT_ROSMSG_H
#define MY_GRAND_PROJECT_ROSMSG_H

#include <math.h>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "pcl/filters/conditional_removal.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include "ICPDefine.h"

class ROSmsg {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ROSmsg();
    ROSmsg(int i, int j);

    pclPtr cloud_origin;
    pclPtr cloud_aligned;

    std::string data_path;
    int data_group;
    int data_index;

    Eigen::Matrix4f trans_mat;
    Eigen::Affine3f transform_affine;


    void load_pcd();
    void show_origin();
    void show_aligned();
    pclPtr filter_cloud(pclPtr cloud, std::string axis, float threshold, std::string op);
    void load_mat(std::stringstream &word);
};


#endif //MY_GRAND_PROJECT_ROSMSG_H
