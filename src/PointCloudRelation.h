#ifndef MY_GRAND_PROJECT_POINTCLOUDRELATION_H
#define MY_GRAND_PROJECT_POINTCLOUDRELATION_H

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/3dsc.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/icp.h>

#include "ICPDefine.h"
#include "PointCloudNode.h"
#include "ICPOptions.h"

class PointCloudRelation {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointCloudRelation();
    void set_relation(const cloudPtr &pcA, const cloudPtr &pcB);
    void find_overlap();
    cloudPtr find_another(const cloudPtr &pc);
    Eigen::Matrix4f trans_mat(const cloudPtr &pc);
    void run_icp();
    int is_pair() const;


    bool pairFlag;
    cloudPtr pcA;
    cloudPtr pcB;
    pclPtr overlapA;
    pclPtr overlapB;
    Eigen::Matrix4f transform_matrix;
    float match_score;
};


#endif //MY_GRAND_PROJECT_POINTCLOUDRELATION_H
