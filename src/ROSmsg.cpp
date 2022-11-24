#include "ROSmsg.h"
#include <cstdlib>
ROSmsg::ROSmsg(int i, int j) {
    cloud_origin = (pclPtr)(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_aligned = (pclPtr)(new pcl::PointCloud<pcl::PointXYZ>);
    data_path = "/home/yyf/Documents/ros_srtp/pcd/";
    data_group = i;
    data_index = j;
    transform_affine = Eigen::Affine3f::Identity();
}

void ROSmsg::load_pcd() {
    std::string file_name = data_path + std::to_string(data_group) + "/origin/frame-0" + std::to_string(data_index) + ".pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud_origin);
}

void ROSmsg::show_origin() {
    pcl::visualization::PCLVisualizer viewer;
    viewer.setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> rgb(cloud_origin);
    viewer.addPointCloud(cloud_origin, rgb, "1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "1");
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    while (!viewer.wasStopped())
    {
        viewer.spinOnce (100);
    }
}

void ROSmsg::show_aligned() {
    pcl::visualization::PCLVisualizer viewer;
    viewer.setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> rgb(cloud_aligned);
    viewer.addPointCloud(cloud_aligned, rgb, "2");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "2");
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    while (!viewer.wasStopped())
    {
        viewer.spinOnce (100);
    }
}

void ROSmsg::load_mat(std::stringstream &word) {

    Eigen::Matrix4f trans_mat0 = Eigen::Matrix4f::Identity();
    Eigen::Affine3f transform_affine0;
    Eigen::Quaternionf quaternion0(0.5, -0.5, 0.5, -0.5);
    Eigen::Matrix3f rotation_matrix0;
    rotation_matrix0 = quaternion0.matrix();

    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            trans_mat0(i, j) = rotation_matrix0(i, j);

    float x, y, z, rx, ry, rz, rw;
    word >> x >> y >> z >> rx >> ry >> rz >> rw;
    x += -0.8;
    y += 0.0;
    z += 0.01;

    Eigen::Matrix4f trans_mat1 = Eigen::Matrix4f::Identity();;
    Eigen::Quaternionf quaternion1(rw, rx, ry, rz);

    Eigen::Matrix3f rotation_matrix1;
    rotation_matrix1 = quaternion1.matrix();

    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            trans_mat1(i, j) = rotation_matrix1(i, j);
    trans_mat1(0, 3) = x;
    trans_mat1(1, 3) = y;
    trans_mat1(2, 3) = z;

    trans_mat = trans_mat0 * trans_mat1.reverse();

//    std::cout << trans_mat0 << "\n\n";
//    std::cout << trans_mat1 << "\n\n";
    std::cout << "Group: " << data_group << ", Index: " << data_index << "\n\n";

    pcl::transformPointCloud(*cloud_origin, *cloud_aligned, trans_mat0);
    pcl::transformPointCloud(*cloud_aligned, *cloud_aligned, trans_mat1);

    cloud_aligned = filter_cloud(cloud_aligned, "x", 0.5, ">");
    cloud_aligned = filter_cloud(cloud_aligned, "y", 0.5, ">");
    cloud_aligned = filter_cloud(cloud_aligned, "z", 0.01, "<");


    Eigen::Matrix4f trans_mat2 = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f rotation_matrix2;

    if(data_group == 3) {
        Eigen::Quaternionf quaternion2(0.7071, 0, 0, 0.7071);
        rotation_matrix2 = quaternion2.matrix();
    }
    else if(data_group == 2) {
        Eigen::Quaternionf quaternion2(0, 0, 0, 1);
        rotation_matrix2 = quaternion2.matrix();
    }
    else if(data_group == 1) {
        Eigen::Quaternionf quaternion2(0.7071, 0, 0, -0.7071);
        rotation_matrix2 = quaternion2.matrix();
    }
    else {
        Eigen::Quaternionf quaternion2(1, 0, 0, 0);
        rotation_matrix2 = quaternion2.matrix();
    }

    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            trans_mat2(i, j) = rotation_matrix2(i, j);

    pcl::transformPointCloud(*cloud_aligned, *cloud_aligned, trans_mat2);


//    Eigen::Matrix4f trans_mat_r = Eigen::Matrix4f::Identity();;
////    Eigen::Quaternionf quaternion_r(rw, rx, ry, rz);
//
//    Eigen::Matrix3f rotation_matrix_r;
////    rotation_matrix_r = quaternion_r.matrix();
//
////    for(int i=0; i<3; i++)
////        for(int j=0; j<3; j++)
////            trans_mat_r(i, j) = rotation_matrix_r(i, j);
//    trans_mat_r(0, 3) = 1 / (std::rand() % 20 + 1);
//    trans_mat_r(1, 3) = 1 / (std::rand() % 20 + 1);
//    trans_mat_r(2, 3) = 1 / (std::rand() % 20 + 1);
//    pcl::transformPointCloud(*cloud_aligned, *cloud_aligned, trans_mat_r);
}

pclPtr ROSmsg::filter_cloud(pclPtr cloud, std::string axis, float threshold, std::string op)
{
    pclPtr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConditionOr<pcl::PointXYZ>::Ptr range_cond(
            new pcl::ConditionOr<pcl::PointXYZ>()
    );
    pcl::ComparisonOps::CompareOp oper;
    if (op == ">")
        oper = pcl::ComparisonOps::LT;
    else
        oper = pcl::ComparisonOps::GT;

    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>(axis, oper, threshold))
    );
    // Build the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud);
    condrem.filter(*filtered);
    return filtered;
}

