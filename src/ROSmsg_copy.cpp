#include "ROSmsg.h"

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

//    Eigen::Matrix4f trans_mat0;
//    Eigen::Affine3f transform_affine0;
//    Eigen::Quaternionf quaternion0(-0.5, 0.5, -0.5, 0.5);
//    Eigen::AngleAxisf rotation_vector0(quaternion0);
//    transform_affine0.rotate(rotation_vector0);
//    transform_affine0.translation() << 0, 0, 0;
//    trans_mat0 = transform_affine0.matrix();

//    Eigen::Vector3f eulerAngle(-M_PI/2, 0, -M_PI/2);
//    Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(eulerAngle(0),Eigen::Vector3f::UnitX()));
//    Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(eulerAngle(1),Eigen::Vector3f::UnitY()));
//    Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(eulerAngle(2),Eigen::Vector3f::UnitZ()));
//    Eigen::AngleAxisf rotation_vector0();
//    Eigen::Quaternionf quaternion0;
//    quaternion0 = yawAngle * pitchAngle * rollAngle;
//    std::cout << quaternion0.x() << " "
//            << quaternion0.y() << " "
//            << quaternion0.z() << " "
//            << quaternion0.w() << " "
//            << "\n\n";

    float x, y, z, rx, ry, rz, rw;
    word >> x >> y >> z >> rx >> ry >> rz >> rw;
    x += -2.0;
    y += 0.0;
    z += 0.1;

    Eigen::Matrix4f trans_mat1;
    Eigen::Quaternionf quaternion1(rw, rx, ry, rz);
    Eigen::AngleAxisf rotation_vector1(quaternion1);
    Eigen::Affine3f transform_affine1;
    transform_affine1.rotate(rotation_vector1);
    transform_affine1.translation() << x, y, z;
    trans_mat1 = transform_affine1.matrix();

//    trans_mat = trans_mat0 * trans_mat1;
//
//    std::cout << trans_mat0 << "\n\n";
//
    std::cout << trans_mat1 << "\n\n";
//
//    std::cout << trans_mat << "\n\n";

    pcl::transformPointCloud(*cloud_origin, *cloud_aligned, trans_mat.inverse());
}