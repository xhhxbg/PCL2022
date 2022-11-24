#include "PointCloudNode.h"

PointCloudNode::PointCloudNode() {
    cloud = (pclPtr)(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_voxel = (pclPtr)(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_target = (pclPtr)(new pcl::PointCloud<pcl::PointXYZ>);
    is_fixed = false;
}

void PointCloudNode::set_index(const int &index) {
    this->index = index;
    label = "cloud_" + std::to_string(index);
}

// load pcd/ply file into this->cloud
void PointCloudNode::load_cloud(const std::string &file_name) {
    if(file_name.find(".ply") != std::string::npos)
        pcl::io::loadPLYFile<pcl::PointXYZ>(file_name, *cloud);
    if(file_name.find(".pcd") != std::string::npos)
        pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud);
}

// result store in this->cloud_voxel
void PointCloudNode::get_voxel(const float &hullvoxelsize) {
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> hullvoxel;
    hullvoxel.setInputCloud(cloud);
    hullvoxel.setLeafSize(hullvoxelsize, hullvoxelsize, hullvoxelsize);
    hullvoxel.filter(*cloud_voxel);
}

