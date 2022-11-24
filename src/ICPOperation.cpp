#include "ICPOperation.h"

ICPOptions opt;

ICPOperation::ICPOperation() {
    PCNum = 0;
}

void ICPOperation::add_file(const std::string &file_name) {
    cloudPtr PCNode(new PointCloudNode);
    PCNode->set_index(PCNum);
    PCNode->load_cloud(file_name);
    PCNode->get_voxel(opt.HullVoxelSize);
    PCGroup.push_back(PCNode);
    PCNum++;
}

void ICPOperation::pair_clouds() {
    for(int i = 0; i < PCNum; i++) {
        for(int j = i + 1; j < PCNum; j++) {
            PointCloudRelation relation;
            relation.set_relation(PCGroup[i], PCGroup[j]);

            std::cout << "Pairing "
                      << PCGroup[i]->label
                      << " and "
                      << PCGroup[j]->label
                      << " ......" << std::endl;

            relation.find_overlap();
            if(relation.is_pair()) {
                relation.run_icp();
                PCRelations.push_back(relation);
            }
        }
    }
}

void ICPOperation::make_tree() {
    PCGroup[opt.IdxFixedPointClouds]->is_fixed = true;
    PCGroup[opt.IdxFixedPointClouds]->mat_to_fix.setIdentity();
    int num_fixed = PCNum - 1;
    while(num_fixed) {
        float min_score = 100.0;
        cloudPtr cloud_base;
        cloudPtr cloud_to_fix;
        PointCloudRelation new_relation;
        for(auto known: PCGroup) {
            if(known->is_fixed) {
                // found a known point cloud.
                for(auto next_relation: PCRelations) {
                    // travel its relations.
                    cloudPtr next_cloud = next_relation.find_another(known);
                    if((next_cloud != nullptr) && (!next_cloud->is_fixed)) {
                        // found an unfixed relative cloud.
                        if(next_relation.match_score < min_score) {
                            // update the min score.
                            min_score = next_relation.match_score;
                            new_relation = next_relation; // new relation to be established.
                            cloud_to_fix = next_cloud;
                            cloud_base = known;
                        }
                    }
                }
            }
        }
        std::cout << "next cloud: " << cloud_to_fix->label << ", "
                  << "base cloud: " << cloud_base->label << std::endl;
        std::cout << new_relation.transform_matrix << std::endl;
        cloud_to_fix->is_fixed = true;
        cloud_to_fix->mat_to_fix = new_relation.trans_mat(cloud_to_fix) * cloud_base->mat_to_fix;
        num_fixed--;
    }
}

void ICPOperation::show_origin() {
    pcl::visualization::PCLVisualizer viewer;
    viewer.setBackgroundColor(0, 0, 0);
    for(auto iter: PCGroup) {
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> rgb(iter->cloud);
        viewer.addPointCloud(iter->cloud, rgb, iter->label);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, iter->label);
    }
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    while (!viewer.wasStopped())
    {
        viewer.spinOnce (100);
    }
}

void ICPOperation::merge_cloud() {
    pcl::visualization::PCLVisualizer viewer;
    pclPtr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    viewer.setBackgroundColor(0, 0, 0);
    for(auto iter: PCGroup) {
        pcl::transformPointCloud(*iter->cloud, *iter->cloud_target, iter->mat_to_fix);
//        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> rgb(iter->cloud_target);
//        viewer.addPointCloud(iter->cloud_target, rgb, iter->label);
//        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, iter->label);
        *merged_cloud += *iter->cloud_target;
    }

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> hullvoxel;
    hullvoxel.setInputCloud(merged_cloud);
    hullvoxel.setLeafSize(0.01, 0.01, 0.01);
    hullvoxel.filter(*merged_cloud);

    pcl::io::savePCDFile ("../data/cloud_merged.pcd", *(merged_cloud));
    pcl::io::savePLYFile ("../data/cloud_merged.ply", *(merged_cloud));

    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> rgb(merged_cloud);
    viewer.addPointCloud(merged_cloud, rgb, "");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "");
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    while (!viewer.wasStopped())
    {
        viewer.spinOnce (100);
    }
}


int main() {
    ICPOperation icp;
    for (int group = 0; group < 4; group++)
        for (int index = 0; index < 4; index++)
            icp.add_file("../data/cloud" + std::to_string(group) + "_" + std::to_string(index) + ".pcd");

//    icp.add_file("../data/lionscan0approx.pcd");
//    icp.add_file("../data/lionscan1approx.pcd");
//    icp.add_file("../data/lionscan2approx.pcd");
//    icp.add_file("../data/lionscan3approx.pcd");
//    icp.add_file("../data/lionscan4approx.pcd");
    icp.show_origin();
    icp.pair_clouds();
    icp.make_tree();
    icp.show_origin();
    icp.merge_cloud();
}