#ifndef MY_GRAND_PROJECT_ICPOPERATION_H
#define MY_GRAND_PROJECT_ICPOPERATION_H

#include <vector>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include "PointCloudNode.h"
#include "PointCloudRelation.h"
#include "ICPOptions.h"

class ICPOperation {
public:
    ICPOperation();
    std::vector<cloudPtr> PCGroup;
    std::vector<PointCloudRelation> PCRelations;
    void add_file(const std::string &file_name);
    void pair_clouds();
    void make_tree();
    void merge_cloud();
    void show_origin();

private:
    int PCNum;
};


#endif //MY_GRAND_PROJECT_ICPOPERATION_H
