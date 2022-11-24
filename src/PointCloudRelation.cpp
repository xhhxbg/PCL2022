#include "PointCloudRelation.h"

extern ICPOptions opt;

PointCloudRelation::PointCloudRelation() {
    overlapA = (pclPtr)(new pcl::PointCloud<pcl::PointXYZ>);
    overlapB = (pclPtr)(new pcl::PointCloud<pcl::PointXYZ>);
}

int PointCloudRelation::is_pair() const {
    return pairFlag;
}

void PointCloudRelation::set_relation(const cloudPtr &pcA, const cloudPtr &pcB) {
    this->pcA = pcA;
    this->pcB = pcB;
}

cloudPtr PointCloudRelation::find_another(const cloudPtr &pc) {
    if(pc == pcA)
        return pcB;
    else if(pc == pcB)
        return pcA;
    else
        return nullptr;
}

// pcA * trans_mat(pcA) = pcB; pcB * trans_mat(pcB) = pcA;
Eigen::Matrix4f PointCloudRelation::trans_mat(const cloudPtr &pc) {
    if(pc == pcA)
        return transform_matrix;
    else if(pc == pcB)
        return transform_matrix.inverse();


}

void PointCloudRelation::find_overlap() {
    pclPtr &source = pcA->cloud_voxel;
    pclPtr &target = pcB->cloud_voxel;

    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>core;
    core.setInputSource(source);
    core.setInputTarget(target);
    pcl::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences); //共享所有权的智能指针，以kdtree做索引
    core.determineReciprocalCorrespondences(*cor, opt.MaxDistance); //点之间的最大距离,cor对应索引

    if(cor->size() > opt.MinOverlapping)
    {
        std::cout << "  Found overlapping points: " << cor->size() << std::endl;

        // overlap point cloud
        overlapA->width = overlapB->width = cor->size();
        overlapA->height = overlapB->height = 1;
        overlapA->is_dense = overlapB->is_dense = false;
        overlapA->points.resize(overlapA->width * overlapA->height);
        overlapB->points.resize(overlapB->width * overlapB->height);

        for (size_t i = 0; i < cor->size(); i++)
        {
            overlapA->points[i].x = source->points[cor->at(i).index_query].x;
            overlapA->points[i].y = source->points[cor->at(i).index_query].y;
            overlapA->points[i].z = source->points[cor->at(i).index_query].z;

            overlapB->points[i].x = target->points[cor->at(i).index_match].x;
            overlapB->points[i].y = target->points[cor->at(i).index_match].y;
            overlapB->points[i].z = target->points[cor->at(i).index_match].z;
        }
        pairFlag = true;
    }
    else
    {
        std::cout << "  No overlapping points." << std::endl;
        pairFlag = false;
    }
}

void PointCloudRelation::run_icp() {
    std::cout << "Running ICP Algorithm ......" << std::endl;
    pclPtr &key_src = overlapA;
    pclPtr &key_tgt = overlapB;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setRadiusSearch(opt.PlaneSearchRadius);
    // 计算Target法线表示
    pcl::PointCloud<pcl::Normal>::Ptr tgt_norm(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(key_tgt);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(key_tgt);
    ne.setSearchMethod(tree);
    // 计算每个点处的法向量
    ne.compute(*tgt_norm);
    // 拼接法线表示与原始点云
    pcl::PointCloud<pcl::PointNormal>::Ptr tgt_concat(
            new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*key_tgt, *tgt_norm, *tgt_concat);
    // 同理，计算Source法线表示
    pcl::PointCloud<pcl::Normal>::Ptr src_norm(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(key_src);
    tree->setInputCloud(key_src);
    ne.setSearchMethod(tree);
    ne.compute(*src_norm);
    pcl::PointCloud<pcl::PointNormal>::Ptr src_concat(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*key_src, *src_norm, *src_concat);
    // ICP
    pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
    // 设置点到面匹配作为规则
    pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>::Ptr
            trans_lls(new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>);
    icp.setTransformationEstimation (trans_lls);
    // 设置输入点云和配置参数
    icp.setInputSource (src_concat);
    icp.setInputTarget (tgt_concat);

    icp.setMaximumIterations(opt.MaxNoIt); // 最大迭代次数
    icp.setTransformationEpsilon(1e-30); // 两次变化矩阵之间的差值
    icp.setEuclideanFitnessEpsilon(opt.MaxRoughness); // 均方误差
    icp.align(*src_concat);

    transform_matrix = icp.getFinalTransformation();
    match_score = icp.getFitnessScore();

    std::cout << "  ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
}

