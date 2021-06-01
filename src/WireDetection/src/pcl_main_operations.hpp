//
// Created by Anton Veynshter on 19.03.20.
//

#pragma once

#include <iostream>
#include <string>
#include <stdlib.h>
#include <cmath>
#include <unistd.h>
#include <vector>

#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>

#include "PhoXi.h"

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

std::vector<pcl::PointIndices> euclidian_clustering(CloudT::Ptr cloud_filtered) {

    std::vector<pcl::PointIndices> cluster_indices;
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_filtered);
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.0003); // 2cm
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(2500000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);
    return cluster_indices;
}

CloudT::Ptr passThrough(double limit_min, double limit_max, CloudT::Ptr pcl_points) {
    // pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(pcl_points);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(limit_min, limit_max);
    // pass.setFilterLimitsNegative (true);
    pass.filter(*pcl_points);

    // pcl::IndicesPtr indices (new std::vector <int>);
    pass.setInputCloud(pcl_points);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 0.3);
    // pass.setFilterLimitsNegative (true);
    pass.filter(*pcl_points);
    return pcl_points;
}

CloudT::Ptr voxelGrid(double leaf_size, CloudT::Ptr pcl_points) {

    CloudT::Ptr cloud_filtered(new CloudT);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(pcl_points);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloud_filtered);
    return cloud_filtered;
}

void area_bounding_box(CloudT::Ptr cloud_cluster, PointT &min_point_AABB, PointT &max_point_AABB,
                       PointT &min_point_OBB, PointT &max_point_OBB, PointT &position_OBB,
                       Eigen::Matrix3f &rotational_matrix_OBB) {
    pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
    feature_extractor.setInputCloud(cloud_cluster);
    feature_extractor.compute();
    std::vector<float> moment_of_inertia;
    std::vector<float> eccentricity;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    feature_extractor.getMomentOfInertia(moment_of_inertia);
    feature_extractor.getEccentricity(eccentricity);
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues(major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter(mass_center);
}
