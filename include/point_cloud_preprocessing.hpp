#ifndef POINT_CLOUD_PREPROCESSING_HPP_
#define POINT_CLOUD_PREPROCESSING_HPP_

#include <cmath>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template <typename PointT> using CloudT = typename pcl::PointCloud<PointT>::Ptr;

// This function accepts point cloud and returns PointCloud<CloudT>::Ptr -> downsampled with Voxel Grid and filtered
// with DROR filter using KDTree for nearest neighbour search
// lx, ly, lz - leaf sizes of the voxel grid
template <typename PointT>
CloudT<PointT> filterCloudDROR(const CloudT<PointT> &cloud, float lx = 0.2, float ly = 0.2, float lz = 0.2,
                               float radius_multiplier = 3.0, float azimuth_angle_res_deg = 0.04,
                               int min_neighbours = 3, int min_search_radius = 0.05)
{
    using kdtree_t = typename pcl::KdTreeFLANN<PointT>;
    using kdtree_ptr_t = typename pcl::KdTreeFLANN<PointT>::Ptr;

    int min_neighbours_inc = min_neighbours + 1;
    float azimuth_angle_res_rad = azimuth_angle_res_deg * static_cast<float>(M_PI / 180.0);

    // Downsampling
    CloudT<PointT> downsampled_cloud = std::make_shared<pcl::PointCloud<PointT>>();
    typename pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(lx, ly, lz);
    voxel_grid.filter(*downsampled_cloud);

    // Filtering (DROR)
    CloudT<PointT> filtered_cloud = std::make_shared<pcl::PointCloud<PointT>>();

    kdtree_ptr_t kdtree_ptr = std::make_shared<kdtree_t>();
    kdtree_ptr->setInputCloud(downsampled_cloud);

    for (typename pcl::PointCloud<PointT>::iterator it = downsampled_cloud->points.begin();
         it != downsampled_cloud->points.end(); ++it)
    {
        const float &xi = it->x;
        const float &yi = it->y;
        const float &zi = it->z;
        float distance_from_pov = std::sqrt(xi * xi + yi * yi + zi * zi);
        float search_radius = radius_multiplier * azimuth_angle_res_rad * std::pow(distance_from_pov / 5.0f, 3);

        if (search_radius < min_search_radius)
        {
            search_radius = min_search_radius;
        }

        std::vector<int> neigh_idxs;
        std::vector<float> neigh_dists;

        int num_neigh_found = kdtree_ptr->radiusSearch(*it, search_radius, neigh_idxs, neigh_dists);
        if (num_neigh_found > min_neighbours_inc)
        {
            filtered_cloud->points.emplace_back(*it);
        }
    }

    return filtered_cloud;
}

#endif /* POINT_CLOUD_PREPROCESSING_HPP_ */