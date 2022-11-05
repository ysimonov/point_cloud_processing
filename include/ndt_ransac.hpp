#ifndef NDT_RANSAC_HPP_
#define NDT_RANSAC_HPP_

#include <Eigen/Dense>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace planefit
{
// using PointT = pcl::PointXYZ;
template <typename PointT>
void planeFitRANSAC(const typename pcl::PointCloud<PointT>::Ptr &point_cloud, float cell_size = 32.0f,
                    float threshold = 1e-4)
{
    // Partition point cloud into cells
    typename pcl::octree::OctreePointCloudChangeDetector<PointT> octree(cell_size);

    // Add PointCloud points to octree
    octree.setInputCloud(point_cloud);
    octree.addPointsFromInputCloud();

    // Switch OcTree buffers
    std::vector<int> point_idx_vec;

    // Get vector of point indices from octree voxel
    octree.getPointIndicesFromNewVoxels(point_idx_vec);

    // Print points
    for (size_t i = 0; i < point_idx_vec.size(); ++i)
    {
        PointT point = point_cloud->points[point_idx_vec[i]];
        std::cout << i << "# Index: " << point_idx_vec[i] << " point: " << point.x << " " << point.y << " " << point.z
                  << std::endl;
    }
}
} // namespace planefit

#endif // NDT_RANSAC_HPP_
