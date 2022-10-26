#ifndef GROUND_SEGMENTATION_HPP_
#define GROUND_SEGMENTATION_HPP_

#include <chrono>
#include <memory>
#include <pcl/point_cloud.h>
#include <unordered_set>
#include <utility>

template <typename PointT> using CloudT = typename pcl::PointCloud<PointT>::Ptr;

// Ground segmentation based on RANSAC and pcl
template <typename PointT>
std::pair<CloudT<PointT>, CloudT<PointT>> segmentGround(const CloudT<PointT> &cloud, const int &iterations,
                                                        const float &dist_threshold)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    // ====== Algorithm Start ======

    size_t num_cloud_points = cloud->points.size();
    std::unordered_set<unsigned int> inlier_indices;
    PointT point1;
    PointT point2;
    PointT point3;
    unsigned int idx1;
    unsigned int idx2;
    unsigned int idx3;
    float a, b, c, d;
    float distance;
    float length;
    srand((unsigned)time(NULL));

    for (int iter = 0; iter < iterations; ++iter)
    {
        std::unordered_set<unsigned int> temp_indices;

        // get 3 random points
        // generates random numbers between 0 and num_cloud_points
        while (temp_indices.size() < 3)
            temp_indices.insert((rand() % num_cloud_points));

        auto it = temp_indices.begin();
        idx1 = *it;
        ++it;
        idx2 = *it;
        ++it;
        idx3 = *it;

        point1 = cloud->points[idx1];
        point2 = cloud->points[idx2];
        point3 = cloud->points[idx3];

        // fit plane points
        a = (((point2.y - point1.y) * (point3.z - point1.z)) - ((point2.z - point1.z) * (point3.y - point1.y)));
        b = (((point2.z - point1.z) * (point3.x - point1.x)) - ((point2.x - point1.x) * (point3.z - point1.z)));
        c = (((point2.x - point1.x) * (point3.y - point1.y)) - ((point2.y - point1.y) * (point3.x - point1.x)));
        d = -(a * point1.x + b * point1.y + c * point1.z);
        length = std::sqrt(a * a + b * b + c * c);

        // find inlier indices
        for (size_t i = 0; i < num_cloud_points; ++i)
        {
            if (i != idx1 || i != idx2 || i != idx3)
            {
                distance =
                    std::fabs(a * cloud->points[i].x + b * cloud->points[i].y + c * cloud->points[i].z + d) / length;

                // if distance is smaller than threshold count, it is an inlier point
                if (distance <= dist_threshold)
                {
                    temp_indices.insert(i);
                }
            }
        }

        // store temporary buffer if there are more than previously identified points
        if (temp_indices.size() > inlier_indices.size())
        {
            // update inlier indices
            inlier_indices = std::move(temp_indices);
        }
    }

    // segment the larger planar component from the remaining cloud
    if (inlier_indices.empty())
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    // ground and non-ground points
    CloudT<PointT> ground_cloud(new pcl::PointCloud<PointT>);
    CloudT<PointT> nonground_cloud(new pcl::PointCloud<PointT>);

    for (size_t i = 0; i < num_cloud_points; ++i)
    {
        const PointT &point = cloud->points[i];
        if (inlier_indices.count(i))
        {
            ground_cloud->points.emplace_back(point);
        }
        else
        {
            nonground_cloud->points.emplace_back(point);
        }
    }

    std::pair<CloudT<PointT>, CloudT<PointT>> result(ground_cloud, nonground_cloud);
    // ====== Algorithm Finish ======

    auto stop_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - start_time).count() / 1000.0f;

    std::cout << "Segmentation time: " << elapsed_time << " seconds\n";

    return result;
}

#endif /* GROUND_SEGMENTATION_HPP_ */