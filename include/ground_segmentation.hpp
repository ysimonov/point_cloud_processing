#ifndef GROUND_SEGMENTATION_HPP_
#define GROUND_SEGMENTATION_HPP_

#include "patchworkpp.hpp"
#include <chrono>
#include <memory>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <unordered_set>
#include <utility>
#include <vector>

/* TODO:
 * 1. Check https://en.wikipedia.org/wiki/Random_sample_consensus
 * 2. Set minimum number of points to estimate parameters based on some kind of permutation (minima?)
 * 3. Some kind of metric to break from the loop prematurely?
 *
 */

template <typename PointT> using CloudT = typename pcl::PointCloud<PointT>::Ptr;

// Ground segmentation based on RANSAC and pcl
template <typename PointT>
std::pair<CloudT<PointT>, CloudT<PointT>> segmentGroundCustomRANSAC(const CloudT<PointT> &cloud, const int &iterations,
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

    // set randomness
    srand((unsigned)time(nullptr));

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

    typename std::pair<CloudT<PointT>, CloudT<PointT>> result(ground_cloud, nonground_cloud);
    // ====== Algorithm Finish ======

    auto stop_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - start_time).count() / 1000.0f;

    std::cout << "Segmentation time: " << elapsed_time << " seconds\n";

    return result;
}

template <typename PointT>
std::pair<CloudT<PointT>, CloudT<PointT>> segmentGroundPclRANSAC(const CloudT<PointT> &cloud, const int &iterations,
                                                                 const float &dist_threshold)
{

    auto start_time = std::chrono::high_resolution_clock::now();

    // ====== Algorithm Start ======

    std::vector<int> ground_indices;
    pcl::PointIndices::Ptr ground_indices_object = std::make_shared<pcl::PointIndices>();
    pcl::Indices nonground_indices;

    typename pcl::SampleConsensusModelPlane<PointT>::Ptr plane_model =
        std::make_shared<pcl::SampleConsensusModelPlane<PointT>>(cloud);

    typename pcl::RandomSampleConsensus<PointT> ransac(plane_model);
    ransac.setMaxIterations(iterations);
    ransac.setDistanceThreshold(dist_threshold);
    ransac.setNumberOfThreads(0); // set threads automatically to max
    ransac.computeModel(1);
    ransac.getInliers(ground_indices);

    CloudT<PointT> ground_cloud = std::make_shared<pcl::PointCloud<PointT>>();
    CloudT<PointT> nonground_cloud = std::make_shared<pcl::PointCloud<PointT>>();

    pcl::copyPointCloud(*cloud, ground_indices, *ground_cloud);

    typename pcl::ExtractIndices<PointT> extract;
    ground_indices_object->indices = ground_indices;
    extract.setInputCloud(cloud);
    extract.setIndices(ground_indices_object);
    extract.setNegative(true);
    extract.filter(nonground_indices);

    pcl::copyPointCloud(*cloud, nonground_indices, *nonground_cloud);

    typename std::pair<CloudT<PointT>, CloudT<PointT>> result(ground_cloud, nonground_cloud);

    // ====== Algorithm Finish ======

    auto stop_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - start_time).count() / 1000.0f;

    std::cout << "Segmentation time: " << elapsed_time << " seconds\n";

    return result;
}

void convertPCLToEigen(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_pcl, Eigen::MatrixXf &cloud_eigen)
{
    size_t number_of_points = cloud_pcl->points.size();
    cloud_eigen = Eigen::MatrixXf(number_of_points, 4);
    for (size_t i = 0; i < number_of_points; ++i)
    {
        cloud_eigen(i, 0) = cloud_pcl->points[i].x;
        cloud_eigen(i, 1) = cloud_pcl->points[i].y;
        cloud_eigen(i, 2) = cloud_pcl->points[i].z;
        cloud_eigen(i, 3) = cloud_pcl->points[i].intensity;
    }
}

void convertPCLToEigen(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_pcl, Eigen::MatrixXf &cloud_eigen)

{
    size_t number_of_points = cloud_pcl->points.size();
    cloud_eigen = Eigen::MatrixXf(number_of_points, 3);
    for (size_t i = 0; i < number_of_points; ++i)
    {
        cloud_eigen(i, 0) = cloud_pcl->points[i].x;
        cloud_eigen(i, 1) = cloud_pcl->points[i].y;
        cloud_eigen(i, 2) = cloud_pcl->points[i].z;
    }
}

void convertEigenToPCL(const Eigen::MatrixXf &cloud_eigen, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_pcl)
{
    size_t num_pts = cloud_eigen.rows();
    cloud_pcl->points.resize(num_pts);
    for (size_t i = 0; i < num_pts; ++i)
    {
        float intensity = 1.0f;
        cloud_pcl->points[i].getVector3fMap() =
            Eigen::Vector3f(cloud_eigen(i, 0), cloud_eigen(i, 1), cloud_eigen(i, 2));
    }
}

void convertEigenToPCL(const Eigen::MatrixXf &cloud_eigen, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_pcl)
{
    size_t num_pts = cloud_eigen.rows();
    cloud_pcl->points.resize(num_pts);
    for (size_t i = 0; i < num_pts; ++i)
    {
        float intensity = 1.0f;
        cloud_pcl->points[i].getVector4fMap() =
            Eigen::Vector4f(cloud_eigen(i, 0), cloud_eigen(i, 1), cloud_eigen(i, 2), intensity);
    }
}

template <typename PointT>
std::pair<CloudT<PointT>, CloudT<PointT>> segmentGroundPatchworkpp(const CloudT<PointT> &cloud,
                                                                   patchwork::PatchWorkpp &patchworkpp)
{
    // Convert pcl to Eigen
    Eigen::MatrixXf cloud_eigen;
    convertPCLToEigen(cloud, cloud_eigen);

    // Estimate Ground with PatchWork
    patchworkpp.estimateGround(cloud_eigen);

    // Get Ground and Non-ground points
    Eigen::MatrixX3f ground_cloud_eigen = patchworkpp.getGround();
    Eigen::MatrixX3f nonground_cloud_eigen = patchworkpp.getNonground();
    double patchworkpp_time_tiken = patchworkpp.getTimeTaken();

    // Convert back to PCL
    CloudT<PointT> ground_cloud = std::make_shared<pcl::PointCloud<PointT>>();
    CloudT<PointT> nonground_cloud = std::make_shared<pcl::PointCloud<PointT>>();

    convertEigenToPCL(ground_cloud_eigen, ground_cloud);
    convertEigenToPCL(nonground_cloud_eigen, nonground_cloud);

    typename std::pair<CloudT<PointT>, CloudT<PointT>> result(ground_cloud, nonground_cloud);

    std::cout << "Patchworkpp Time Taken : " << patchworkpp_time_tiken / 1000000 << "(sec)" << std::endl;

    return result;
}

#endif /* GROUND_SEGMENTATION_HPP_ */