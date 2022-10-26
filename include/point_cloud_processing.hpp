#ifndef POINT_CLOUD_PROCESSING_HPP_
#define POINT_CLOUD_PROCESSING_HPP_

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <eigen3/Eigen/Dense>

#include <pcl/console/print.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/visualization/keyboard_event.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>

#define BOOST_FILESYSTEM_VERSION 3
#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>

#endif // POINT_CLOUD_PROCESSING_HPP_