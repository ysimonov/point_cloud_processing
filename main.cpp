#include "object_detection.hpp"

static bool next_iteration = false;

namespace fs = boost::filesystem;

/**
 * @brief   Return the filenames of all files that have the specified extension
 *          in the specified directory and all subdirectories.
 */
std::vector<fs::path> readFilenamesExt(fs::path const &root, std::string const &ext)
{
    std::vector<fs::path> paths;

    if (fs::exists(root) && fs::is_directory(root))
    {
        for (const auto &entry : fs::recursive_directory_iterator(root))
        {
            if (fs::is_regular_file(entry) && entry.path().extension() == ext)
                paths.emplace_back(entry.path());
        }
    }

    return paths;
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *cookie)
{
    if (event.getKeySym() == "space" && event.keyDown())
    {
        next_iteration = true;
    }
}

int main()
{

    std::string filepath = "/home/yevgeniy/Documents/GitHub/data/";

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    std::vector<fs::path> filenames = readFilenamesExt(filepath, ".pcd");

    // sort names lexicographically
    std::sort(filenames.begin(), filenames.end());
    std::vector<fs::path>::iterator filename_iterator = filenames.begin();

    // for (const auto& name : filenames)
    // {
    //     std::cout << "Loaded " << name.string() << std::endl;
    // }

    // prepare visualizer
    std::string point_cloud_id = "raw cloud";
    pcl::visualization::PCLVisualizer viewer("Point Cloud Animation");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud_color_handler(cloud, 0, 255, 45);
    viewer.setBackgroundColor(0, 0, 0);
    viewer.setSize(1920, 1080); // window size
    viewer.setCameraPosition(-42, -49, 58, 0.5, 0.06, 2.59);
    viewer.registerKeyboardCallback(&keyboardEventOccurred, (void *)nullptr);

    bool first_iteration = true;
    while (viewer.wasStopped() == false)
    {
        viewer.spinOnce();
        if (next_iteration || first_iteration)
        {

            // check if there are any more frames to read
            if (filename_iterator == filenames.end())
            {
                break;
            }

            // Read data file
            std::string filename = (*filename_iterator).string();
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *cloud) == -1)
            {
                PCL_ERROR("Clouldn't read .pcd file\n");
                return EXIT_FAILURE;
            }
            unsigned int number_of_points = cloud->size();
            std::cout << "Loaded " << number_of_points << " points from " << filename << " file.\n";

            // for (size_t i = 0; i < 100; ++i) std::cout << cloud->points[i] << std::endl;

            // Ground segmentation
            pcl::PointIndicesPtr ground_indices = std::make_shared<pcl::PointIndices>();
            pcl::ProgressiveMorphologicalFilter<pcl::PointXYZI> ground_filter;
            ground_filter.setInputCloud(cloud);
            ground_filter.setMaxWindowSize(20);
            ground_filter.setSlope(1.0f);
            ground_filter.setInitialDistance(-0.5f);
            ground_filter.setMaxDistance(0.4f);
            ground_filter.extract(ground_indices->indices);

            pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            pcl::ExtractIndices<pcl::PointXYZI> extractor;
            extractor.setInputCloud(cloud);
            extractor.setIndices(ground_indices);
            extractor.filter(*ground_cloud);

            std::cout << "Ground cloud after filtering: " << std::endl;
            std::cout << *ground_cloud << std::endl;

            // Visualise points
            if (first_iteration)
            {
                viewer.addPointCloud(cloud, cloud_color_handler, point_cloud_id);
            }
            else
            {
                viewer.updatePointCloud(cloud, cloud_color_handler, point_cloud_id);
            }

            Eigen::Affine3f viewer_pose = viewer.getViewerPose();
            Eigen::Matrix3f rotation_matrix = viewer_pose.rotation();
            Eigen::Vector3f translation_vector = viewer_pose.translation();

            std::cout << "Camera Rotation: " << std::endl << rotation_matrix << std::endl;
            std::cout << "Camera Translation: " << std::endl << translation_vector << std::endl;
            std::cout << viewer_pose.matrix() << std::endl;

            // viewer.cam
            filename_iterator++;
            first_iteration = false;
            next_iteration = false;
        }
    }

    return EXIT_SUCCESS;
}