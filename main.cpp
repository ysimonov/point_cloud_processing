#include "ground_segmentation.hpp"
#include "point_cloud_processing.hpp"

static bool next_iteration = false;
static unsigned int screenshot_no = 0;

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

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *viewer)
{
    if (event.getKeySym() == "space" && event.keyDown())
    {
        next_iteration = true;
    }
    else if (event.getKeySym() == "s" && event.keyDown())
    {
        pcl::visualization::PCLVisualizer *v = static_cast<pcl::visualization::PCLVisualizer *>(viewer);
        v->saveScreenshot("../images/screenshot_" + std::to_string(screenshot_no) + ".png");
        ++screenshot_no;
    }
}

int main()
{

    std::string filepath = "../data/";

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    std::vector<fs::path> filenames = readFilenamesExt(filepath, ".pcd");

    if (filenames.empty())
    {
        std::cerr << "Could not find data files" << std::endl;
        return EXIT_FAILURE;
    }

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
    viewer.registerKeyboardCallback(&keyboardEventOccurred, &viewer);

    // for segmentation
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr nonground_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> ground_cloud_color_handler(ground_cloud, 212, 212,
                                                                                                212);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> nonground_cloud_color_handler(nonground_cloud, 0,
                                                                                                   255, 0);
    std::string ground_cloud_id = "ground";
    std::string nonground_cloud_id = "nonground";

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

            // Ground segmentation (TODO)
            const auto &segmented_clouds = segmentGround<pcl::PointXYZI>(cloud, 80, 0.2);
            ground_cloud = segmented_clouds.first;
            nonground_cloud = segmented_clouds.second;

            std::cout << "Number of ground points: " << ground_cloud->points.size() << std::endl;
            std::cout << "Number of nonground points: " << nonground_cloud->points.size() << std::endl;

            // Visualise points
            if (first_iteration)
            {
                // viewer.addPointCloud(cloud, cloud_color_handler, point_cloud_id);

                ground_cloud_color_handler.setInputCloud(ground_cloud);
                viewer.addPointCloud(ground_cloud, ground_cloud_color_handler, ground_cloud_id);

                nonground_cloud_color_handler.setInputCloud(nonground_cloud);
                viewer.addPointCloud(nonground_cloud, nonground_cloud_color_handler, nonground_cloud_id);
            }
            else
            {
                // viewer.updatePointCloud(cloud, cloud_color_handler, point_cloud_id);

                ground_cloud_color_handler.setInputCloud(ground_cloud);
                viewer.updatePointCloud(ground_cloud, ground_cloud_color_handler, ground_cloud_id);

                nonground_cloud_color_handler.setInputCloud(nonground_cloud);
                viewer.updatePointCloud(nonground_cloud, nonground_cloud_color_handler, nonground_cloud_id);
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