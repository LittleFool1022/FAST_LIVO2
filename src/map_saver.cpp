#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <ctime>
#include <filesystem>

class MapSaver {
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber map_sub_;
    std::string save_dir_;
    std::string cloud_topic_;
    std::string map_topic_;
    bool save_cloud_;
    bool save_map_;
    bool has_saved_cloud_;
    bool has_saved_map_;
    ros::Time last_cloud_time_;
    ros::Time last_map_time_;
    bool cloud_received_;
    bool map_received_;
    sensor_msgs::PointCloud2::ConstPtr last_cloud_;
    nav_msgs::OccupancyGrid::ConstPtr last_map_;

public:
    MapSaver() : save_cloud_(false), save_map_(false),
                 has_saved_cloud_(false), has_saved_map_(false),
                 cloud_received_(false), map_received_(false) {
        // Read configuration from parameter server
        std::string default_dir = std::string(getenv("HOME")) + "/catkin_ws/maps";
        nh_.param("save_dir", save_dir_, default_dir);
        nh_.param("save_cloud", save_cloud_, true);
        nh_.param("save_map", save_map_, true);
        nh_.param("cloud_topic", cloud_topic_, std::string("/livox/lidar"));
        nh_.param("map_topic", map_topic_, std::string("/map"));

        // Create save directory
        int ret = system(("mkdir -p " + save_dir_).c_str());
        if (ret != 0) {
            ROS_WARN("Failed to create directory: %s", save_dir_.c_str());
        }

        // Subscribe to point cloud and map topics
        cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &MapSaver::cloudCallback, this);
        map_sub_ = nh_.subscribe(map_topic_, 1, &MapSaver::mapCallback, this);

        ROS_INFO("Map saver node started, save directory: %s", save_dir_.c_str());
        ROS_INFO("Subscribing to cloud topic: %s", cloud_topic_.c_str());
        ROS_INFO("Subscribing to map topic: %s", map_topic_.c_str());
        ROS_INFO("Will automatically save map and cloud on first receive.");

        // Create timer to check topic status
        ros::Timer check_timer = nh_.createTimer(ros::Duration(1.0), &MapSaver::checkTopics, this);
    }

    void checkTopics(const ros::TimerEvent& event) {
        if (!cloud_received_ && save_cloud_) {
            ROS_WARN("No point cloud data received. Please ensure FAST-LIVO2 is running and publishing to topic: %s", cloud_topic_.c_str());
        }
        if (!map_received_ && save_map_) {
            ROS_WARN("No grid map data received. Please ensure map server is running and publishing to topic: %s", map_topic_.c_str());
        }
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (!save_cloud_) return;
        cloud_received_ = true;
        last_cloud_time_ = ros::Time::now();
        last_cloud_ = msg;  // Store the latest cloud
        if (has_saved_cloud_) return;  // Only save if not already saved
        // Convert point cloud message to PCL format
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);
        // Generate filename with timestamp
        std::string filename = save_dir_ + "/cloud_" + getTimestamp() + ".pcd";
        // Save point cloud file
        if (pcl::io::savePCDFileBinary(filename, *cloud) == 0) {
            ROS_INFO("Point cloud map saved to: %s", filename.c_str());
            has_saved_cloud_ = true;
        } else {
            ROS_ERROR("Failed to save point cloud map");
        }
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        if (!save_map_) return;
        map_received_ = true;
        last_map_time_ = ros::Time::now();
        last_map_ = msg;  // Store the latest map
        if (has_saved_map_) return;  // Only save if not already saved
        // Generate filename with timestamp
        std::string filename = save_dir_ + "/map_" + getTimestamp() + ".yaml";
        // Save map file
        if (saveMapToFile(*msg, filename)) {
            ROS_INFO("Grid map saved to: %s", filename.c_str());
            has_saved_map_ = true;
        } else {
            ROS_ERROR("Failed to save grid map");
        }
    }

private:
    std::string getTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
        return ss.str();
    }

    bool saveMapToFile(const nav_msgs::OccupancyGrid& map, const std::string& filename) {
        try {
            // Save YAML file
            std::ofstream yaml_file(filename);
            if (!yaml_file.is_open()) {
                ROS_ERROR("Failed to open YAML file for writing: %s", filename.c_str());
                return false;
            }

            // Generate PGM filename
            std::string pgm_filename = filename.substr(0, filename.find_last_of('.')) + ".pgm";

            // Write YAML header
            yaml_file << "image: " << pgm_filename << "\n";
            yaml_file << "resolution: " << map.info.resolution << "\n";
            yaml_file << "origin: [" << map.info.origin.position.x << ", "
                     << map.info.origin.position.y << ", " << map.info.origin.position.z << "]\n";
            yaml_file << "negate: 0\n";
            yaml_file << "occupied_thresh: 0.65\n";
            yaml_file << "free_thresh: 0.196\n";

            yaml_file.close();

            // Save PGM file
            std::ofstream pgm_file(pgm_filename, std::ios::binary);
            if (!pgm_file.is_open()) {
                ROS_ERROR("Failed to open PGM file for writing: %s", pgm_filename.c_str());
                return false;
            }

            // Write PGM header
            pgm_file << "P5\n";
            pgm_file << map.info.width << " " << map.info.height << "\n";
            pgm_file << "255\n";

            // Write map data
            for (unsigned int y = 0; y < map.info.height; y++) {
                for (unsigned int x = 0; x < map.info.width; x++) {
                    unsigned int i = x + (map.info.height - y - 1) * map.info.width;
                    if (map.data[i] == -1) {
                        pgm_file << (unsigned char)205;
                    } else if (map.data[i] == 0) {
                        pgm_file << (unsigned char)254;
                    } else if (map.data[i] == 100) {
                        pgm_file << (unsigned char)0;
                    } else {
                        pgm_file << (unsigned char)205;
                    }
                }
            }

            pgm_file.close();
            return true;
        } catch (const std::exception& e) {
            ROS_ERROR("Exception while saving map: %s", e.what());
            return false;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_saver");
    MapSaver map_saver;
    ros::spin();
    return 0;
} 