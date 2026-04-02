#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Core>

class MOTNode
{
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher dynamic_cloud_pub_;
    ros::Publisher mot_tracks_pub_;
    double leaf_size_;
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;

public:
    MOTNode(ros::NodeHandle &nh) : nh_(nh)
    {
        nh_.param("leaf_size", leaf_size_, 0.12);
        nh_.param("cluster_tolerance", cluster_tolerance_, 0.35);
        nh_.param("min_cluster_size", min_cluster_size_, 10);
        nh_.param("max_cluster_size", max_cluster_size_, 5000);

        cloud_sub_ = nh_.subscribe("/livox/lidar", 10, &MOTNode::cloudCallback, this);
        dynamic_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dynamic_cloud", 10);
        mot_tracks_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/mot_tracks", 10);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // VoxelGrid downsampling
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(static_cast<float>(leaf_size_), static_cast<float>(leaf_size_), static_cast<float>(leaf_size_));
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        vg.filter(*cloud_filtered);

        std::vector<pcl::PointIndices> clusters;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(cloud_filtered);

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
        ece.setClusterTolerance(cluster_tolerance_);
        ece.setMinClusterSize(min_cluster_size_);
        ece.setMaxClusterSize(max_cluster_size_);
        ece.setSearchMethod(tree);
        ece.setInputCloud(cloud_filtered);
        ece.extract(clusters);

        // 输出轨迹输入格式: [id, x, y, z, size, id, x, y, z, size, ...]
        std_msgs::Float32MultiArray tracks_msg;
        tracks_msg.data.reserve(clusters.size() * 5);
        int id = 0;
        for (const auto &c : clusters)
        {
            Eigen::Vector3f centroid(0.0f, 0.0f, 0.0f);
            for (const auto idx : c.indices)
            {
                centroid.x() += cloud_filtered->points[idx].x;
                centroid.y() += cloud_filtered->points[idx].y;
                centroid.z() += cloud_filtered->points[idx].z;
            }
            centroid /= static_cast<float>(c.indices.size());

            tracks_msg.data.push_back(static_cast<float>(id++));
            tracks_msg.data.push_back(centroid.x());
            tracks_msg.data.push_back(centroid.y());
            tracks_msg.data.push_back(centroid.z());
            tracks_msg.data.push_back(static_cast<float>(c.indices.size()));
        }

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        output.header = msg->header;
        dynamic_cloud_pub_.publish(output);
        mot_tracks_pub_.publish(tracks_msg);

        ROS_INFO_THROTTLE(1.0, "mot_perception: points=%zu, clusters=%zu", cloud_filtered->size(), clusters.size());
    }
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mot_perception_node");
    ros::NodeHandle nh("~");
    MOTNode node(nh);
    ros::spin();
    return 0;
}
