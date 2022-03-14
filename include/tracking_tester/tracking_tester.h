#ifndef TRACKING_TESTER
#define TRACKING_TESTER

#include <ros/ros.h>
#include <typeinfo>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_msgs/TFMessage.h>

#include <camera_apps_msgs/ObjectInfo.h>
#include <camera_apps_msgs/ObjectsInfo.h>

#include "hungarian.h"

class TrackingTester
{
    public:
        TrackingTester(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        ~TrackingTester();
    private:
        void tf_callback(const tf2_msgs::TFMessageConstPtr &tf_msg);
        void objects_callback(const camera_apps_msgs::ObjectsInfoConstPtr &objects_msg);

        bool search_tf(ros::Time time, tf2_msgs::TFMessage& synchronized_tf);
        std::vector<std::pair<double, double>> create_gt_poses(tf2_msgs::TFMessage tf);
        std::vector<std::vector<double>> create_cost_mat(camera_apps_msgs::ObjectsInfo objects_info,
                std::vector<std::pair<double, double>> gt_poses);
        std::vector<std::vector<double>> create_transpose_mat(std::vector<std::vector<double>> mat);
        std::vector<std::vector<int>> create_map(std::vector<std::vector<double>> cost_mat, camera_apps_msgs::ObjectsInfo objects_info);
        double calc_error(std::pair<double, double> gt_pose, camera_apps_msgs::ObjectInfo object_info);
        void count_switch(std::vector<std::vector<int>> map);

        int tf_msg_queue_size_;
        double time_gap_threshold_;
        int person_num_;
        double mapping_error_th_;

        bool synchro_flag_;
        int FP_sum_ = 0;
        int FN_sum_ = 0;
        int switch_sum_ = 0;
        int person_sum_ = 0;
        int map_sum_ = 0;
        double error_sum_ = 0;
        std::vector<std::vector<int>> before_map_;
        bool first_map_flag_ = false;
        double max_error_ = 0;

        std::vector<tf2_msgs::TFMessage> tf_msgs_;

        ros::Subscriber tf_sub_;
        ros::Subscriber objects_sub_;
};

#endif 
