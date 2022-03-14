#include <tracking_tester/tracking_tester.h>

TrackingTester::TrackingTester(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    pnh.param("tf_msg_queue_size", tf_msg_queue_size_, 100);
    pnh.param("time_gap_threshold", time_gap_threshold_, 0.02);
    pnh.param("person_num", person_num_, 3);
    pnh.param("mapping_error_th", mapping_error_th_, 0.5);

    tf_sub_ = nh.subscribe("/tf", 10, &TrackingTester::tf_callback, this);
    objects_sub_ = nh.subscribe("/tracking_result", 10, &TrackingTester::objects_callback, this);

    for(int i=0; i<person_num_; i++){
        before_map_.push_back({-1, -1});
    }
}

TrackingTester::~TrackingTester()
{
}
void TrackingTester::tf_callback(const tf2_msgs::TFMessageConstPtr &tf_msg)
{
    tf_msgs_.push_back(*tf_msg);
    if(tf_msgs_.size() > tf_msg_queue_size_){
        tf_msgs_.erase(tf_msgs_.begin());
    }
}

void TrackingTester::objects_callback(const camera_apps_msgs::ObjectsInfoConstPtr &objects_msg)
{

    // std::cout << "start" << std::endl;
    // if(objects_msg->objects_info.size() == 0){
    //     FN_sum_ += person_num_;
    //     person_sum_ += person_num_;
    //     return;
    //
    // }

    tf2_msgs::TFMessage synchronized_tf;
    bool synchro_flag = search_tf(objects_msg->header.stamp, synchronized_tf);
    if(!synchro_flag) return;
    if(synchronized_tf.transforms.size() != person_num_ + 1){
        // std::cout << "mocap num" << synchronized_tf.transforms.size() << std::endl;
        // std::cout << "mocap data num is wrong" << std::endl;

        return;
    }
    else{
        if(objects_msg->objects_info.size() == 0){
            FN_sum_ += person_num_;
            person_sum_ += person_num_;
            return;

        }

        auto gt_poses = create_gt_poses(synchronized_tf);
        auto cost_mat = create_cost_mat(*objects_msg, gt_poses);
        std::vector<std::vector<int>> map = create_map(cost_mat, *objects_msg);
        for(int i=0; i<map.size(); i++){
            if(map[i][0] == -1) continue;
            double error = calc_error(gt_poses[i], objects_msg->objects_info[map[i][1]]);
            if(error > mapping_error_th_){
                FP_sum_++;
                FN_sum_++;
                std::cout << "trash map!" << std::endl;
                std::cout << "error " << error << std::endl;
            }
            else{
                error_sum_ += error;
                map_sum_++;

                if(max_error_ < error) max_error_ = error;
            }
        }
        // if(first_map_flag_) count_switch(map);
        // before_map_ = map;
        count_switch(map);
        first_map_flag_ = true;
        person_sum_ += person_num_;

        double MOTA = 1 - ((FP_sum_ + FN_sum_ + switch_sum_) / (double)person_sum_);
        double MOTP = error_sum_ / map_sum_;
        std::cout << "FP: " << FP_sum_;
        std::cout << " FN: " << FN_sum_;
        std::cout << " switch_id: " << switch_sum_;
        std::cout << " person_sum_: " << person_sum_;
        std::cout << " map_sum_: " << map_sum_;
        // std::cout << " error_sum_: " << error_sum_;
        std::cout << " MOTA: " << MOTA;
        std::cout << " MOTP: " << MOTP;
        std::cout << " max error: " << max_error_;
        std::cout << std::endl;
    }
}

bool TrackingTester::search_tf(ros::Time time, tf2_msgs::TFMessage& synchronized_tf)
{
    for(const auto& tf_msg: tf_msgs_){
        ros::Duration gap = tf_msg.transforms[0].header.stamp - time;
        if(std::abs(gap.toSec()) <= time_gap_threshold_){
            synchronized_tf = tf_msg;
            return true;
        }
    }
    // std::cout << "cannot search synchronized tf" << std::endl;
    ros::Duration gap_begin = tf_msgs_[0].transforms[0].header.stamp - time;
    ros::Duration gap_back = tf_msgs_.back().transforms[0].header.stamp - time;

    return false;
}

std::vector<std::pair<double, double>> TrackingTester::create_gt_poses(tf2_msgs::TFMessage tf)
{
    std::vector<std::pair<double, double>> gt_poses;
    double world_to_camera_x = tf.transforms[0].transform.translation.x;
    double world_to_camera_y = tf.transforms[0].transform.translation.y;

    for(int i=1; i<tf.transforms.size(); i++){
        double x = tf.transforms[i].transform.translation.x;
        double y = tf.transforms[i].transform.translation.y;

        gt_poses.push_back(std::make_pair(x - world_to_camera_x, y - world_to_camera_y));
    }
    return gt_poses;
}

std::vector<std::vector<double>> TrackingTester::create_cost_mat(camera_apps_msgs::ObjectsInfo objects_info,
                std::vector<std::pair<double, double>> gt_poses)
{
    std::vector<std::vector<double>> cost_mat;
    for(const auto& object_info: objects_info.objects_info){
        std::vector<double> cost_row;
        for(const auto& gt_pose: gt_poses){
            cost_row.push_back(calc_error(gt_pose, object_info));
        }
        cost_mat.push_back(cost_row);
    }
    // for(const auto& row: cost_mat){
    //     for(const auto& num: row){
    //         std::cout << std::setw(5) << num << " ";
    //     }
    //     std::cout << std::endl;
    // }
    // std::cout << std::endl;
    return cost_mat;
}

std::vector<std::vector<double>> TrackingTester::create_transpose_mat(std::vector<std::vector<double>> mat)
{
    std::vector<std::vector<double>> transpose_mat(mat[0].size(), std::vector<double>(mat.size()));
    for(int i=0; i<mat.size(); i++){
        for(int j=0; j<mat[0].size(); j++){
            transpose_mat[j][i] = mat[i][j];
        }
    }
    return transpose_mat;
}

std::vector<std::vector<int>> TrackingTester::create_map(std::vector<std::vector<double>> cost_mat, camera_apps_msgs::ObjectsInfo objects_info)
{
    auto object_list = objects_info.objects_info;
    bool transpose_flag = false;
    if(cost_mat.size() < cost_mat[0].size()){
    // if(object_list.size() < person_num_){
        cost_mat = create_transpose_mat(cost_mat);
        transpose_flag = true;
        FN_sum_ += cost_mat.size() - cost_mat[0].size();
        // FN_sum_ += person_num_ - object_list.size();
    }
    else{
        FP_sum_ += object_list.size() - person_num_;
    }

    Hungarian<double> hungarian(cost_mat);
    std::vector<int> alloc = hungarian.solve().second;

    //mapのインデックスはgtのインデックスに対応，１列目の要素がトラッキングID，２列目の要素がObjectsInfoでのインデックスを表す．
    //
    std::vector<std::vector<int>> map(person_num_, std::vector<int>(2, -1));
    // std::vector<std::vector<int>> map(cost_mat[0].size(), std::vector<int>(2, -1));
    // for(const auto& row: map){
    //     for(const auto& num: row){
    //         std::cout << num << " ";
    //     }
    //     std::cout << std::endl;
    // }
    std::cout << std::endl;
    for(int i=0; i<alloc.size(); i++){
        if(alloc[i] != -1){
            if(!transpose_flag){
                map[alloc[i]][0] = object_list[i].id;
                map[alloc[i]][1] = i;
            }
            if(transpose_flag){
                map[i][0] = object_list[alloc[i]].id;
                map[i][1] = alloc[i];
            }
        }
    }
    
    // std::cout << "mat" << std::endl;
    // for(const auto& row: map){
    //     for(const auto& num: row){
    //         std::cout << num << " ";
    //     }
    //     std::cout << std::endl;
    // }
    return map;
}
double TrackingTester::calc_error(std::pair<double, double> gt_pose, camera_apps_msgs::ObjectInfo object_info)
{
    double dist_x = gt_pose.first - object_info.centroid.x;
    double dist_y = gt_pose.second - object_info.centroid.y;

    return std::sqrt(dist_x * dist_x + dist_y * dist_y);
}

void TrackingTester::count_switch(std::vector<std::vector<int>> map)
{
    for(int i=0; i<map.size(); i++){
        if(map[i][0] != -1 && before_map_[i][0] != -1 && map[i][0] != before_map_[i][0]){
            switch_sum_++;
        }
        if(map[i][0] != -1){
            before_map_[i] = map[i];
        }
    }
}
