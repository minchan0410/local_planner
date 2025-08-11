#include <stdio.h>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <vector>
#include <utility>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <erp_driver/erpCmdMsg.h>
#include <erp_driver/erpStatusMsg.h>



// GLOBAL VAR

ros::Publisher path_viz;
ros::Publisher allpath_viz;
ros::Publisher cmd_pub;

static const float wheelbase = 1.2;
static const float deg_max = 30;

double car_x, car_y;
double car_yaw;

bool gridReady = false;
bool gpathReady = false;
nav_msgs::OccupancyGrid grid_;

std::vector<std::pair<double, double>> gpath2loc;
std::vector<nav_msgs::Path> vizPaths;

struct pathPoint
{
    double x;
    double y;
    double yaw;
    double cl; // point 까지 누적 길이
};

double dist(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}


void odom_gps_Callback(const nav_msgs::Odometry::ConstPtr& msg){
    car_x = msg->pose.pose.position.x;
    car_y = msg->pose.pose.position.y;
}

void vehicle_yaw_Callback(const std_msgs::Float32::ConstPtr& msg){
    car_yaw = msg->data;
}

void global_path_Callback(const nav_msgs::Path::ConstPtr& msg){
    gpathReady = true;
    gpath2loc.clear();

    double car_yaw_ros = car_yaw + 1.570796;

    for (const auto& pose_stamped : msg->poses){
        double dx = pose_stamped.pose.position.x - car_x;
        double dy = pose_stamped.pose.position.y - car_y;
        
        double local_x = cos(-car_yaw) * dx - sin(-car_yaw) * dy;
        double local_y = sin(-car_yaw) * dx + cos(-car_yaw) * dy;

        gpath2loc.emplace_back(local_x, local_y);
    }

}

void local_map_Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    grid_ = *msg;
    gridReady = true;
}

// void erp_Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){

// }

bool isObstacleNear(double x, double y, double radius) {
    if (gridReady){
        double res = grid_.info.resolution;
        int width = grid_.info.width;
        int height = grid_.info.height;

        int gx = (x - grid_.info.origin.position.x) / res;
        int gy = (y - grid_.info.origin.position.y) / res;
        int r = radius / res;

        for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                int nx = gx + dx;
                int ny = gy + dy;

                if (nx < 0 || ny < 0 || nx >= width || ny >= height)
                    continue;

                if (std::hypot(dx * res, dy * res) > radius)
                    continue;

                int index = ny * width + nx;
                if (grid_.data[index] == 100)  // threshold
                    return true;
            }
        }
        return false;
    }
    else{
        return false;
    }
}

void vizSelPath(int selIDX){
    nav_msgs::Path selected = vizPaths[selIDX];
    selected.header.stamp = ros::Time::now();
    selected.header.frame_id = "base_link"; // 또는 odom 등 frame에 맞게

    path_viz.publish(selected);
}

void vizAllPaths_loop(const std::vector<bool>& isPathSafe){
    ros::Time now = ros::Time::now();

    for (int i = 0; i < vizPaths.size(); ++i){
        if(isPathSafe[i]){
            nav_msgs::Path path = vizPaths[i];
            path.header.stamp = now;
            path.header.frame_id = "base_link"; // or odom 등 사용 중인 프레임
            path_viz.publish(path);
            ros::Duration(0.02).sleep();  // 약간의 딜레이를 주면 Rviz에서 잘 보임
        }
    }
}

void vizAllPaths() {
    visualization_msgs::MarkerArray marker_array;
    ros::Time now = ros::Time::now();

    for (int i = 0; i < vizPaths.size(); ++i) {
        const auto& path = vizPaths[i];

        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";  // 또는 "odom"
        marker.header.stamp = now;
        marker.ns = "paths";
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.01;

        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        for (const auto& pose_stamped : path.poses) {
            geometry_msgs::Point p;
            p.x = pose_stamped.pose.position.x;
            p.y = pose_stamped.pose.position.y;
            p.z = 0.1;
            marker.points.push_back(p);
        }

        marker_array.markers.push_back(marker);
    }

    allpath_viz.publish(marker_array);
}

void Compute(){

    double m_dist_adj_pts = 0.1; // be a distance between two adjacent points
    double m_center2rear = 0.5;
    double m_velo_e = 9; // 36
    double m_velo_s = 0.9;
    double m_rho = 1.15;
    double v_j = 3.0; // 속도 관련
    double sizeratio = 0.18;
    double q_tentacle = pow((v_j - m_velo_s)/(m_velo_e - m_velo_s), (1.0 / 1.2));// * sizeratio; // 0.095667?? // q_tentacle : paper eq(5)
    double l_outmost = (8 + 33.5 * pow(q_tentacle, 1.2)) * sizeratio;

    double R_outmost = l_outmost / ((0.6 * M_PI) * (1 - pow(q_tentacle, 0.9)));    // Outmost radius of tentacle : paper eq(2)  4.407623
    double dphi_tentacle[81];              // d(phi) of each tentacle
    double l_tentacle[81];                 // tentacle length : paper eq(3),(4)
    double r_tentacle[81];                 // radius of each tentacle: paper eq(3)

    std::vector<double> pathlen2obs(81);

    std::vector<bool> isPathSafe(81);
    std::fill(isPathSafe.begin(), isPathSafe.end(), true); 

    // std::vector<Vector3d> m_TentaclePath[81];
    std::vector<std::vector<pathPoint>> m_TentaclePath(81);


    // auto start = std::chrono::high_resolution_clock::now();


    // 경로 생성은 1ms 이내로 완료됨
    /////////////////////////////////////////////////////////
    // local path generation ////////////////////////////////
    /////////////////////////////////////////////////////////
    r_tentacle[40] = 999999.9;  //infinite, straight_line for tentacle --> wtf...?
    for (int k = 0; k < 41; k++){ // FROM LEFT TO CENTER // 근데 40은 무한대 아닌가? 41->40으로 수정함 3/12

        nav_msgs::Path path;
        path.header.frame_id = "base_link";

        l_tentacle[k] = l_outmost + 20 * sqrt((k + 1)/ 40.0) * sizeratio; // k가 커질수록 l_tentacle[]이 길어짐. floor는 주어진 실수보다 작거나 같은 가장 큰 실수값 반환환
        r_tentacle[k] = pow(m_rho, k) * R_outmost;
        dphi_tentacle[k] = m_dist_adj_pts / abs(r_tentacle[k]); // angle which makes the distance between two adjacent points, 10 cm

        for (int m = 0; m < floor(l_tentacle[k]/m_dist_adj_pts); m++){ // 길이에 따라 for loop 몇번 돌지 결정 floor는 주어진 실수보다 작거나 같은 가장 큰 실수값 반환.

            double p_x = r_tentacle[k] * sin(dphi_tentacle[k] * m) - m_center2rear;
            double p_y = r_tentacle[k] * (1 - cos(dphi_tentacle[k] * m));
            double p_r = dphi_tentacle[k] * m;
            double p_cl = m * 0.1;

            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "base_link";
            pose.pose.position.x = p_x;
            pose.pose.position.y = p_y;
            path.poses.push_back(pose);

            m_TentaclePath[k].emplace_back(pathPoint{p_x, p_y, p_r,p_cl});
        }
        vizPaths[k] = path;
    }
    
    for (int k = 41; k < 81; k++){ // FROM RIGHT TO CENTER

        nav_msgs::Path path;
        path.header.frame_id = "base_link";

        l_tentacle[k] = l_outmost + 20 * sqrt((k - 40) / 40.0) * sizeratio;
        r_tentacle[k] = -pow(m_rho, (k - 41)) * R_outmost;
        dphi_tentacle[k] = m_dist_adj_pts / abs(r_tentacle[k]);    // angle which makes the distance between two adjacent point 10 cm

        for (int m = 0; m < floor(l_tentacle[k]/m_dist_adj_pts); m++){

            double p_x = - r_tentacle[k] * sin(dphi_tentacle[k] * m) - m_center2rear;
            double p_y = r_tentacle[k] * (1 - cos(dphi_tentacle[k] * m));
            double p_r = - dphi_tentacle[k] * m;
            double p_cl = m * 0.1;

            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "base_link";
            pose.pose.position.x = p_x;
            pose.pose.position.y = p_y;
            path.poses.push_back(pose);

            m_TentaclePath[k].emplace_back(pathPoint{p_x, p_y, p_r, p_cl});
        }
        vizPaths[k] = path;
    }
    
    vizAllPaths();

    // std::ofstream file("/home/minchan/obs_control/log/tp.csv");

    // if (!file.is_open()) {
    //     std::cerr << "파일 열기 실패" << std::endl;
    // } else {
    //     file << "angle_index,x,y\n";

    //     for (size_t k = 0; k < m_TentaclePath.size(); ++k) {
    //         for (const auto& pt : m_TentaclePath[k]) {
    //             file << k << "," << pt.x << "," << pt.y << "," << pt.yaw << "," << pt.cl << "\n";
    //         }
    //     }

    //     file.close();
    //     std::cout << "CSV 저장 완료: /home/minchan/tentacle_path.csv" << std::endl;
    // }


    auto start = std::chrono::high_resolution_clock::now();
    // loop time이 너무 길다.
    // 각 경로가 SafePath인지 아닌지 확인

    /////////////////////////////////////////////////////////
    // Get Path Info ////////////////////////////////////////
    /////////////////////////////////////////////////////////
    for (int k = 0; k < 81; k++) {
        bool safeFlag = true;
        for (const auto& path_point : m_TentaclePath[k]){
            double p_x_rear = path_point.x;
            double p_y_rear = path_point.y;
            double pyaw = path_point.yaw;
            double c_l = path_point.cl;

            double p_x_front = p_x_rear + 2 * m_center2rear * cos(pyaw); //2 * center2rear == wheelbase
            double p_y_front = p_y_rear + 2 * m_center2rear * sin(pyaw);

            double check_r = 0.6; //차량 가로 길이 : 1.2m

            bool front_check = false;
            bool rear_check = false;
            
            if (gridReady == true){
                front_check = isObstacleNear(p_x_front - 0.82, p_y_front, check_r); // base link와 velodyne 기준 좌표계는 x축 기준 0.82 차이남. 좌표계 매칭.
                rear_check  = isObstacleNear(p_x_rear - 0.82, p_y_rear, check_r);
            }
            else{
                ROS_INFO("OccGrid does not received YET!");
            }
            
            if (front_check == true || rear_check == true){
                safeFlag = false;
                isPathSafe[k] = false;  // safepath가 아님으로 변경
                pathlen2obs[k] = c_l; // 장애물까지의 거리
                break;
            }
        }
        if (safeFlag == true){
            pathlen2obs[k] = m_TentaclePath[k].back().cl; // safepath이면 마지막 누적거리가 장애물까지의 거리
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // ROS_INFO("loop time : %ldms",duration_ms.count());

    // vizAllPaths_loop(isPathSafe);

    // for(int k = 0; k < 81; k++){
    //     if(isPathSafe[k]){
    //         printf("%d ", k);
    //     }
    // }
    // printf("\n");

    /////////////////////////////////////////////////////////
    // Path choosing Algorithm //////////////////////////////
    /////////////////////////////////////////////////////////

    bool all_safe = std::all_of(isPathSafe.begin(), isPathSafe.end(), [](bool v) { return v; });   // vector가 모두 true인지
    bool all_danger = std::none_of(isPathSafe.begin(), isPathSafe.end(), [](bool v) { return v; }); // vector가 모두 false인지

    int selIDX = 1;

    // -----------------------------------------------------------------
    // std::ofstream file("/home/minchan/obs_control/log/pathloc.csv", std::ios::app);

    // if (!file.is_open()) {
    //     std::cerr << "파일 열기 실패" << std::endl;
    // } else {
    //     file << selIDX << ",";
    //     for (size_t k = 0; k < gpath2loc.size(); ++k) {
    //         file << gpath2loc[k].first << "," << gpath2loc[k].second;
    //         if (k != gpath2loc.size() - 1) {
    //             file << ",";
    //         }
    //     }
    //     file << "\n";
    //     file.close();
    //     std::cout << "csv saved.csv" << std::endl;
    // }

     // -----------------------------------------------------------------


    if (all_safe){
        std::vector<double> path_tracking_score(81);
        if(gpathReady){ // global path가 callback 되면
            for(int k = 0; k < 81; k++){
                if(isPathSafe[k]){
                    double lp_x = m_TentaclePath[k].back().x;
                    double lp_y = m_TentaclePath[k].back().y;
                    double min_d = 100;
                    for(const auto& gp : gpath2loc){
                        double d = dist(gp.first, gp.second, lp_x, lp_y);
                        if (d < min_d){
                            min_d = d;
                        }
                    }
                    path_tracking_score[k] = min_d;
                }
                else{
                    path_tracking_score[k] = -100; // not safe
                }
            }
        }
        else{
            ROS_INFO("Global path not received");
        }
        //Normalize
        double max_pt = 100;
        double min_pt = -100;
        for(int k = 0; k < 81; k++){
            double pt_score = path_tracking_score[k];
            if(pt_score > 0){
                if (pt_score < min_pt) min_pt = pt_score;
                if (pt_score > max_pt) max_pt = pt_score;
            }
        }
        for(int k = 0; k < 81; k++){
            double pt_score = path_tracking_score[k];
            if(pt_score > 0){
                double norm_score = 1 - (path_tracking_score[k] - min_pt) / (max_pt - min_pt);
                path_tracking_score[k] = norm_score; 
            }
        }

        auto max_iter = std::max_element(path_tracking_score.begin(), path_tracking_score.end());
        selIDX = std::distance(path_tracking_score.begin(), max_iter);
        // ROS_INFO("path : %d", selIDX);
    }
    else if (all_danger){ // 모두 safe path가 아닐 때
        std::vector<double> path_len_score(81);

        for(int k = 0; k < 81; k++){
            if(isPathSafe[k]){
                path_len_score[k] = -100;
            }
            else{ // maybe all this case
                pathlen2obs[k] = path_len_score[k];
            }
        }

        auto max_iter = std::max_element(path_len_score.begin(), path_len_score.end());
        selIDX = std::distance(path_len_score.begin(), max_iter);
    }
    else{
        //safe & not safe 섞여 있는 경우 safe path들에 대해서만 처리
        std::vector<double> path_tracking_score(81); // 초기화 안하면 0.0으로 자동 초기화
        std::vector<double> path_len_score(81);
        std::vector<double> total_score(81);


        // -- path traking score -- //
        // 가장 마지막 점이 경로와 얼마나 가까운지를 확인
        if(gpathReady){ // global path가 callback 되면
            for(int k = 0; k < 81; k++){
                if(isPathSafe[k]){
                    double lp_x = m_TentaclePath[k].back().x;
                    double lp_y = m_TentaclePath[k].back().y;
                    double min_d = 100;
                    for(const auto& gp : gpath2loc){
                        double d = dist(gp.first, gp.second, lp_x, lp_y);
                        if (d < min_d){
                            min_d = d;
                        }
                    }
                    path_tracking_score[k] = min_d;
                }
                else{
                    path_tracking_score[k] = -100; // not safe
                }
            }
        }
        else{
            ROS_INFO("Global path not received");
        }
        //Normalize
        double max_pt = 100;
        double min_pt = -100;
        for(int k = 0; k < 81; k++){
            double pt_score = path_tracking_score[k];
            if(pt_score > 0){
                if (pt_score < min_pt) min_pt = pt_score;
                if (pt_score > max_pt) max_pt = pt_score;
            }
        }
        for(int k = 0; k < 81; k++){
            double pt_score = path_tracking_score[k];
            if(pt_score > 0){
                double norm_score = 1 - (path_tracking_score[k] - min_pt) / (max_pt - min_pt);
                path_tracking_score[k] = norm_score; 
            }
        }
        // ------------------------------------------------------------------------//
        // -- path len score -- // 
        for(int k = 0; k < 81; k++){
            if (isPathSafe[k]){
                path_len_score[k] = m_TentaclePath[k].back().cl;
            }
            else{
                path_len_score[k] = -100;
            }
        }
        // normalize
        double max_pl = 100;
        double min_pl = -100;
        for(int k = 0; k < 81; k++){
            double pl_score = path_len_score[k];
            if(pl_score > 0){
                if(pl_score < min_pl) min_pl = pl_score;
                if(pl_score > max_pl) max_pl = pl_score;
            }
        }
        for(int k = 0; k < 81; k++){
            double pl_score = path_len_score[k];
            if(pl_score > 0){
                double norm_score = (path_len_score[k] - min_pl) / (max_pl - min_pl);
                path_len_score[k] = norm_score; 
            }
        }

        // merge two score

        double pl_gain = 0.7;           // k
        double pt_gain = 1 - pl_gain;   // 1 - k

        for(int k = 0; k < 81; k++){
            if (isPathSafe[k]){
                total_score[k] = pl_gain * path_len_score[k] + pt_gain * path_tracking_score[k];
            }
            else{
                total_score[k] = -100;
            }
        }

        auto max_iter = std::max_element(path_tracking_score.begin(), path_tracking_score.end());
        selIDX = std::distance(path_tracking_score.begin(), max_iter);
        // ROS_INFO("case3 path : %d", selIDX);
    }


    // selIDX --> steering
    double steering = 0;
    int pathlen = static_cast<int>(m_TentaclePath[selIDX].size());
    int halflen = pathlen / 2;
    steering = m_TentaclePath[selIDX][halflen].yaw;

    ROS_INFO("steering : %.2f", steering);

    erp_driver::erpCmdMsg command;
    command.speed = 40;
    

    // uint8 gear
    // uint8 speed
    // int32 steer
    // uint8 brake
    
}


int main(int argc, char** argv){
    
    ros::init(argc, argv, "local_planner");
    ros::NodeHandle nh_;


    // nh_.getParam("wheelbase", m_safe_region);
    // nh_.getParam()

    // SUBSCRIBER
    ros::Subscriber odom_gps_Sub = nh_.subscribe("/odom_gps", 10, odom_gps_Callback);
    ros::Subscriber vehicle_yaw_Sub = nh_.subscribe("/vehicle_yaw", 10, vehicle_yaw_Callback);
    ros::Subscriber global_path_Sub = nh_.subscribe("/global_path", 10, global_path_Callback);
    ros::Subscriber local_map_Sub = nh_.subscribe("/local_map", 10, local_map_Callback);
    ros::Subscriber local_map_Sub = nh_.subscribe("/erp42_status", 10, erp_Callback);

    path_viz = nh_.advertise<nav_msgs::Path>("path_viz", 1);
    allpath_viz = nh_.advertise<visualization_msgs::MarkerArray>("allpath_viz", 1);
    cmd_pub = nh_.advertise<erp_driver::erpCmdMsg>("/erp42_ctrl_cmd");

    vizPaths.resize(81);

    ros::Rate rate(100);
    while (ros::ok()){
        ros::spinOnce();
        auto start = std::chrono::high_resolution_clock::now();
        
        Compute();


        // ROS_INFO("running");
        // auto end = std::chrono::high_resolution_clock::now();
        // auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        // ROS_INFO("loop time : %ldms",duration_ms.count());
        rate.sleep();
    }

    // PUBLISHER

}