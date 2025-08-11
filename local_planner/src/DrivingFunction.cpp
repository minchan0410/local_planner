#include <boost/timer/timer.hpp>
#include <boost/thread/thread.hpp>
#include <vector>
#include <Eigen/Core>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h> //VIZVEHICLE
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <algorithm>
#include <iostream>
#include <stdio.h>
#include <string>
#include <fstream>
#include <stdlib.h>
#include <time.h>
#include <cmath>
#include <sstream>
#include <chrono>
#include <ctime> 

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;
using std::ifstream;

#define KMpH2MpS 0.277777
#define Mps2KMpH 3.6

#define MIN(x,y) ((x>y) ? y : x)
#define MAX(x,y) ((x>y) ? x : y)
#define SIGN(x) ((x >= 0) ? 1 : -1)
#define DISTANCE(x1, y1, x2, y2) sqrt((x1 - x2)*(x1 - x2) +(y1 - y2)*(y1 - y2))

#define _RAD2DEG 180 / M_PI
#define _DEG2RAD M_PI / 180

#define WHEEL_BASE  2.700 // CARLA: 2.7359668, GENESIS G80: 3.009, IONIC: 2.700
#define STEERING_RATIO 17.5
#define MAX_STEERING 480.0
#define WATCHMILE_MODE false

#define MIN_VEL_INPUT 3.0 // 3.0 
#define MAX_VEL_INPUT 8.0 // 8.0 for tentacle

#define INF 0x3f

struct CARPOSE {
    double x, y, th, vel;
};
CARPOSE m_car;

double m_safe_region, m_collision_radius, m_pointsafe_region1, m_pointsafe_region2, m_sidesafe_region, m_center2rear;
double m_rho, m_velo_e, m_velo_s; // Tentacle parameters

ros::Subscriber Sub_occMap, Sub_distanceMap;  //add
ros::Subscriber Sub_localization, Sub_NaviInfo;

ros::Publisher Pub_collision_check;
ros::Publisher pub_intersectFlag;
ros::Publisher Pub_tentacle, Pub_chosenPath, Pub_chosenPathbyValue; // tentacle add
ros::Publisher pub_control;
ros::Publisher Pub_obstacleLocal;

ros::Publisher pub_trj;
ros::Publisher Pub_tem;
ros::Publisher Pub_tem2;

ros::Publisher Pub_Occ2Global;

////////////////////////////////////////////////////
//DK ==========================
ros::Subscriber Sub_PedInfo;
ros::Publisher Pub_text_marker;
ros::Publisher Pub_pedestrian_normal;
ros::Publisher Pub_pedestrian_caution;
/////////////////////////////////////////////////////

ros::Subscriber Sub_globalPath;
nav_msgs::Path trajectory;

ros::Time m_ros_time;
double limitDeg = 180.0;
double m_pix2meter = 0.03;

std_msgs::Header m_header;
geometry_msgs::PoseArray collision_edge;
nav_msgs::OccupancyGrid::Ptr m_distance_map;
bool** HeightMap;
double** DistanceMap;
typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
VPointCloud m_obstaclePtCloudLocal;

// Tentacle related
double m_Steer_cmd;
bool m_distMap_flag;
int m_distance_to_intersection;
int m_navi_info;
bool m_at_intersection;
int m_discriminate_range, m_discriminate_range_lr;
// bpp; get_watchmile_mode;
bool m_watchmile_mode;
double m_inter_threshold{5.0};

bool ego_vehicle_pose_flag{false};
double m_data_keeping_time{1.0};

// MQTT Peds Info



////////////////////DK pedestrian info/////////////////// ==========================
//x는 아래방향, y는 오른쪽방향의 좌표. id는 보행자 식별
//caution은 보행자 식별 기능을 사용할 때(m_use_pedestrain_tracking == true) 거리가 가까워지는 보행자에 한해서 유의대상으로 지정
#define PED_TRACK_THRESHOLD 1.0
#define MAX_PEDS 20
#define PEDESTRIAN_WARNING_THRESHOLD 32
#define ACCEL_PER_LOOP 0.1  //spin rate를 정확히 알지 못해 10Hz로 가정해서 구한 loop당 가속도

double m_Velocity_cmd;
double m_prev_vel;

struct PedPose{
    double x, y;
    int id=-1;
    bool caution=false;
    ros::Time detect_time{ros::Time::now()};

    double dist() {
        return DISTANCE(x,y,0,0);
    }
    double dist(PedPose other) {
        return DISTANCE(x,y,other.x,other.y);
    }
    
};


struct VehPose{
    double x = 0.0, y = 0.0;
    double spd = 0.0, th = 0.0;
};

VehPose m_ego;
bool m_use_pedestrian_tracking{false};
std::vector<PedPose> m_peds; //, prev_peds;
bool id_occupancy[MAX_PEDS]={};
double nearest;

// vehicle coordinats with respect to the global coord 
double m_vehicle_x_global = 0.0;
double m_vehicle_y_global = 0.0;
double m_vehicle_th_global = 0.0;

visualization_msgs::Marker text_overlay;
geometry_msgs::PoseArray normal_pedestrian;

////////////////////////////////////////////////////////// ==========================

ros::Time ros_time;

visualization_msgs::MarkerArray marker_array_vehicle_;

vector<double> linspace(double a, double b, int num)
{
    // create a vector of length num
    vector<double> v(num);
    double tmp = 0.0;
    
    // now assign the values to the vector
    for (int i = 0; i < num; i++)
    {
        v[i] = a + i * ( (b - a) / (double)num );
    }
    return v;
}

void Local2Global(double Lx, double Ly, double &gX, double &gY) {
    gX = m_car.x + (Lx * cos(m_car.th) - Ly * sin(m_car.th));
    gY = m_car.y + (Lx * sin(m_car.th) + Ly * cos(m_car.th));
}


vector<double> linspace(double a, double b, int num, bool include_end)
{
    // create a vector of length num
    vector<double> v(num);
    double tmp = 0.0;
    
    // now assign the values to the vector
    for (int i = 0; i < num; i++)
    {
        v[i] = a + i * ( (b - a) / (double)(num - 1) );
    }
    return v;
}

void collision_range(double x, double y, double theta) {
    std_msgs::Header m_header; 
    m_header.stamp = ros::Time::now();
    m_header.frame_id = "map";

    collision_edge.header = m_header;        
    std::vector<double> angle_set = linspace(0.0, 2.0 * M_PI, 20);
    geometry_msgs::PoseStamped _poseStamped;
    // Large Circle
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x + m_safe_region * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y + m_safe_region * sin(angle_set[i]);    //y_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    // Large Circle1
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x + m_collision_radius*cos(theta) + m_safe_region * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y + m_collision_radius*sin(theta) + m_safe_region * sin(angle_set[i]);    //y_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    // Large Circle2
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x + 2.0 * m_collision_radius*cos(theta) + m_safe_region * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y + 2.0 * m_collision_radius*sin(theta) + m_safe_region * sin(angle_set[i]);    //y_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }        

    // point circle
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x - 0.719*cos(theta) - 0.9175*sin(theta) + m_pointsafe_region1 * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y - 0.719*sin(theta) + 0.9175*cos(theta) + m_pointsafe_region1 * sin(angle_set[i]);    //y_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x - 0.719*cos(theta) + 0.9175*sin(theta) + m_pointsafe_region1 * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y - 0.719*sin(theta) - 0.9175*cos(theta) + m_pointsafe_region1 * sin(angle_set[i]);    //y_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x + 2*m_collision_radius*cos(theta) + 0.735*cos(theta) - 0.9175*sin(theta) + m_pointsafe_region2 * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y + 2*m_collision_radius*sin(theta) + 0.735*sin(theta) + 0.9175*cos(theta) + m_pointsafe_region2 * sin(angle_set[i]);    //x_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x + 2*m_collision_radius*cos(theta) + 0.735*cos(theta) + 0.9175*sin(theta) + m_pointsafe_region2 * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y + 2*m_collision_radius*sin(theta) + 0.735*sin(theta) - 0.9175*cos(theta) + m_pointsafe_region2 * sin(angle_set[i]);    //x_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x + m_collision_radius*cos(theta) - 0.6593*cos(theta) - 0.8587*sin(theta) + m_sidesafe_region * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y + m_collision_radius*sin(theta) - 0.6593*sin(theta) + 0.8587*cos(theta) + m_sidesafe_region * sin(angle_set[i]);    //x_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x + m_collision_radius*cos(theta) - 0.6593*cos(theta) + 0.8587*sin(theta) + m_sidesafe_region * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y + m_collision_radius*sin(theta) - 0.6593*sin(theta) - 0.8587*cos(theta) + m_sidesafe_region * sin(angle_set[i]);    //x_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x + m_collision_radius*cos(theta) + 0.6593*cos(theta) - 0.8587*sin(theta) + m_sidesafe_region * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y + m_collision_radius*sin(theta) + 0.6593*sin(theta) + 0.8587*cos(theta) + m_sidesafe_region * sin(angle_set[i]);    //x_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x + m_collision_radius*cos(theta) + 0.6593*cos(theta) + 0.8587*sin(theta) + m_sidesafe_region * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y + m_collision_radius*sin(theta) + 0.6593*sin(theta) - 0.8587*cos(theta) + m_sidesafe_region * sin(angle_set[i]);    //x_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    Pub_collision_check.publish(collision_edge);
    collision_edge.poses.clear();
}

void CallbackLocalizationData(const std_msgs::Float32MultiArray::ConstPtr& msg) { // m_car_idx becomes the center of the rear axle
    // std::cout << "callback CallbackLocalizationData start" << std::endl;
    m_car.x = msg->data.at(0);      // x
    m_car.y = msg->data.at(1);      // y
    m_car.th = msg->data.at(2);     // theta
    m_car.vel = msg->data.at(3)*KMpH2MpS;    // to [m/s]

    // std::cout << "callback CallbackLocalizationData end" << std::endl;
}
bool  DRIVE_FLAG = true;

// Navi Info. consists of {distance_to_the_next_corner, navi_dir, x, y, orientation, velocity} with respect to the DYROS frame
void CallbackNaviInfo(std_msgs::Float32MultiArrayPtr msg) {
    // std::cout << "callback CallbackNaviInfo start" << std::endl;
	// printf("PLEASE\nPLEASE\nPLEASE\nPLEASE\nPLEASE\nPLEASE\nPLEASE\nPLEASE\nPLEASE\nPLEASE\nPLEASE\nPLEASE\n");

    if (msg->data.size() >= 6 && msg->data.size() != 2) {

        // std::cout << "m_distance_to_intersection" <<msg->data.at(0)<<"msg->data.at(1)"<<msg->data.at(1)<<std::endl;
        ego_vehicle_pose_flag = true;
        DRIVE_FLAG = false;
        //m_distance_to_intersection = msg->data.at(0);
        m_navi_info = (msg->data.at(0) == 0.0 && msg->data.at(1) == 1) ? m_navi_info: msg->data.at(1);
        ROS_INFO("nav info1 : %d",m_navi_info);
        m_at_intersection = (m_distance_to_intersection > m_inter_threshold) ? false : true; // if you want to test with watchmile
        
        m_ego.x = msg->data.at(2) * m_pix2meter; // Change to Meter
        m_ego.y = msg->data.at(3) * m_pix2meter;
        m_ego.th = msg->data.at(4) * _DEG2RAD; // Change to Radian
        m_ego.spd = msg->data.at(5);

        // cout << "m_ego: " << m_ego.x << ", " << m_ego.y << ", " << m_ego.spd << ", " << m_ego.th << endl;           
        // int buffInt;
        // stringstream ss;
        // ss.str(msg->data.at(1));
        // ss >> buffInt;
        // std::cout << "INT: "<< buffInt << std::endl;
    }
    if (msg->data.size() == 2) {
        // std::cout << "222222" << std::endl;
	    DRIVE_FLAG = true;
        m_distance_to_intersection = msg->data.at(0);
        m_navi_info = (msg->data.at(0) == 0.0 && msg->data.at(1) == 1) ? m_navi_info: msg->data.at(1);
        // m_navi_info = msg->data.at(1);
        ROS_INFO("nav info2 : %d",m_navi_info);
        m_at_intersection = (m_distance_to_intersection > m_inter_threshold) ? false : true; // if you want to test with watchmile

    //     // m_vehicle_x_global = msg->data.at(2) * 100.0/ 3.0;
    //     // m_vehicle_y_global = msg->data.at(3) * 100.0/ 3.0;
    //     // m_vehicle_th_global = msg->data.at(5);

    }
}




// MK ============================
// max_peds_dist; maximum bound of the distance of which the pedestrian is taken into account 
// min_safety_dist; minimum safety bound of the distance of which the pedestrian is taken into account 
// ped_dist; closest distance from pedestrians
// decel [m/s^2]; constant deceleration value
// target_velocity: the vehicle's target velocity
double linear_decel_velo(double ped_dist, double target_velocity, double min_safety_dist, double max_peds_dist, double decel) {
// def linear_decel(ped_dist, target_velocity=3, min_safety_dist=5.75, max_peds_dist=10, decel=1):
    if (min_safety_dist < ped_dist && ped_dist < max_peds_dist) // linearly decreased in inverse proportional manner to the distance
        return target_velocity * (ped_dist - min_safety_dist) / (max_peds_dist - min_safety_dist);
    else if (ped_dist < min_safety_dist)
        return 0.0;
    else
        return target_velocity;
}

// // ============================
// //% target_{x,y}: position with respect to the global frame
// //% coord_{x,y,th}: pose with respect to the global frame
// //% rel_{x,y}: relative position of target with respect to coord_{x,y,th}
// void GetRelativePosition(double target_x, double target_y, double target_th, double coord_x, double coord_y, double coord_th, double &rel_x, double &rel_y) {
//     double rel_position_x = target_x - coord_x;
//     double rel_position_y = target_y - coord_y;
//     double D = sqrt(pow(rel_position_x, 2) + pow(rel_position_y, 2));
//     double alpha = atan2(rel_position_y, rel_position_x);
//     rel_x = D * cos(alpha - coord_th);
//     rel_y = D * sin(alpha - coord_th);
//     rel_th = target_th - coord_th;
// }

double VelocityController() {
    // Parameters
    double target_velo{5.0}; // [km/h]
    double safety_dist{3.0}; // [m]
    double attention_dist{30.0}; // [m]

    double desired_velocity = target_velo;

    if (!ego_vehicle_pose_flag) // ego's pose haven't got yet.
        return desired_velocity;

    nearest = 100;  //100 mean no pedestrian detected
    for(PedPose person: m_peds) { // finds a distance from the closest pedestrian
        if( person.x > 0.0 && person.dist() < nearest ) // 만약, 보행자가 차량의 앞쪽에 있으면서,, 일정 거리 이하로 차량에 가까울 때. 
            nearest = person.dist();
    }
    if(nearest <= PEDESTRIAN_WARNING_THRESHOLD) 
        cout << "% WARNING :: Pedestrian in " << nearest <<" meters" << endl;

    // 차량으로부터 3 m 에서 15 m 안에 보행자가 있을 때만 고려. 1 m/s^2 감속 진행. 차량의 타겟 속도는 5 km/h이며, nearest는 인식된 보행자 중, 차량과 가장 가까이 있는 보행자의 거리.
    desired_velocity = linear_decel_velo(nearest, target_velo, safety_dist, attention_dist, 1.0); //(가까운 보행자와의 거리, 정지 threshold, 보행자 경계를 위한 범위 (min, max), deceleration)
    // }

    // ========== VISUAZLIZE pedestrian interaction info ==========
    if (nearest == 100) // it means that no m_peds are detected
        text_overlay.text = "Not any pedestrians \n are detected";
    else {
        text_overlay.text = "Nearest Ped: " + to_string(nearest)+"\nVelocity_cmd: "+to_string(desired_velocity);
        // std::cout << "DESIRED VELO: " << desired_velocity << std::endl;
    }
    Pub_text_marker.publish(text_overlay);
    // Pub_pedestrian_normal.publish(normal_pedestrian);
    
    return desired_velocity;
}

void Publish_topic(double steer, double velo) {
    // static int dk;
    // pub_cnt++;
    // double dt = (ros::Time::now() - m_ros_time).toSec();
    // if (limitDeg*dt < abs(steer - m_Steer_cmd)) {
    //     m_Steer_cmd += (double)(SIGN(steer - m_Steer_cmd)*limitDeg*dt);
    // }
    // else
    if (abs(steer) > MAX_STEERING)
        steer = SIGN(steer) * MAX_STEERING;
    m_Steer_cmd = steer;
    m_Velocity_cmd = velo;
    
    std_msgs::Float32MultiArray controlData;
    controlData.data.push_back(m_Steer_cmd);
    controlData.data.push_back(m_Velocity_cmd); // velocity
    pub_control.publish(controlData);

}


/////////////////////////////////////////////////
/////////////////  TENTAClE OBSTACLE DETECTION

double map_range = 0.0, m_gridResol = 0.0, m_gridDim = 0.0;
double m_gridDim_width = 0.0, m_gridDim_height = 0.0;
pcl::PointCloud<pcl::PointXYZ>::Ptr g_pTree;
pcl::KdTreeFLANN<pcl::PointXYZ> g_kdTree;

//add
void arr2real(int recvX, int recvY, double& outX, double& outY) {
    outX = recvX * m_gridResol - (m_gridResol*m_gridDim - m_gridResol) / 2.0;
    outY = recvY * m_gridResol - (m_gridResol*m_gridDim - m_gridResol) / 2.0;
}

void real2arr(double recvX, double recvY, int& outX, int& outY) {
    outX = (m_gridDim / 2.0) + recvX / m_gridResol;
    outY = (m_gridDim / 2.0) + recvY / m_gridResol;
}

void arr2real_dv(int recvX, int recvY, double& outX, double& outY) {
    outX = recvX * m_gridResol - (m_gridResol * m_gridDim_width - m_gridResol) / 2.0;
    outY = recvY * m_gridResol - (m_gridResol * m_gridDim_height - m_gridResol) / 2.0;
    double rX = outX;
    double rY = outY;
    outX = -rY + 18.0;
    outY = rX;
}

std::vector<pcl::PointXYZ> SearchNodeByRadius(pcl::PointXYZ searchPoint, float radius)
{
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    std::vector<pcl::PointXYZ> pvNode;
    if( g_kdTree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance ) > 0 )
        // searchPoint 기준으로 radius 반경 내에 존재하는 점들을 찾는 함수.
        // 결과로 반경내 점들의 인덱스와 거리가 저장된다.
        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)//저장된 반경내 점들의 인덱스를 사용해 원래 좌표를 pvNode에 저장. pvNode를 반환
            pvNode.push_back(g_pTree->points[pointIdxRadiusSearch[i]]);

    return pvNode;
}


void publish_peds_info() {
    normal_pedestrian.poses.clear();
    normal_pedestrian.header.stamp = ros::Time::now();
    normal_pedestrian.header.frame_id = "map";
    geometry_msgs::Pose pose;
    for(PedPose person : m_peds) {
        pose.position.x = person.x;
        pose.position.y = person.y;
        pose.position.z = 15.0;
        // float resolution =100.0;
        // for(int i=0;i<resolution;i++) {
        //     geometry_msgs::Point point;
        //         point.x = person.dist() * cos(i/resolution * 2*M_PI);
        //         point.y = person.dist() * sin(i/resolution * 2*M_PI);
        //         point.z = 0;
        //     geometry_msgs::Pose pose;
        //         pose.position = point;
        //         pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        normal_pedestrian.poses.push_back(pose);
    } 
    Pub_pedestrian_normal.publish(normal_pedestrian);
}

bool isFreeSpace(float x, float y, double yaw)
{
    // Large Circle
    int obsCnt = SearchNodeByRadius(pcl::PointXYZ(x, y, 0),m_safe_region).size();
    obsCnt += SearchNodeByRadius(pcl::PointXYZ(x + m_collision_radius*cos(yaw), y + m_collision_radius*sin(yaw), 0),m_safe_region).size();
    obsCnt += SearchNodeByRadius(pcl::PointXYZ(x + 2*m_collision_radius*cos(yaw), y + 2*m_collision_radius*sin(yaw), 0),m_safe_region).size();
    //--------------------------------------------------------------------------------------------------------   
    //each vertex collision-check on the vehicle
    obsCnt += SearchNodeByRadius(pcl::PointXYZ(x - 0.719*cos(yaw) - 0.9175*sin(yaw),
                                               y - 0.719*sin(yaw) + 0.9175*cos(yaw), 0),m_pointsafe_region1).size();
    obsCnt += SearchNodeByRadius(pcl::PointXYZ(x - 0.719*cos(yaw) + 0.9175*sin(yaw), 
                                               y - 0.719*sin(yaw) - 0.9175*cos(yaw), 0),m_pointsafe_region1).size();
    obsCnt += SearchNodeByRadius(pcl::PointXYZ(x + 2*m_collision_radius*cos(yaw) + 0.735*cos(yaw) - 0.9175*sin(yaw),
                                               y + 2*m_collision_radius*sin(yaw) + 0.735*sin(yaw) + 0.9175*cos(yaw), 0),m_pointsafe_region2).size();
    obsCnt += SearchNodeByRadius(pcl::PointXYZ(x + 2*m_collision_radius*cos(yaw) + 0.735*cos(yaw) + 0.9175*sin(yaw), 
                                               y + 2*m_collision_radius*sin(yaw) + 0.735*sin(yaw) - 0.9175*cos(yaw), 0),m_pointsafe_region2).size();
    //empty side (on the sides of the vehicle)
    obsCnt += SearchNodeByRadius(pcl::PointXYZ(x + m_collision_radius*cos(yaw) - 0.6593*cos(yaw) - 0.8587*sin(yaw), 
                                               y + m_collision_radius*sin(yaw) - 0.6593*sin(yaw) + 0.8587*cos(yaw), 0),m_sidesafe_region).size();
    obsCnt += SearchNodeByRadius(pcl::PointXYZ(x + m_collision_radius*cos(yaw) - 0.6593*cos(yaw) + 0.8587*sin(yaw), 
                                               y + m_collision_radius*sin(yaw) - 0.6593*sin(yaw) - 0.8587*cos(yaw), 0),m_sidesafe_region).size();
    obsCnt += SearchNodeByRadius(pcl::PointXYZ(x + m_collision_radius*cos(yaw) + 0.6593*cos(yaw) - 0.8587*sin(yaw), 
                                               y + m_collision_radius*sin(yaw) + 0.6593*sin(yaw) + 0.8587*cos(yaw), 0),m_sidesafe_region).size();
    obsCnt += SearchNodeByRadius(pcl::PointXYZ(x + m_collision_radius*cos(yaw) + 0.6593*cos(yaw) + 0.8587*sin(yaw), 
                                               y + m_collision_radius*sin(yaw) + 0.6593*sin(yaw) - 0.8587*cos(yaw), 0),m_sidesafe_region).size();
//--------------------------------------------------------------------------------------------------------   
    if( obsCnt > 0 )
        return false;
    else
        return true;
}

bool isFreeSpace2(float x, float y)
{
    // Large Circle
    int obsCnt = SearchNodeByRadius(pcl::PointXYZ(x, y, 0), m_safe_region).size();
//--------------------------------------------------------------------------------------------------------   
    if( obsCnt > 0 )
        return false;
    else
        return true;
}

void CallbackDistMap(const nav_msgs::OccupancyGrid::Ptr map) {
    m_distMap_flag = true;
    // m_distance_map = map;
    // Call A Distance Map
    DistanceMap = new double*[map->info.width];
    for (int x = 0; x < map->info.width; x++)
        DistanceMap[x] = new double[map->info.height];
    for (int x = 0 ; x < map->info.width ; x++)
        for (int y = 0; y < map->info.height ; y++)
            DistanceMap[x][y] = map->data[map->info.width * y + x];
}


/////////////////  TENTAClE FUNCTION
double m_dist_adj_pts = 0.1; // be a distance between two adjacent points
double distance_map_based_tentacle(double v_j) { // VALUE IS COST! IT IS GOING TO SELECT THE MINIMUM TENTACLE
    // GEOMETRIES
    // std::cout << "distance_map_based_tentacle function start" << std::endl;
    
    double sizeratio = 0.6; // magic number: 0.73 // 텐타클의 길이 비율율
    double q_tentacle = pow((v_j - m_velo_s)/(m_velo_e - m_velo_s), (1.0 / 1.2));// * sizeratio; // 0.095667?? // q_tentacle : paper eq(5)
    double l_outmost = (8 + 33.5 * pow(q_tentacle, 1.2)) * sizeratio;      // ; 7.303120
    // std::cout << "v_j "<< v_j << std::endl;
    // std::cout << "m_velo_s "<< m_velo_s << std::endl; // 0
    // std::cout << "m_velo_e "<< m_velo_e << std::endl; // 0

    
   

    double R_outmost = l_outmost / ((0.6 * M_PI) * (1 - pow(q_tentacle, 0.9)));    // Outmost radius of tentacle : paper eq(2)  4.407623
    double dphi_tentacle[81];              // d(phi) of each tentacle
    double l_tentacle[81];                 // tentacle length : paper eq(3),(4)
    double r_tentacle[81];                 // radius of each tentacle: paper eq(3)
    vector<Vector3d> m_TentaclePath[81];     // Tentacle points
    
    // Building A Line of Tentacle
    r_tentacle[40] = 999999.9;  //infinite, straight_line for tentacle --> wtf...?
    for (int k = 0; k < 41; k++){ // FROM LEFT TO CENTER // 근데 40은 무한대 아닌가? 41->40으로 수정함 3/12
        
        l_tentacle[k] = l_outmost + 20 * sqrt(k / 40.0) * sizeratio; // k가 커질수록 l_tentacle[]이 길어짐. floor는 주어진 실수보다 작거나 같은 가장 큰 실수값 반환환
        r_tentacle[k] = pow(m_rho, k) * R_outmost;
        dphi_tentacle[k] = m_dist_adj_pts / abs(r_tentacle[k]); // angle which makes the distance between two adjacent points, 10 cm
        
        // cout<<k<<"th : "<<l_tentacle[k]<<endl;

        for (int m = 0; m < floor(l_tentacle[k]/m_dist_adj_pts); m++){ // 길이에 따라 for loop 몇번 돌지 결정 floor는 주어진 실수보다 작거나 같은 가장 큰 실수값 반환.
            // 길이가 길수록 더 많은 점 생성.
            m_TentaclePath[k].push_back(Vector3d(r_tentacle[k] * sin(dphi_tentacle[k] * m) - m_center2rear,
                                              r_tentacle[k] * (1 - cos(dphi_tentacle[k] * m)),
                                              dphi_tentacle[k] * m));
        }
    }
    
    for (int k = 41; k < 81; k++){ // FROM RIGHT TO CENTER
        l_tentacle[k] = l_outmost + 20 * sqrt((k - 40) / 40.0) * sizeratio;
        r_tentacle[k] = -pow(m_rho, (k - 41)) * R_outmost;
        dphi_tentacle[k] = m_dist_adj_pts / abs(r_tentacle[k]);    // angle which makes the distance between two adjacent point 10 cm

        for (int m = 0; m < floor(l_tentacle[k]/m_dist_adj_pts); m++){
            m_TentaclePath[k].push_back(Vector3d(-r_tentacle[k] * sin(dphi_tentacle[k] * m) - m_center2rear,
                                              r_tentacle[k] * (1-cos(dphi_tentacle[k] * m)),
                                              -dphi_tentacle[k] * m)); // 
        }
    }
  
/////////////////  TENTAClE OBSTACLE DETECTION
    double m_obstacle[81];      // a number of points (or index) to obstacle for each tentacle // 장애물까지 point의 개수
    double l_obstacle[81];      // a distance (or length) to obstacle for each tentacle // 장애물까지의 거리
    for (int k = 0; k < 81; k++) {
        for (int m = 0; m < m_TentaclePath[k].size(); m++) { // 각 텐타클 path에 생성된 point의 수만큼 loop
            m_obstacle[k] = m;
            if (isFreeSpace(m_TentaclePath[k][m][0] - m_center2rear , m_TentaclePath[k][m][1], m_TentaclePath[k][m][2]) == false) {
                // 각각 isFreeSpace에서 x,y, yaw인데
                break;
            }
        }
        l_obstacle[k] = m_obstacle[k] * m_dist_adj_pts; // m_dist_adj_pts는 인접한 포인트끼리의 거리
        // cout<<k<<"th : "<<l_obstacle[k]<<endl;
    }

    double m_obstacle_gen[81];
    double l_obstacle_gen[81];
    int k_idx_most_free = 0;
    for (int k = 0; k < 81; k++) {
        for (int m = 0; m < m_TentaclePath[k].size(); m++) {
            m_obstacle_gen[k] = m;
            if (isFreeSpace2(m_TentaclePath[k][m][0] - m_center2rear, m_TentaclePath[k][m][1]) == false) 
                break;
        }
        
        l_obstacle_gen[k] = m_obstacle_gen[k] * m_dist_adj_pts; // within tentacle,
        if(l_obstacle_gen[k] > l_obstacle_gen[k_idx_most_free]) // find a longest distance to obstacle
            k_idx_most_free = k;
    }

////////////////////// DECISION PART
    double l_safety_dist = 4;        // safety distance // originally 6 zz
    double deceleration = 1.0;         // deceleration (m/s^2)
    double l_crash_dist = l_safety_dist;
    // double l_crash_dist = l_safety_dist + pow(v_j * KMpH2MpS,2)/(2.0 * deceleration);         
    // Crash distance : paper eq (9), If lower than l_crash_dist, that tentacle is not drivable
 
    // VARIABLES
    // storing a value
    double v_clearance[81];
    double v_trajectory[81];
    double v_flatness[81];
    vector<double> v_combined;
    
    // clearance value
    double c_clearance = log(3) / (20.0 * sizeratio); // paper eq (11)
    double c_flatness = log(3) / 0.3;


    double dist_diff[81], angle_diff[81];
    double v_dist_max = 0, v_dist_min = 100;        // the max/min value of v_dist at all tentacles (need to be calculated.)
    double v_dist[81];
    int idx_path = floor(l_crash_dist / m_dist_adj_pts); //55      // distance between waypoints is 0.2 m 
    // cout<<idx_path<<endl;
    // combined value // vnv
    double a_clearance = 1;       // 1.0
    double a_flatness = 1;        // Distance // 0.2?
    double a_trajectory = 2;      // 0.5 for the best

    double min_dis = INF;
    double way_dis = 0.0;
    int current_waypoint = 0;

    double ten_slope = 0.0;
    double trj_slope = 0.0;
    double dis_a = 0.0;
    double alpha = 0.0;
    visualization_msgs::MarkerArray tem;

    for(int k = 0; k < 81; k++){

        // cout<<k<<"번째 tentacle's radius : "<<r_tentacle[k]<<endl;
        // CLEARANCE VALUE // Equation 10.
        if (m_obstacle[k] == m_TentaclePath[k].size() - 1) v_clearance[k] = 0.0; // No obstacles in the path
        else v_clearance[k] = 2.0 - 2.0 / (1 + exp(-c_clearance * l_obstacle[k])); // l_obs의 길이가 길면 v_clearance는 작아짐. l_obs가 짧아지면 v_clr는 커짐짐

        // cout<<k<<"th : "<<m_TentaclePath[k].size()<<endl;
        

        // OBSTACLE RATE VALUE
        double obs_rate_sum = 0.0;
        int _count = 0;
        for (int j = 0; j < m_obstacle[k]; j++) {
            int gridX = (int)((m_TentaclePath[k][j][0] - m_center2rear + (m_gridDim * m_gridResol) / 2.0) / (double)m_gridResol);
            int gridY = (int)((m_TentaclePath[k][j][1] + (m_gridDim * m_gridResol) / 2.0) / (double)m_gridResol);
            obs_rate_sum += (m_distMap_flag && 0 <= gridX < m_gridDim && 0 <= gridY < m_gridDim) ? DistanceMap[gridX][gridY] : 100.0;
            _count++;
        }
        v_flatness[k] = 2.0 / (1.0 + exp(-c_flatness * (obs_rate_sum / (100.0 * _count)))) - 1.0; // if obstacle_rate_sum increases, v_flatness increases


        // TRAJECTORY VALUE
        v_trajectory[k] = 1.0;
        double ten_x, ten_y;
        Local2Global(m_TentaclePath[k][idx_path][0],m_TentaclePath[k][idx_path][1],ten_x,ten_y);

        double global_tentacle_center, global_zero;
        Local2Global(0,r_tentacle[k],global_zero, global_tentacle_center);

        ten_slope = - (ten_x - global_zero) / (ten_y - global_tentacle_center); //(0,r_tentacle) 텐타클 경로의 각도.......
        // cout << k << " [idx_path] : " << idx_path << endl;
          

        if(trajectory.poses.size() == 0) // 전역경로가 있을 때 continue
            continue;

    
        

        for(int idx = 0;idx < trajectory.poses.size();idx++){
            way_dis = DISTANCE(trajectory.poses[idx].pose.position.x, trajectory.poses[idx].pose.position.y, // for loop 돌며 경로상 가장 가까운 점을 waypoint로 한다.
                                                             m_car.x,                               m_car.y);
            if(way_dis < min_dis){
                min_dis = way_dis;
                current_waypoint = idx;
            }
        }
        // std::cout<<"trajectory : "<<trajectory.poses.size()<<"current_waypoint+idx_path+1 : "<<current_waypoint+idx_path+1<<std::endl;

        trj_slope = (trajectory.poses[current_waypoint+idx_path+1].pose.position.y - trajectory.poses[current_waypoint+idx_path-1].pose.position.y) / 
                        (trajectory.poses[current_waypoint+idx_path+1].pose.position.x - trajectory.poses[current_waypoint+idx_path-1].pose.position.x);

        // 현재 waypoint의 앞뒤 하나씩의 trajectory pose를 통해서 trajectory의 기울기를 구한다.
        
        // cout<<"current : "<<trajectory.poses[current_waypoint+idx_path].pose.position.x<<" | "<<trajectory.poses[current_waypoint+idx_path].pose.position.y<<endl;
        /////////////////////
        // cout<<k<<"tenslope : "<<ten_slope<<" trj_slope : "<<trj_slope<<endl;   
        
        
        // dis_a = DISTANCE( m_TentaclePath[k][idx_path][0], m_TentaclePath[k][idx_path][1],
        //                  trajectory.poses[current_waypoint+idx_path].pose.position.x, trajectory.poses[current_waypoint+idx_path].pose.position.y);
        
        // dis_a calculate part ===================================
        // original
        // dis_a = DISTANCE( ten_x, ten_y,
        //                  trajectory.poses[current_waypoint+idx_path].pose.position.x, trajectory.poses[current_waypoint+idx_path].pose.position.y);
        dis_a = DISTANCE( ten_x, ten_y,
            trajectory.poses[current_waypoint].pose.position.x, trajectory.poses[current_waypoint].pose.position.y);
        
        
                         // cout << k << "th : " << dis_a << endl;
        // cout<<k<<"th : "<<m_TentaclePath[k][idx_path][0]<<endl;

        visualization_msgs::Marker mk2;
        mk2.header.frame_id = "local";
        mk2.header.stamp = ros::Time();
        mk2.id = k;
        mk2.type = visualization_msgs::Marker::SPHERE;
        mk2.action = visualization_msgs::Marker::ADD;
        mk2.pose.position.x = ten_x;
        mk2.pose.position.y = ten_y;
        mk2.pose.position.z = 0.0;
        mk2.pose.orientation.x = 0.0;
        mk2.pose.orientation.y = 0.0;
        mk2.pose.orientation.z = 0.0;
        mk2.pose.orientation.w = 1.0;
        mk2.scale.x = 1;
        mk2.scale.y = 1;
        mk2.scale.z = 1;
        mk2.color.r = 0.0;
        mk2.color.g = 1.0;
        mk2.color.b = 0.0;
        mk2.color.a = 1.0;
        tem.markers.push_back(mk2);
        
        alpha = abs(atan2(abs((ten_slope - trj_slope)), abs((1 + ten_slope * trj_slope)))); // 삼각함수 덧셈정리 사용. ten_slope와 trj_slope의 차이를 구하기
        // cout << k << "th alpha: " << alpha<<" || dis"<<dis_a<<" || " ;
        // v_dist[k] = dis_a + r_tentacle[k] * alpha;
        v_dist[k] = dis_a + 3.0 * alpha;  // vnv // ten_slope와 trj_slope의 차이가 클수록 v_dist값이 커지게 됨
        // cout << k << "th: " << v_dist[k] << endl;
    }

    // for ( int i=0; i<81; i++){
    //     printf("v_dist[%d] = %.2f \n",i,v_dist[i]);
    // }
 
    Pub_tem2.publish(tem); // tem_x tem_y publish

    // geometry_msgs::PoseStamped trj_pose;
    // trj_pose.header.frame_id = "map";
    // trj_pose.pose.position.x = trajectory.poses[current_waypoint+idx_path].pose.position.x;
    // trj_pose.pose.position.y = trajectory.poses[current_waypoint+idx_path].pose.position.y;

    // pub_trj.publish(trj_pose);
    // cout<<trajectory.poses[current_waypoint+idx_path].pose.position.x<<" || "<<trajectory.poses[current_waypoint+idx_path].pose.position.y<<endl;
    
    if(trajectory.poses.size() != 0){
        visualization_msgs::Marker mk;
        mk.header.frame_id = "local";
        mk.header.stamp = ros::Time();
        mk.id = 0;
        mk.type = visualization_msgs::Marker::SPHERE;
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.position.x = trajectory.poses[current_waypoint+idx_path].pose.position.x;
        mk.pose.position.y = trajectory.poses[current_waypoint+idx_path].pose.position.y;
        mk.pose.position.z = 0.0;
        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;
        mk.scale.x = 1;
        mk.scale.y = 1;
        mk.scale.z = 1;
        mk.color.r = 0.0;
        mk.color.g = 0.0;
        mk.color.b = 1.0;
        mk.color.a = 1.0;
        Pub_tem.publish(mk); // 경로 pub
    }


    double v_max = *max_element(begin(v_dist), end(v_dist));
    double v_min = *min_element(begin(v_dist), end(v_dist));
    // cout << "v_max: " << v_max << " " << "v_min: " << v_min << endl; 
    // double max_navi = *max_element(v_navi, v_navi + 81);
    // double min_navi = *min_element(v_navi, v_navi + 81);

    // Finding Max/Min of v_dist and Calculating COMBINED VALUE
    for (int k = 0; k < 81; k++){
        // v_navi[k] = (v_navi[k] - min_navi) / (max_navi - min_navi);
        v_trajectory[k] = (v_dist[k] - v_min) / (v_max - v_min);
        // cout<<k<<"th v_traj: "<<v_trajectory[k]<<  " v_combined: " << a_trajectory * v_trajectory[k] << endl;
        v_combined.push_back(a_clearance * v_clearance[k] + a_flatness * v_flatness[k] + a_trajectory * v_trajectory[k]);
                        // + a_navi * v_navi[k];
    }

    // Find drivable tentacles
    vector<int> drivable_tentacle_idx;
    for (int k = 0; k < 81; k++){
        if (l_obstacle[k] >= l_crash_dist) { // tentacle의 장애물까지의 길이가 crash_dist 보다 크면 drivablabe_tentacle_idx에 push
            drivable_tentacle_idx.push_back(k);
        }
    }

    int selected_k = -1;
    vector<int> drivable_tentacle_idx_left;
    vector<int> drivable_tentacle_idx_right;
    vector<int> drivable_tentacle_idx_straight;
    
    // store drivable tentacles into each discrete tentacle 
    for(const auto &driv_idx: drivable_tentacle_idx) // 각 방향으로 drivable한 idx를 저장. 
    {   // m_discrimiate_range_lr = 1
        // m_discrimiate_range = 1
        if (driv_idx <= 20)  // drive_idx < 1
            drivable_tentacle_idx_left.push_back(driv_idx);
        if (driv_idx > 0 && driv_idx < 80) // 1 <= drive_idx <= 40 || 42 <= 81
            drivable_tentacle_idx_straight.push_back(driv_idx);
        if (driv_idx >= 60) // 41 <= driv_idx 
            drivable_tentacle_idx_right.push_back(driv_idx);
    }


    if (drivable_tentacle_idx.size() == 0) {// if there is no drivable tentacles
        selected_k = k_idx_most_free; // 만약 drivable한 idx가 없으면 가장 긴 길이의 경로를 선택하여 주행
        //// cout<<"can 13 selected k : "<<selected_k<<endl;
    }
    else { 
	    bool _decision = (m_watchmile_mode) ? m_at_intersection : true; // 만약 watchmile_mode가 True라면 _decision을 m_at_intersection로 변경 False이면 True로 함
        // std::cout<<m_at_intersection<<"_decision"<<_decision<<"m_navi_info"<<m_navi_info<<std::endl;
        // bool _decision = DRIVE_FLAG;
        // At LEFT

        // --------------------------------------------------------------------------------------------------------------//
        // #### 현재 navi_info를 callback하는 함수가 주석 처리되어 있음. m_navi_info는 멤버변수 설정 부분에서 1로 설정되어 있음//
        // --------------------------------------------------------------------------------------------------------------//
        if (_decision && m_navi_info == 2) {
            // std::cout<<"left    :      "<< m_discriminate_range_lr<<std::endl;
            for (int k = 0; k < m_discriminate_range_lr; k++) {
                if (v_clearance[k] == 0.0) {
                    selected_k = k;
                    cout<<"can 1 selected k : "<<selected_k<<endl;
                    break;
                }
            }
            if (drivable_tentacle_idx_left.size() == 0) { // if there are only non-drivable tentacles // STRIAHGT
                
                
                if (drivable_tentacle_idx_straight.size() != 0){
                    selected_k = drivable_tentacle_idx_straight[0];
                    cout<<"can2 selected k : "<<selected_k<<endl;
                }
                
                for(const auto &driv_idx: drivable_tentacle_idx_straight)
                {
                    if(v_combined[driv_idx] < v_combined[selected_k]) {
                        selected_k = driv_idx;             // find selected_k which makes v_combined minimum
                        cout<<"can3 selected k : "<<selected_k<<endl;
                    }
                }
            }
            else if (selected_k == -1) { // there is no clearance tentacle, but drivable tentacle exists
                selected_k = drivable_tentacle_idx_left[0];
                cout<<"can 4 selected k : "<<selected_k<<endl;
                for(const auto &driv_idx: drivable_tentacle_idx_left)
                {
                    if(v_combined[driv_idx] < v_combined[selected_k]) {
                        selected_k = driv_idx;             // find selected_k which makes v_combined minimum
                        cout<<"can5 selected k : "<<selected_k<<endl;
                    }
                }
            }
        }
        // At RIGHT
        else if (_decision && m_navi_info == 3) {
            // std::cout<<"right   :    "<<41 + m_discriminate_range_lr<<std::endl;
            for (int k = 41; k < 41 + m_discriminate_range_lr; k++) {
                // std::cout<<"K    :      "<< k <<std::endl;
                if (v_clearance[k] == 0.0) {
                    selected_k = k;
                    cout<<"can6 selected k : "<<selected_k<<endl;
                    break;
                }
            }
            
            if (drivable_tentacle_idx_right.size() == 0) {// not any drivable crash distance
                // std::cout<<"STOP1"<<std::endl;
                // std::cout << "Vector size: " << drivable_tentacle_idx_straight.size() << std::endl;

                if (drivable_tentacle_idx_straight.size() != 0){
                    selected_k = drivable_tentacle_idx_straight[0];
                    cout<<"can7 selected k : "<<selected_k<<endl;
                }
                
                
                for(const auto &driv_idx: drivable_tentacle_idx_straight)
                {   
                    
                    std::cout<<"driv_idx  :  "<<driv_idx<<std::endl;
                    if(v_combined[driv_idx] < v_combined[selected_k]) {
                        selected_k = driv_idx;             // find selected_k which makes v_combined minimum
                        cout<<"can8 selected k : "<<selected_k<<endl;
                    }
                }
                std::cout<<"STOP2"<<std::endl;
                
            }
            else if (selected_k == -1) {
                // std::cout<<"STOP3"<<std::endl;
                selected_k = drivable_tentacle_idx_right[0];
                cout<<"can 9 selected k : "<<selected_k<<endl;
                for(const auto &driv_idx: drivable_tentacle_idx_right)
                {
                    if(v_combined[driv_idx] < v_combined[selected_k]) {
                        selected_k = driv_idx;             // find selected_k which makes v_combined minimum
                        cout<<"can10 selected k : "<<selected_k<<endl;
                    }
                }
            }
        }
    // At STRAIGHT // 그럼 얘가 실행되는 상태 --------------------------------------------------------------------------- //
        else { 
            if (drivable_tentacle_idx_straight.size() == 0) { // if there are only non-drivable tentacles // STRIAHGT
                selected_k = k_idx_most_free; 
                // cout<<"can 11 selected k : "<<selected_k<<endl;
            }
            else {
                selected_k = drivable_tentacle_idx_straight[0];
                // cout<<"can drivable_tentacle_idx_straight selected k : "<<selected_k<<endl;
                for(const auto &driv_idx: drivable_tentacle_idx_straight)
                {
                    
                    if(v_combined[driv_idx] < v_combined[selected_k]) { 
                        // ****  V_combined가 가장 작은 값을 선택하게 됨 **** //
                        selected_k = driv_idx;             
                        // find selected_k which makes v_combined minimum
                        ////cout<<"can 12 selected k : "<<selected_k<<endl;
                    }
                }
            }
        }
    } 
    // -----------------------------------------------------------------------------------------------------------------//
    ////cout << "[selected_k] : "<< selected_k << endl;

//  --------------------------------------------------  Rviz용 publish 시작작 ---------------------------------------------------------- //
    geometry_msgs::PoseArray tentacle_path;  //add

    tentacle_path.header.stamp = ros::Time::now();
    tentacle_path.header.frame_id = "map";

    for (int k = 0; k < 81; k++){
        for (int m = 0; m < m_TentaclePath[k].size(); m++){
            geometry_msgs::PoseStamped poseStamped;

            double X_tentacle_l, Y_tentacle_l;

            // Local2Global(m_TentaclePath[k][m][0],m_TentaclePath[k][m][1],X_tentacle_l,Y_tentacle_l);
            poseStamped.pose.position.x = m_TentaclePath[k][m][0];
            poseStamped.pose.position.y = m_TentaclePath[k][m][1];

            // std::cout << "before : " << m_TentaclePath[k][m][0] << ", " << m_TentaclePath[k][m][1] << " after : " << X_tentacle_l << ", " << Y_tentacle_l << std::endl;

            // poseStamped.pose.position.x = X_tentacle_l;
            // poseStamped.pose.position.y = Y_tentacle_l;
            poseStamped.pose.position.z = 0.5;
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(m_TentaclePath[k][m][2]);
            poseStamped.header = m_header;
            poseStamped.pose.orientation = odom_quat;
            tentacle_path.poses.push_back(poseStamped.pose);
        }
    }
    Pub_tentacle.publish(tentacle_path);
    //
    // std::cout<<"-----------------2"<<std::endl;
    // PoseArray of chosen tentacle (by the decision process)
    geometry_msgs::PoseArray chosen_path_value, chosen_tentacle;
    chosen_path_value.header.stamp = ros::Time::now();
    chosen_path_value.header.frame_id = "map";
    // std::cout<<"m_obstacle[selected_k] : "<<m_obstacle[selected_k]<<std::endl;

    if(m_obstacle[selected_k]<1)
        m_obstacle[selected_k] = 0;

    for (int m = 0; m < m_obstacle[selected_k]; m++){ // 장애물까지 선택된 경로 좌표 publish.
        geometry_msgs::PoseStamped poseStamped;

        double X_chosen_path_value_l, Y_chosen_path_value_l;

        // Local2Global(m_TentaclePath[selected_k][m][0],m_TentaclePath[selected_k][m][1],X_chosen_path_value_l,Y_chosen_path_value_l);
        // poseStamped.pose.position.x = X_chosen_path_value_l;
        // poseStamped.pose.position.y = Y_chosen_path_value_l;


        poseStamped.pose.position.x = m_TentaclePath[selected_k][m][0];
        poseStamped.pose.position.y = m_TentaclePath[selected_k][m][1];
        poseStamped.pose.position.z = 0.6;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(m_TentaclePath[selected_k][m][2]);
        poseStamped.header = m_header;
        poseStamped.pose.orientation = odom_quat;
        chosen_path_value.poses.push_back(poseStamped.pose);
    }
    Pub_chosenPathbyValue.publish(chosen_path_value);
    // std::cout<<"-----------------3"<<std::endl;
    // PoseArray of chosen tentacle (by the decision process)
    chosen_tentacle.header.stamp = ros::Time::now();
    chosen_tentacle.header.frame_id = "map";
             
    for (int m = 0; m < m_TentaclePath[selected_k].size(); m+=5){
        geometry_msgs::PoseStamped poseStamped;

        double X_chosen_tentacle_l, Y_chosen_tentacle_l;

        // Local2Global(m_TentaclePath[selected_k][m][0],m_TentaclePath[selected_k][m][1],X_chosen_tentacle_l,Y_chosen_tentacle_l);
        // poseStamped.pose.position.x = X_chosen_tentacle_l;
        // poseStamped.pose.position.y = Y_chosen_tentacle_l;


        poseStamped.pose.position.x = m_TentaclePath[selected_k][m][0];
        poseStamped.pose.position.y = m_TentaclePath[selected_k][m][1];
        poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(m_TentaclePath[selected_k][m][2]);
        poseStamped.header = m_header;
        poseStamped.pose.orientation = odom_quat;
        chosen_tentacle.poses.push_back(poseStamped.pose);
    }
    Pub_chosenPath.publish(chosen_tentacle);
//  --------------------------------------------------  Rviz용 publish 끝 ---------------------------------------------------------- //
    
    // -------------------------------------- Steering Angle Compute (bicycle model) ----------------------------------------------- //
    if(selected_k < 41){
        printf("path(0 is Most straight) : %d\n", selected_k - 40);
    }
    else{
        printf("path(0 is Most straight) : %d\n", 81 - selected_k);
    }
    
    double steer_from_tentacle = - atan2(1.0 / r_tentacle[selected_k], (v_j * KMpH2MpS / WHEEL_BASE)) * (_RAD2DEG) * STEERING_RATIO;
    return steer_from_tentacle;
    // ----------------------------------------------------------------------------------------------------------------------------- //
}

void Compute() {
    // std::cout << "compute function start" << std::endl;

    double gain = 1.173;
    double steer = gain * distance_map_based_tentacle(MIN_VEL_INPUT);
 
    double velo = VelocityController();
   
    // std::cout << "VELO: " << velo << std::endl;
    
    Publish_topic(steer, velo);

    // Only recent data is remained.
    for (vector<PedPose>::iterator it=m_peds.begin(); it!=m_peds.end();) {
        if ( (ros::Time::now() - (*it).detect_time).toSec() > m_data_keeping_time ) // data_keeping_time이 지난 데이터들은 버려짐.
            it = m_peds.erase(it);
        else
            ++it;
    }

    // Accredited Testing
    publish_peds_info();
}
void CallbackGlobalPath(const nav_msgs::Path::ConstPtr& msg){
    trajectory = *msg;
  
}

sensor_msgs::PointCloud2 convertToPointCloud2(const VPointCloud& vcloud) {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    for (const auto& point : vcloud.points) {
        pcl_cloud.push_back(point);
    }

    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(pcl_cloud, ros_cloud);
    ros_cloud.header.frame_id = "local";  // RViz에서 볼 수 있도록 프레임 설정
    ros_cloud.header.stamp = ros::Time::now();
    
    return ros_cloud;
}

void CallbackHeightMap(const nav_msgs::OccupancyGrid::Ptr map) {  // for occupancy grid map construction
    // std::cout << "callback height map start" << std::endl;
    m_ros_time = ros::Time::now();
    map_range = map->info.resolution * map->info.width;
    m_gridDim = map->info.width;
    m_gridResol = map->info.resolution; // 1개의 cell이 몇 미터인지/ width는 미터단위가 아니라 셀의 개수. 즉 width*resolution 해야 맵의 크기가 나옴옴

    HeightMap = new bool*[map->info.width];
    for (int x = 0; x < map->info.width; x++)
        HeightMap[x] = new bool[map->info.height]; 

    // Global Coordinate Obstacle Data
    m_obstaclePtCloudLocal.clear();
    m_obstaclePtCloudLocal.header.frame_id = "local";
    m_obstaclePtCloudLocal.points.resize(map->info.width*map->info.height);    //

    int obs_count = 0;
    vector<Vector2d> vObstacle;
  
    for (int x = m_gridDim-1 ; x >= 0 ; x--) {
        for (int y = m_gridDim-1 ; y >=0 ; y--) {
            HeightMap[x][y] = map->data[y * (map->info.width) + x] ? true : false;///All observalbe
           
            double doX, doY, doX_g, doY_g;// Global obstacles' INFO 
            arr2real(x * HeightMap[x][y], y * HeightMap[x][y], doX, doY);
            vObstacle.push_back(Vector2d(doX, doY));  // Current doX and doY are located with respect to the LOCAL-axis
            Local2Global(doX, doY, doX_g, doY_g);
            m_obstaclePtCloudLocal.points[obs_count].z = 1.0;
            m_obstaclePtCloudLocal.points[obs_count].x = doX_g; // 장애물이 있으면 좌표가 0,0,1이 아님
            m_obstaclePtCloudLocal.points[obs_count++].y = doY_g;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    m_obstaclePtCloudLocal.points.resize(obs_count);

    sensor_msgs::PointCloud2 car_occ = convertToPointCloud2(m_obstaclePtCloudLocal); // vpointcloud는 Velodyne전용 
    Pub_Occ2Global.publish(car_occ); // Heightmap 쓰는게 나을 거 같다 굳이 publish 안해도 될듯함.



    // Pub_obstacleLocal.publish(m_obstaclePtCloudLocal);


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   
    if( g_pTree != NULL )
        g_pTree->clear();
    g_pTree = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    

    for(int k = 0; k < vObstacle.size(); k++)
        g_pTree->push_back(pcl::PointXYZ(vObstacle[k](0), vObstacle[k](1), 0.0));
    
    g_kdTree.setInputCloud(g_pTree); // pcl(포인트 클라우드 라이브러리) 공간 내에서 빠르게 근접한 점을 찾기 위한 데이터 구조로 변환
    // 결국 g_kdTree가 장애물 occupancy map이다.
    // m_car.x = m_car.y = m_car.th = m_car.vel = 0.0;
    
    Compute();
    // std::cout << "callback height map end" << std::endl;
}

//% rel_{x,y}: relative position of target with respect to coord_{x,y,th}
void GetRelativePosition(double target_x, double target_y, double target_th, double coord_x, double coord_y, double coord_th, double &rel_x, double &rel_y, double &rel_th) {
    double rel_position_x = target_x - coord_x;
    double rel_position_y = target_y - coord_y;
    double D = sqrt(pow(rel_position_x, 2) + pow(rel_position_y, 2));
    double alpha = atan2(rel_position_y, rel_position_x);
    rel_x = D * cos(alpha - coord_th);
    rel_y = D * sin(alpha - coord_th);
    rel_th = target_th - coord_th;
}

vector<string> split_string(string input, char delimiter) {
    vector<string> answer;
    stringstream ss(input);
    string temp;
 
    while (getline(ss, temp, delimiter)) {
        answer.push_back(temp);
    }
 
    return answer;
}

// TOP과 Vestella Lab.의 X와 동일.          
// 보행자 정보 같은 경우. I.e., DYROS frame과 동일하기에 그대로 사용하면 됨.
// Assume that the datastructure of Info: [{id: CCTV12_1, top: 1800, left: 2451, update: 2022-10-18-10-20-15}]      which means [{CCTV ID, Y, X, The Time to be detected}] with respect to the original frame (Vestella Lab.)
void CallbackMQTT_PedInfo(const std_msgs::StringPtr Peds) {
    if (!ego_vehicle_pose_flag) 
        return; 
    //After the node successfully gets the vehicle's pose information, then, it recognizes the pedestrian information 
    std::string peds_list = Peds->data;
    std::string s = peds_list;

    auto end = std::chrono::system_clock::now();
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    // std::cout << "Actual PC Time: " << std::ctime(&end_time) << std::endl;

    // Here, the peds information is continuously stored in the vector.
    vector<string> result = split_string(peds_list, '}, {');
    for (int i = 1; i < result.size(); i++) {
        vector<string> tmp_ped_info = split_string(result[i], ', ');
        
        // *** CHANGE the pedestrian information to the DYROS frame. 
        int buffInt;
        stringstream ss;
        ss.str(tmp_ped_info[3]); 
        ss >> buffInt;
        double xxx = (double) buffInt;
        ss.str(tmp_ped_info[5]); 
        ss >> buffInt;
        double yyy = (double) buffInt;

        PedPose person = PedPose();
        // Pedestrian's position's dimension is changed to Meter
        double absolute_x = xxx * m_pix2meter;
        double absolute_y = yyy * m_pix2meter;

        // Now get the relative position of the pedestrian's position with respect to the vehicle's frame
        double rel_position_x = absolute_x - m_ego.x;
        double rel_position_y = absolute_y - m_ego.y;
        double D = sqrt(pow(rel_position_x, 2) + pow(rel_position_y, 2));
        double alpha = atan2(rel_position_y, rel_position_x);
        double coord_th = m_ego.th;
        person.x = D * cos(alpha - coord_th);
        person.y = D * sin(alpha - coord_th);

        // It contains the coordinate with respect to the absolute frame
        // 2023.01.17 MS
        if (person.x > 0.0) {
            m_peds.push_back(person);
        }

    }


}


void VisualizeValInit() {
    text_overlay.header.frame_id = "map"; 
    // text_overlay.header.stamp = ros::Time::now(); 
    text_overlay.text = "Initialized"; 
    text_overlay.color.r = 1.0;
    text_overlay.color.g = 1.0;
    text_overlay.color.b = 1.0;
    text_overlay.color.a = 1.0; 
    text_overlay.scale.z = 1.5; 
    text_overlay.type = visualization_msgs::Marker::TEXT_VIEW_FACING; 
    text_overlay.id = 0; 
    text_overlay.action = visualization_msgs::Marker::ADD; 
    text_overlay.pose.orientation = tf::createQuaternionMsgFromYaw(90.0 * _DEG2RAD); 
    text_overlay.pose.position.x = 0.0; 
    text_overlay.pose.position.y = -10.0; 
    text_overlay.pose.position.z = 5.0; 
}
//DK================================================

void MemberValInit(const ros::NodeHandle &ros_node){
    m_distMap_flag = false;

    // Sampling Collision Parameters
    ros_node.getParam("safe_region", m_safe_region);
    ros_node.getParam("collision_radius", m_collision_radius);
    ros_node.getParam("pointsafe_region1", m_pointsafe_region1);
    ros_node.getParam("pointsafe_region2", m_pointsafe_region2);
    ros_node.getParam("sidesafe_region", m_sidesafe_region);
    ros_node.getParam("center_to_rear", m_center2rear); // 1.096

    ros_node.getParam("limit_steering_velocity", limitDeg);
    ros_node.getParam("watchmile_mode", m_watchmile_mode);
    // m_watchmile_mode = false;
    
    ros_node.getParam("intersection_threshold", m_inter_threshold);
    
    // Tentacle
    ros_node.getParam("rho", m_rho); //1.15
    ros_node.getParam("velo_e", m_velo_e); //36
    ros_node.getParam("velo_s", m_velo_s); // 0.9
    ros_node.getParam("discriminate_range", m_discriminate_range);
    ros_node.getParam("discriminate_range_lr", m_discriminate_range_lr);

    //DK
    ros_node.getParam("use_pedestrian_tracking", m_use_pedestrian_tracking);
    m_Velocity_cmd = 3.0;

    m_Steer_cmd = 0.0;
    m_distance_to_intersection = 10;
    m_navi_info = 1; // straight
    m_at_intersection = false;

}

void DRIVINGFUNC_THREAD(){
    int argc = 0;
    char** argv;
    ros::init(argc, argv, "Driving_function_thread");
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh("~");
    
    // add
    Pub_tentacle = nh_.advertise<geometry_msgs::PoseArray>("poseArrayTentacle", 1);
    Pub_chosenPath = nh_.advertise<geometry_msgs::PoseArray>("poseArraychosenPath", 1);
    Pub_chosenPathbyValue = nh_.advertise<geometry_msgs::PoseArray>("poseArraychosenPathbyValue", 1);

    Pub_Occ2Global = nh_.advertise<sensor_msgs::PointCloud2>("Occ2Global", 1);
    // Pub_pedestrian = nh_.advertise<geometry_msgs::PoseArray>("poseArraychosenPathbyValue", 1);
    
    pub_control = nh_.advertise<std_msgs::Float32MultiArray>("/motion_planning_data", 1);
    Pub_obstacleLocal = nh_.advertise<VPointCloud>("velodyne_obs", 1); // MSK: Obstacle Information

    Pub_collision_check = nh_.advertise<geometry_msgs::PoseArray>("collision_edge", 1);
    pub_intersectFlag = nh_.advertise<std_msgs::Int32MultiArray>("intersect_flag", 1);
    
     // DK ==========================
    Pub_text_marker = nh_.advertise<visualization_msgs::Marker>("text_marker",1);
    Pub_pedestrian_normal = nh_.advertise<geometry_msgs::PoseArray>("normal_pedestrian",1);
    Pub_pedestrian_caution = nh_.advertise<geometry_msgs::PoseArray>("caution_pedestrian",1);
    // Sub_PedInfo = nh_.subscribe("/pedestrian_msg", 2, &CallbackPedInfo);    //DK

    pub_trj = nh_.advertise<geometry_msgs::Pose>("trj",1);
    Pub_tem2 = nh_.advertise<visualization_msgs::MarkerArray>("ma",1);
    Pub_tem = nh_.advertise<visualization_msgs::Marker>("mkmkmkmkmkmkmkmk",1);
    
    
    // Sub_PedInfo = nh_.subscribe("/peds_info", 2, &CallbackMQTT_PedInfo);    //DK

    //add
    Sub_globalPath = nh_.subscribe("/global_path", 1, CallbackGlobalPath);
    Sub_occMap = nh_.subscribe("occ_map", 1, CallbackHeightMap);
    Sub_distanceMap = nh_.subscribe("dist_map", 1, CallbackDistMap);
    Sub_localization = nh_.subscribe("/LocalizationData", 10, CallbackLocalizationData);
    // Sub_NaviInfo = nh_.subscribe("/navi_msg", 1, CallbackNaviInfo);


    
    MemberValInit(nh_);
    VisualizeValInit(); //DK
    cout << "START driving_function wow" << endl;
    ros::spin();
}

int main(int argc, char* argv[])
{
    try {
        DRIVINGFUNC_THREAD();
    }
    catch(std::exception& e) {
        std::cerr << "------------ error: " << e.what() << "\n";
        return 1;
    }
    catch(...) {
        std::cerr << "------------ Exception of unknown type!\n";
    }
    return 0;
} 


