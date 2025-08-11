#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <cmath>  // for std::hypot, std::ceil

class VelodyneToGrid {
public:
    VelodyneToGrid() {
        sub_ = nh_.subscribe("/velodyne_points", 1, &VelodyneToGrid::pointCloudCallback, this);
        pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_map", 1);

        // OccupancyGrid 기본 설정
        grid_.info.resolution = 0.1; // 10cm
        grid_.info.width = 100;      // 10m x 10m
        grid_.info.height = 100;
        grid_.info.origin.position.x = -5.0;
        grid_.info.origin.position.y = -5.0;
        grid_.info.origin.position.z = -0.7;
        grid_.data.resize(grid_.info.width * grid_.info.height, 0);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // OccupancyGrid 초기화
        std::fill(grid_.data.begin(), grid_.data.end(), 0);

        double z_threshold = -0.4;  // 바닥 필터링 기준

        for (const auto& pt : cloud.points) {
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;
            if (pt.z < z_threshold) continue;  // 바닥 제거

            int x_idx = static_cast<int>((pt.x - grid_.info.origin.position.x) / grid_.info.resolution);
            int y_idx = static_cast<int>((pt.y - grid_.info.origin.position.y) / grid_.info.resolution);

            if (x_idx >= 0 && x_idx < grid_.info.width &&
                y_idx >= 0 && y_idx < grid_.info.height) {
                int index = y_idx * grid_.info.width + x_idx;
                grid_.data[index] = 100;
            }
        }

        for (int x = 30; x <= 50; x++) {
            for (int y = 43; y <= 55; y++) {
                int index = y * grid_.info.width + x;
                grid_.data[index] = 0;
            }
        }


        // ---------------- 장애물 확장 코드 추가 ----------------
        std::vector<int8_t> original = grid_.data;  // 원본 복사
        double dilation_radius = 0.3;               // 확장 반경 (예: 30cm)
        int dilation_cells = std::ceil(dilation_radius / grid_.info.resolution);
        int width = grid_.info.width;
        int height = grid_.info.height;

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int idx = y * width + x;

                if (original[idx] == 100) {
                    for (int dy = -dilation_cells; dy <= dilation_cells; ++dy) {
                        for (int dx = -dilation_cells; dx <= dilation_cells; ++dx) {
                            int nx = x + dx;
                            int ny = y + dy;

                            if (nx < 0 || ny < 0 || nx >= width || ny >= height)
                                continue;

                            if (std::hypot(dx, dy) * grid_.info.resolution <= dilation_radius) {
                                int nidx = ny * width + nx;
                                if (grid_.data[nidx] != 100)
                                    grid_.data[nidx] = 99;  // 확장된 장애물 셀
                            }
                        }
                    }
                }
            }
        }
        // ------------------------------------------------------

        // 차량 자체를 비우기 (장애물 오인 방지)
        for (int x = 30; x <= 50; x++) {
            for (int y = 43; y <= 55; y++) {
                int index = y * grid_.info.width + x;
                grid_.data[index] = 0;
            }
        }

        grid_.header.stamp = ros::Time::now();
        grid_.header.frame_id = "velodyne";
        pub_.publish(grid_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    nav_msgs::OccupancyGrid grid_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velodyne_to_grid");
    VelodyneToGrid vtg;
    ros::spin();
    return 0;
}
