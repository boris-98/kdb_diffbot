#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class OccupancyGridPublisher : public rclcpp::Node 
{
public:
	OccupancyGridPublisher() : Node("manual_map_publisher") 
	{
		this->declare_parameter<std::string>("map_file", "map.png");
		this->declare_parameter<double>("resolution", 0.05);
		this->declare_parameter<double>("origin_x", 0.0);
		this->declare_parameter<double>("origin_y", 0.0);

		std::string map_file = this->get_parameter("map_file").as_string();
		resolution_ = this->get_parameter("resolution").as_double();
		origin_x_ = this->get_parameter("origin_x").as_double();
		origin_y_ = this->get_parameter("origin_y").as_double();

		cv::Mat img = cv::imread(map_file, cv::IMREAD_GRAYSCALE);
		if (img.empty())
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to load map file: %s", map_file.c_str());
			rclcpp::shutdown();
			return;
		}

		width_ = img.cols;
		height_ = img.rows;

		map_data_.resize(width_ * height_);
		for (int y = 0; y < height_; ++y) {
	            for (int x = 0; x < width_; ++x) {
	                uint8_t pixel = img.at<uint8_t>(height_ - y - 1, x); // flip vertically
	                if (pixel == 255) 
				map_data_[y * width_ + x] = 100;	// occupied
	                else if (pixel == 0) 
				map_data_[y * width_ + x] = 0;		// free
	                else 
				map_data_[y * width_ + x] = -1;		// unknown
	            }
	        }

		og_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
		og_timer = this->create_wall_timer(500ms, std::bind(&OccupancyGridPublisher::og_callback, this));
		
	}

private:
	void og_callback()
	{
		auto occupancy_grid_msg = nav_msgs::msg::OccupancyGrid();

		occupancy_grid_msg.header.stamp = rclcpp::Clock().now();
		occupancy_grid_msg.header.frame_id = "base_link";
		occupancy_grid_msg.info.resolution = resolution_;
		occupancy_grid_msg.info.width = width_;
		occupancy_grid_msg.info.height = height_;
		occupancy_grid_msg.info.origin.position.x = origin_x_;
		occupancy_grid_msg.info.origin.position.y = origin_y_;
		occupancy_grid_msg.info.origin.orientation.w = 1.0;
		occupancy_grid_msg.data = map_data_;

		og_pub_->publish(occupancy_grid_msg);
	}

	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr og_pub_;
	rclcpp::TimerBase::SharedPtr og_timer;

	double resolution_, origin_x_, origin_y_;
	int width_, height_;
	std::vector<int8_t> map_data_;


};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OccupancyGridPublisher>());
	rclcpp::shutdown();
	return 0;
}
