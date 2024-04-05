#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>   // M_PI와 std::fmod 함수를 사용하기 위한 헤더
#include <iomanip> // std::fixed와 std::setprecision 함수를 사용하기 위한 헤더
#include <limits>  // std::numeric_limits를 사용하기 위해 필요한 헤더

bool frame_flag = false;
double temp_range = 0.0;


class ScanConverter : public rclcpp::Node
{
public:
  ScanConverter()
      : Node("scan_converter")
  {
    // "/robot1/scan" 토픽으로부터 LaserScan 메시지를 구독합니다.
    // rclcpp::SensorDataQoS()는 센서 데이터에 적합한 QoS 프로필을 제공합니다.
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/robot1/scan", rclcpp::SensorDataQoS(), std::bind(&ScanConverter::topic_callback, this, std::placeholders::_1));

    // "/robot1/scaned_data" 토픽에 LaserScan 메시지를 발행하는 발행자를 생성합니다.
    // 발행 큐의 크기는 10으로 설정됩니다.
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/robot1/scaned_data", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // 받은 메시지를 복사하여 새로운 메시지 객체를 생성합니다.
    auto filtered_msg = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);

    // 복사된 메시지의 ranges와 intensities 배열을 클리어합니다.
    filtered_msg->ranges.clear();
    filtered_msg->intensities.clear();

    // RCLCPP_INFO(this->get_logger(), "Total data count: '%zu'", msg->ranges.size());

    std::ostringstream oss;                    // 문자열 스트림 생성
    oss << std::fixed << std::setprecision(3); // 소수점 아래 3자리로 고정

    // 스캔 데이터를 +180에서 -180도로 조정
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
      // 현재 각도를 계산하고 도로 변환
      double angle_rad = i * msg->angle_increment;
      double angle_deg = 180.0 - angle_rad * (180.0 / M_PI);

      // 각도를 +180에서 -180으로 조정
      if (angle_deg < -180)
      {
        angle_deg += 360;
      }
      // oss << "[" << angle_deg << "°, " << msg->ranges[i] << "m]";

      // if ((i + 1) % 10 == 0) // 10개 데이터마다 줄 바꿈
      // {
      //   oss << std::endl;
      // }

      if (frame_flag == false) //frame_flag == false -> 프로파일 없는 구간은 그냥 거리값 쓰겠음
      {
        temp_range = msg->ranges[i];
      }
      //frame_flag == true -> 프로파일 있는 구간은 프로파일 만나기 전 거리 그냥 쓰겠음
      if ((angle_deg > 125.0 && angle_deg < 135.0) || (angle_deg > 35.0 && angle_deg < 50.0) || (angle_deg > -135.0 && angle_deg < -125.0) || (angle_deg > -50.0 && angle_deg < -35.0))
      {
        frame_flag = true;
        filtered_msg->ranges.push_back(temp_range);
      }
      else
      {
        frame_flag = false;
        filtered_msg->ranges.push_back(temp_range);
      }

      if (!msg->intensities.empty())
      {
        filtered_msg->intensities.push_back(msg->intensities[i]);
      }
    }
    // RCLCPP_INFO(this->get_logger(), "\n%s", oss.str().c_str());
    // 필터링된 데이터 발행
    publisher_->publish(*filtered_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanConverter>());
  rclcpp::shutdown();
  return 0;
}
