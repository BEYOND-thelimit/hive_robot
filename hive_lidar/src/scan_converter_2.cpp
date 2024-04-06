#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <stdio.h>
#include <vector>

class ScanConverter2 : public rclcpp::Node
{
 public:
  ScanConverter2(/* args */);
  ~ScanConverter2();
  void sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

 private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

ScanConverter2::ScanConverter2(/* args */) : Node("scan_converter2")
{
  subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/robot1/scan", rclcpp::SensorDataQoS(), std::bind(&ScanConverter2::sub_callback, this, std::placeholders::_1));
  publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/robot1/converted_scan", 10);
}

ScanConverter2::~ScanConverter2()
{
}

void ScanConverter2::sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // converted_msg 객체 생성
  auto converted_msg = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
  converted_msg->ranges.clear();
  converted_msg->intensities.clear();

  // 프로파일에 해당하는 range값 -1로 변경
  float temp_scan[msg->ranges.size()];
  float threshold = 0.35; // 프로파일까지 거리
  std::vector<int> first_profile_index;
  std::vector<int> second_profile_index;
  std::vector<int> third_profile_index;
  std::vector<int> fourth_profile_index;
  int profile_number = -1;
  int flag = 0;
  for (size_t i = 0; i < msg->ranges.size(); i++)
  {
    temp_scan[i] = msg->ranges[i];
    if (temp_scan[i] < threshold)
    {
      temp_scan[i] = -1;
    }
    if (i>0 && temp_scan[i] < threshold && temp_scan[i-1]>threshold)
    {
      flag = 1;
      profile_number++;
    }
    else if (i>0 && temp_scan[i] > threshold && temp_scan[i-1]<threshold)
    {
      flag = 0;
    }

    if (flag==1 && profile_number==0)
    {
      first_profile_index.push_back(i);
    }
    else if (flag==1 && profile_number==1)
    {
      second_profile_index.push_back(i);
    }
    else if (flag==1 && profile_number==2)
    {
      third_profile_index.push_back(i);
    }
    else if (flag==1 && profile_number==3)
    {
      fourth_profile_index.push_back(i);
    }
  }
  // 프로파일 인덱스 시작과 끝의 전 후 인덱스
  int lower_1_index= first_profile_index.front()-1;
  int upper_1_index= first_profile_index.back()+1;
  int lower_2_index = second_profile_index.front()-1;
  int upper_2_index = second_profile_index.back()+1;
  int lower_3_index = third_profile_index.front()-1;
  int upper_3_index = third_profile_index.back()+1;
  int lower_4_index = fourth_profile_index.front()-1;
  int upper_4_index = fourth_profile_index.back()+1;

  float range_threshold = 0.5;
  float min_max_range_in_1 = abs(temp_scan[lower_1_index] - temp_scan[upper_1_index]);
  float min_max_range_in_2 = abs(temp_scan[lower_2_index] - temp_scan[upper_2_index]);
  float min_max_range_in_3 = abs(temp_scan[lower_3_index] - temp_scan[upper_3_index]);
  float min_max_range_in_4 = abs(temp_scan[lower_4_index] - temp_scan[upper_4_index]);

  // first profile
  if ( min_max_range_in_1 < range_threshold)
  {
    float one_step_in_1 = min_max_range_in_1 / first_profile_index.size();
    for (int i = 0; i < first_profile_index.size(); i++)
    {
      if (temp_scan[lower_1_index] <= temp_scan[upper_1_index])
      {
        temp_scan[first_profile_index[i]] = temp_scan[lower_1_index] + one_step_in_1 * i;
      }
      else
      {
        temp_scan[first_profile_index[i]] = temp_scan[lower_1_index] - one_step_in_1 * i;
      }
    }
  }
  else
  {
    for (int i = 0; i < first_profile_index.size(); i++)
    {
      temp_scan[first_profile_index[i]] = -1;
    }
  }
  // second profile
  if ( min_max_range_in_2 < range_threshold)
  {
    float one_step_in_2 = min_max_range_in_2 / second_profile_index.size();
    for (int i = 0; i < second_profile_index.size(); i++)
    {
      if (temp_scan[lower_2_index] <= temp_scan[upper_2_index])
      {
        temp_scan[second_profile_index[i]] = temp_scan[lower_2_index] + one_step_in_2 * i;
      }
      else
      {
        temp_scan[second_profile_index[i]] = temp_scan[lower_2_index] - one_step_in_2 * i;
      }
    }
  }
  else
  {
    for (int i = 0; i < second_profile_index.size(); i++)
    {
      temp_scan[second_profile_index[i]] = -1;
    }
  }
  // third profile
  if ( min_max_range_in_3 < range_threshold)
  {
    float one_step_in_3 = min_max_range_in_3 / third_profile_index.size();
    for (int i = 0; i < third_profile_index.size(); i++)
    {
      if (temp_scan[lower_3_index] <= temp_scan[upper_3_index])
      {
        temp_scan[third_profile_index[i]] = temp_scan[lower_3_index] + one_step_in_3 * i;
      }
      else
      {
        temp_scan[third_profile_index[i]] = temp_scan[lower_3_index] - one_step_in_3 * i;
      }
    }
  }
  else
  {
    for (int i = 0; i < third_profile_index.size(); i++)
    {
      temp_scan[third_profile_index[i]] = -1;
    }
  }
  // fourth profile
  if ( min_max_range_in_4 < range_threshold)
  {
    float one_step_in_4 = min_max_range_in_4 / fourth_profile_index.size();
    for (int i = 0; i < fourth_profile_index.size(); i++)
    {
      if (temp_scan[lower_4_index] <= temp_scan[upper_4_index])
      {
        temp_scan[fourth_profile_index[i]] = temp_scan[lower_4_index] + one_step_in_4 * i;
      }
      else
      {
        temp_scan[fourth_profile_index[i]] = temp_scan[lower_4_index] - one_step_in_4 * i;
      }
    }
  }
  else
  {
    for (int i = 0; i < fourth_profile_index.size(); i++)
    {
      temp_scan[fourth_profile_index[i]] = -1;
    }
  }
  for (int i = 0; i < msg->ranges.size(); i++)
  {
    converted_msg->ranges.push_back(temp_scan[i]);
  }
  // 필터링된 데이터 발행
  publisher_->publish(*converted_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanConverter2>());
  rclcpp::shutdown();
  return 0;
}
