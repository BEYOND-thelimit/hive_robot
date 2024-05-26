/*
 * motor_node.cpp
 *
 *      Author: Chis Chun
 */
#include <tutorial_ros2_motor/motor_node.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cmath>
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

//////////////////////////////////////////////////////parameter////////////////////////////////////////////////////////////////
float robot_wheel_radius = 0.0575; //바퀴 반지름
float robot_wheel_base = 0.4329; //바퀴사이 거리

int pwm_val1 = 0;   // PWM 값 초기화
int count_val1 = 0; // 타이머 호출 횟수를 세는 변수 초기화
int pwm_val2 = 0;   // PWM 값 초기화
int count_val2 = 0; // 타이머 호출 횟수를 세는 변수 초기화

int last_left_wheel_pwm = 0;
int last_right_wheel_pwm = 0;
bool last_left_dir = true;
bool last_right_dir = true;
bool zero_flag = true; //모터 반대방향 회전이 필요한 경우 0을 지나고 반대로 넘어가야 하는데 그 경우

float dt = 0.1;

//PID 확인을 위한 parameter
double target_rpm_value1 = 0.0;
double target_rpm_value2 = 0.0;
double real_rpm_value1 = 0.0;
double real_rpm_value2 = 0.0;

//PID
float P1 = 1.05;
float D1 = 0.2;
float P2 = 1.055;
float D2 = 0.2;
float last_error1 = 0.0;
float last_error2 = 0.0;
float pd_rpm1 = 0.0;
float pd_rpm2 = 0.0;

//odom
int pulse_1 = 0;
int pulse_2 = 0;
double now_left_wheel_pose_rad = 0.0; // rad
double now_left_wheel_pose = 0.0; // m = r*theta
double now_right_wheel_pose_rad = 0.0; // rad
double now_right_wheel_pose = 0.0;  // m = r*theta
double left_vel = 0.0; // m/s
double right_vel = 0.0; // m/s
double left_wheel_old_pos_ = 0.0; // m = r*theta
double right_wheel_old_pos_ = 0.0; // m = r*theta
double delta_distance = 0.0; 
double delta_theta = 0.0; 
double x_ = 0.0;
double y_ = 0.0;
double heading_ = 0.0;
double delta_linear = 0.0;
double delta_angular = 0.0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LoadParameters(void)
{
    std::ifstream inFile("/home/ubuntu/robot_ws/src/tutorial_ros2_motor/data/motor_input.txt");
    if (!inFile.is_open())
    {
        RCLCPP_ERROR(rclcpp::get_logger("motor_node"), "Unable to open the file");
        return;
    }

    int i = 0;
    std::size_t found;
    for (std::string line; std::getline(inFile, line);)
    {
        found = line.find("=");

        switch (i)
        {
        case 0:
            pwm_range = atof(line.substr(found + 2).c_str());
            break;
        case 1:
            pwm_frequency = atof(line.substr(found + 2).c_str());
            break;
        case 2:
            pwm_limit = atof(line.substr(found + 2).c_str());
            break;
        case 3:
            control_cycle = atof(line.substr(found + 2).c_str());
            break;
        case 4:
            acceleration_ratio = atof(line.substr(found + 2).c_str());
            break;
        case 5:
            wheel_radius = atof(line.substr(found + 2).c_str());
            break;
        case 6:
            robot_radius = atof(line.substr(found + 2).c_str());
            break;
        case 7:
            encoder_resolution = atof(line.substr(found + 2).c_str());
            break;
            // case :  = atof(line.substr(found+2).c_str()); break;
        }
        i += 1;
    }
    inFile.close();
}

int InitMotors(void)
{
    pinum = pigpio_start(NULL, NULL);

    if (pinum < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("motor_node"), "Setup failed");
        RCLCPP_ERROR(rclcpp::get_logger("motor_node"), "pinum is %d", pinum);
        return 1;
    }

    set_mode(pinum, motor1_dir, PI_OUTPUT);
    set_mode(pinum, motor2_dir, PI_OUTPUT);
    set_mode(pinum, motor1_pwm, PI_OUTPUT);
    set_mode(pinum, motor2_pwm, PI_OUTPUT);
    set_mode(pinum, motor1_encA, PI_INPUT);
    set_mode(pinum, motor1_encB, PI_INPUT);
    set_mode(pinum, motor2_encA, PI_INPUT);
    set_mode(pinum, motor2_encB, PI_INPUT);

    gpio_write(pinum, motor1_dir, PI_LOW);
    gpio_write(pinum, motor2_dir, PI_LOW);

    set_PWM_range(pinum, motor1_pwm, pwm_range);
    set_PWM_range(pinum, motor2_pwm, pwm_range);
    set_PWM_frequency(pinum, motor1_pwm, pwm_frequency);
    set_PWM_frequency(pinum, motor2_pwm, pwm_frequency);
    set_PWM_dutycycle(pinum, motor1_pwm, 0);
    set_PWM_dutycycle(pinum, motor1_pwm, 0);

    set_pull_up_down(pinum, motor1_encA, PI_PUD_UP);
    set_pull_up_down(pinum, motor1_encB, PI_PUD_UP);
    set_pull_up_down(pinum, motor2_encA, PI_PUD_UP);
    set_pull_up_down(pinum, motor2_encB, PI_PUD_UP);

    current_pwm1 = 0;
    current_pwm2 = 0;

    current_direction1 = true;
    current_direction2 = true;

    acceleration = pwm_limit / (acceleration_ratio);

    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "Setup Fin");
    return 0;
}

void SetInterrupts(void)
{
    callback(pinum, motor1_encA, EITHER_EDGE, Interrupt1A);
    callback(pinum, motor1_encB, EITHER_EDGE, Interrupt1B);
    callback(pinum, motor2_encA, EITHER_EDGE, Interrupt2A);
    callback(pinum, motor2_encB, EITHER_EDGE, Interrupt2B);
}

void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
    (void)pi;
    (void)user_gpio;
    (void)level;
    (void)tick;
    if (gpio_read(pinum, motor1_dir) == true)
        encoder_count_1A--;
    else
        encoder_count_1A++;
    speed_count_1++;
}

void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
    (void)pi;
    (void)user_gpio;
    (void)level;
    (void)tick;
    if (gpio_read(pinum, motor1_dir) == true)
        encoder_count_1B--;
    else
        encoder_count_1B++;
    speed_count_1++;
}

void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
    (void)pi;
    (void)user_gpio;
    (void)level;
    (void)tick;
    if (gpio_read(pinum, motor2_dir) == true)
        encoder_count_2A--;
    else
        encoder_count_2A++;
    speed_count2++;
}

void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
    (void)pi;
    (void)user_gpio;
    (void)level;
    (void)tick;
    if (gpio_read(pinum, motor2_dir) == true)
        encoder_count_2B--;
    else
        encoder_count_2B++;
    speed_count2++;
}

int SumMotor1Encoder()
{
    encoder_count_1 = encoder_count_1A + encoder_count_1B;
    return encoder_count_1;
}

int SumMotor2Encoder()
{
    encoder_count_2 = encoder_count_2A + encoder_count_2B;
    return encoder_count_2;
}

void InitEncoders(void)
{
    encoder_count_1 = 0;
    encoder_count_2 = 0;
    encoder_count_1A = 0;
    encoder_count_1B = 0;
    encoder_count_2A = 0;
    encoder_count_2B = 0;
}

void Initialize(void)
{
    LoadParameters();
    InitMotors();
    InitEncoders();
    SetInterrupts();

    wheel_round = 2 * PI * wheel_radius;
    robot_round = 2 * PI * robot_radius;

    switch_direction = true;
    theta_distance_flag = 0;

    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pwm_range %d", pwm_range);
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pwm_frequency %d", pwm_frequency);
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pwm_limit %d", pwm_limit);
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "control_cycle %f", control_cycle);
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "acceleration_ratio %d", acceleration_ratio);
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "Initialize Complete");

    printf("\033[2J");
}

void MotorController(int motor_num, bool direction, int pwm)
{
    int local_pwm = LimitPwm(pwm);

    if (motor_num == 1)
    {
        if (direction == true)
        {
            gpio_write(pinum, motor1_dir, PI_LOW);
            set_PWM_dutycycle(pinum, motor1_pwm, local_pwm);
            current_pwm1 = local_pwm;
            current_direction1 = true;
        }
        else if (direction == false)
        {
            gpio_write(pinum, motor1_dir, PI_HIGH);
            set_PWM_dutycycle(pinum, motor1_pwm, local_pwm);
            current_pwm1 = local_pwm;
            current_direction1 = false;
        }
    }

    else if (motor_num == 2)
    {
        if (direction == true)
        {
            gpio_write(pinum, motor2_dir, PI_LOW);
            set_PWM_dutycycle(pinum, motor2_pwm, local_pwm);
            current_pwm2 = local_pwm;
            current_direction2 = true;
        }
        else if (direction == false)
        {
            gpio_write(pinum, motor2_dir, PI_HIGH);
            set_PWM_dutycycle(pinum, motor2_pwm, local_pwm);
            current_pwm2 = local_pwm;
            current_direction2 = false;
        }
    }
}
void AccelController(int motor_num, bool direction, int desired_pwm)
{
    const int MAX_PWM_CHANGE = 80;  // 최대 PWM 변경량
    if(motor_num == 1)
    {
        if(direction == last_left_dir) //이전과 같은 방향
        {
            int pwm_chage = desired_pwm - last_left_wheel_pwm; //같은 방향이라면 무조건 양수임
            if(pwm_chage >= 0) //가속
            {
                if(pwm_chage > MAX_PWM_CHANGE)
                {
                    desired_pwm = last_left_wheel_pwm + MAX_PWM_CHANGE;
                }
            }
            else //감속
            {
                if(abs(pwm_chage)>MAX_PWM_CHANGE)
                {
                    desired_pwm = last_left_wheel_pwm - MAX_PWM_CHANGE;
                }
            }
        }
        else //역방향
        {
            //0까지 만들고 반대 방향으로 전환
            zero_flag = true;
            if(zero_flag == true && last_left_wheel_pwm != 0)
            {
                if(last_left_wheel_pwm - MAX_PWM_CHANGE > 0)
                {
                    desired_pwm = last_left_wheel_pwm - MAX_PWM_CHANGE;
                    direction = !direction;
                }
                else{
                    desired_pwm = 0;
                    zero_flag = false;
                }
            }
            else if(zero_flag == true && last_left_wheel_pwm == 0)
            {
                desired_pwm = 0;
                zero_flag = false;
            }
        }
        MotorController(motor_num, direction, desired_pwm);
        last_left_dir = direction;
        last_left_wheel_pwm = desired_pwm;
    }
    else if(motor_num == 2)
    {
        if(direction == last_right_dir) //이전과 같은 방향
        {
            int pwm_chage = desired_pwm - last_right_wheel_pwm; //같은 방향이라면 무조건 양수임
            if(pwm_chage >= 0) //가속
            {
                if(pwm_chage > MAX_PWM_CHANGE)
                {
                    desired_pwm = last_right_wheel_pwm + MAX_PWM_CHANGE;
                }
            }
            else //감속
            {
                if(abs(pwm_chage)>MAX_PWM_CHANGE)
                {
                    desired_pwm = last_right_wheel_pwm - MAX_PWM_CHANGE;
                }
            }
        }
        else //역방향
        {
            //0까지 만들고 반대 방향으로 전환
            zero_flag = true;
            if(zero_flag == true && last_right_wheel_pwm != 0)
            {
                if(last_right_wheel_pwm - MAX_PWM_CHANGE > 0)
                {
                    desired_pwm = last_right_wheel_pwm - MAX_PWM_CHANGE;
                    direction = !direction;
                }
                else{
                    desired_pwm = 0;
                    zero_flag = false;
                }
            }
            else if(zero_flag == true && last_left_wheel_pwm == 0)
            {
                desired_pwm = 0;
                zero_flag = false;
            }
        }
        MotorController(motor_num, direction, desired_pwm);
        last_right_dir = direction;
        last_right_wheel_pwm = desired_pwm;
    }
}


int LimitPwm(int pwm)
{
    int output;
    if (pwm > pwm_limit * 2)
    {
        output = pwm_limit;
        RCLCPP_WARN(rclcpp::get_logger("motor_node"), "pwm too fast!!!");
    }
    else if (pwm > pwm_limit)
        output = pwm_limit;
    else if (pwm < 0)
    {
        output = 0;
        RCLCPP_WARN(rclcpp::get_logger("motor_node"), "trash value!!!");
    }
    else
        output = pwm;
    return output;
}

void CalculateRpm()
{
    rpm_value1 = (speed_count_1 * (60 * control_cycle)) / (encoder_resolution * 4);
    speed_count_1 = 0;
    rpm_value2 = (speed_count2 * (60 * control_cycle)) / (encoder_resolution * 4);
    speed_count2 = 0;
}

void Testencoder()
{
    if (SumMotor1Encoder() < 6156)
    {
        MotorController(1, false, 100);
    }
    else
    {
        MotorController(1, true, 0);
    }
}

void Calculate_Odom()
{
  pulse_1 = SumMotor1Encoder();
  pulse_2 = SumMotor2Encoder();

  now_left_wheel_pose_rad = (pulse_1 * 2 * M_PI) / (4 * encoder_resolution);
  now_left_wheel_pose = robot_wheel_radius * now_left_wheel_pose_rad;

  now_right_wheel_pose_rad = (pulse_2 * 2 * M_PI) / (4 * encoder_resolution);
  now_right_wheel_pose = robot_wheel_radius * now_right_wheel_pose_rad;

  left_vel = (now_left_wheel_pose - left_wheel_old_pos_);
  right_vel = (now_right_wheel_pose - right_wheel_old_pos_);

  left_wheel_old_pos_ = now_left_wheel_pose;
  right_wheel_old_pos_ = now_right_wheel_pose;

  delta_distance = (left_vel + right_vel) / 2.0;
  delta_theta = (right_vel - left_vel) / robot_wheel_base;

  x_ += delta_distance * cos(heading_);
  y_ += delta_distance * sin(heading_);
  heading_ += delta_theta;

  delta_linear = delta_distance / dt;
  delta_angular = delta_theta / dt;
}

void InfoMotors()
{
    CalculateRpm();
    printf("\033[2J");
    printf("\033[1;1H");
    printf("Encoder1A : %5d    ||  Encoder2A : %5d\n", encoder_count_1A, encoder_count_2A);
    printf("Encoder1B : %5d    ||  Encoder2B : %5d\n", encoder_count_1B, encoder_count_2B);
    printf("RPM1 : %10.0f    ||  RPM2 : %10.0f\n", rpm_value1, rpm_value2);
    printf("T_RPM1 : %8.0f    ||  T_RPM2 : %8.0f\n", target_rpm_value1, target_rpm_value2);
    printf("PWM1 : %10.0d    ||  PWM2 : %10.0d\n", current_pwm1, current_pwm2);
    printf("DIR1 :%11s    ||  DIR2 :%11s\n", current_direction1 ? "CW" : "CCW", current_direction2 ? "CW" : "CCW");
    printf("ACC  :%11.0d\n", acceleration);
    printf("x : %10.2f    ||  y : %10.2f    ||  heading : %10.2f\n", x_, y_, heading_);
    printf("\n");
    real_rpm_value1 = current_direction1 ? rpm_value1 : -1*rpm_value1;
    real_rpm_value2 = current_direction2 ? rpm_value2 : -1*rpm_value2;
}



class RosCommunicator : public rclcpp::Node
{
public:
    RosCommunicator(const std::string &robot_name)
        : Node(robot_name + "_motor"), robot_name_(robot_name)
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&RosCommunicator::TimerCallback, this));
        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>("/" + robot_name_ + "/cmd_vel", 10, std::bind(&RosCommunicator::cmdVelCallback, this, std::placeholders::_1));
        rpm_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/" + robot_name_ + "/rpm", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/" + robot_name_ + "/odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    int M1RPMtoPWM(float &rpm)
    {
        if(abs(rpm) > max_rpm)
        {
            rpm = max_rpm;
        }
        else
        {
            rpm = abs(rpm);
        }
        float a = 8.502310410437618;
        float b = 22.200326175591222;
        int return_value = round(a * rpm + b);
        if(rpm <= deadzone) //너무 작은 rpm은 deadzone으로 잘라냄
        {
            return 0;
        }
        else
        {
            return return_value;
        }
    }

    int M2RPMtoPWM(float &rpm)
    {
        if(abs(rpm) > max_rpm)
        {
            rpm = max_rpm;
        }
        else
        {
            rpm = abs(rpm);
        }
        rpm = abs(rpm);
        float a = 8.387583359796757;
        float b = 25.50571610034933;
        int return_value = round(a * rpm + b);
        if(rpm <= deadzone) //너무 작은 rpm은 deadzone으로 잘라냄
        {
            return 0;
        }
        else
        {
            return return_value;
        }
    }

    void PDControl(float &P, float &D, float &target_rpm, float real_rpm, float &last_error, float &pd_rpm, bool &dir)
    {
        float error = target_rpm - int(dir)*real_rpm;
        float error_gap = (error - last_error)*dt;
        last_error = error;
        if(abs(target_rpm) <= deadzone)
        {
            pd_rpm = 0.0;
        }
        else
        {
            pd_rpm = real_rpm + P * error + D * error_gap;
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        linear_velocity = msg->linear.x;
        angular_velocity = msg->angular.z;
        left_wheel_velocity = linear_velocity - (angular_velocity * robot_wheel_base / 2);
        right_wheel_velocity = linear_velocity + (angular_velocity * robot_wheel_base / 2);
        left_wheel_rpm = (left_wheel_velocity / (2 * M_PI * robot_wheel_radius)) * 60;
        right_wheel_rpm = (right_wheel_velocity / (2 * M_PI * robot_wheel_radius)) * 60; 

        if (left_wheel_rpm >= 0) //전진인지 후진인지 확인
            left_dir = true;
        else if (left_wheel_rpm < 0)
            left_dir = false;

        if (right_wheel_rpm >= 0)
            right_dir = true;
        else if (right_wheel_rpm < 0)
            right_dir = false;
        
        //target RPM
        target_rpm_value1 = left_wheel_rpm;
        target_rpm_value2 = right_wheel_rpm;

        // //linear regression을 통한 예측값
        left_wheel_pwm = M1RPMtoPWM(left_wheel_rpm); //cmd_vel을 내기 위해 왼쪽 바퀴에 들어가야 할 input pwm
        right_wheel_pwm = M2RPMtoPWM(right_wheel_rpm); //cmd_vel을 내기 위해 오른쪽 바퀴에 들어가야 할 input pwm

        // PDControl(P1,D1,left_wheel_rpm,float(rpm_value1),last_error1,pd_rpm1,current_direction1);
        // PDControl(P2,D2,right_wheel_rpm,float(rpm_value2),last_error2,pd_rpm2,current_direction2);
        // if (pd_rpm1 >= 0) //전진인지 후진인지 확인
        //     left_dir = true;
        // else if (pd_rpm1 < 0)
        //     left_dir = false;
        // if (pd_rpm2 >= 0)
        //     right_dir = true;
        // else if (pd_rpm2 < 0)
        //     right_dir = false;
        // // //linear regression을 통한 예측값
        // left_wheel_pwm = M1RPMtoPWM(pd_rpm1); //cmd_vel을 내기 위해 왼쪽 바퀴에 들어가야 할 input pwm
        // right_wheel_pwm = M2RPMtoPWM(pd_rpm2); //cmd_vel을 내기 위해 오른쪽 바퀴에 들어가야 할 input pwm

    }

    void PWMCheck()
    {
        count_val1++; // 타이머 호출 횟수 증가
        if (count_val1 % 30 == 0)
        { // 0.1초 * 30 = 3초마다 실행
            if (pwm_val1 < 400)
            {
                pwm_val1 += 10; // PWM 값을 10 증가
            }
            else
            {
                pwm_val1 = 0; // PWM 값을 400에 도달하면 0으로 리셋
            }
            AccelController(1, true, pwm_val1); // 모터 컨트롤러에 PWM 값 전달
        }

        count_val2++; // 타이머 호출 횟수 증가
        if (count_val2 % 30 == 0)
        { // 0.1초 * 30 = 2초마다 실행
            if (pwm_val2 < 400)
            {
                pwm_val2 += 10; // PWM 값을 10 증가
            }
            else
            {
                pwm_val2 = 0; // PWM 값을 400에 도달하면 0으로 리셋
            }
            AccelController(2, true, pwm_val2); // 모터 컨트롤러에 PWM 값 전달
        }
    }

    void PublishRpm()
    {
        std_msgs::msg::Float64MultiArray rpm_msg;
        rpm_msg.data.resize(4); // 배열 크기 설정

        rpm_msg.data[0] = target_rpm_value1;  // 목표 RPM1
        rpm_msg.data[1] = target_rpm_value2;  // 목표 RPM2
        rpm_msg.data[2] = real_rpm_value1;    // 현재 RPM1
        rpm_msg.data[3] = real_rpm_value2;    // 현재 RPM2

        rpm_pub_->publish(rpm_msg);
    }


    void PublishOdometry()
    {
        auto current_time = this->get_clock()->now();

        // nav_msgs::msg::Odometry 타입의 odom_msg 메시지 객체를 생성합니다.
        auto odom_msg = nav_msgs::msg::Odometry();

        // 메시지의 헤더 정보를 설정합니다. 
        // 현재 시간을 타임스탬프로 설정하고, frame_id와 child_frame_id를 설정합니다.
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = robot_name_ + "_odom";
        odom_msg.child_frame_id = robot_name_ + "_base_link";

        // 로봇의 현재 위치 정보를 설정합니다.
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        // heading_ 값을 사용해 로봇의 방향을 quaternion으로 변환합니다.
        tf2::Quaternion q;
        q.setRPY(0, 0, heading_); // 롤, 피치, 요 값으로 quaternion 설정
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        // 로봇의 속도 정보를 설정합니다.
        odom_msg.twist.twist.linear.x = delta_linear;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.angular.z = delta_angular;

        // odom_pub_ 퍼블리셔를 통해 odom_msg 메시지를 퍼블리시합니다.
        odom_pub_->publish(odom_msg);

        // TransformStamped 메시지를 생성하여 tf 변환을 설정합니다 (robot1_odom -> robot1_base_link).
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = current_time;
        transform_stamped.header.frame_id = robot_name_ + "_odom";
        transform_stamped.child_frame_id = robot_name_ + "_base_link";
        transform_stamped.transform.translation.x = x_;
        transform_stamped.transform.translation.y = y_;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        // tf_broadcaster_ 객체를 사용해 transform_stamped 메시지를 브로드캐스트합니다.
        tf_broadcaster_->sendTransform(transform_stamped);

        // TransformStamped 메시지를 생성하여 tf 변환을 설정합니다 (world -> robot1_odom).
        geometry_msgs::msg::TransformStamped world_to_odom;
        world_to_odom.header.stamp = current_time;
        world_to_odom.header.frame_id = "world";
        world_to_odom.child_frame_id = robot_name_ + "_odom";
        world_to_odom.transform.translation.x = 0.0; // 원하는 위치로 설정
        world_to_odom.transform.translation.y = 0.0; // 원하는 위치로 설정
        world_to_odom.transform.translation.z = 0.0; // 원하는 위치로 설정
        world_to_odom.transform.rotation.x = 0.0;
        world_to_odom.transform.rotation.y = 0.0;
        world_to_odom.transform.rotation.z = 0.0;
        world_to_odom.transform.rotation.w = 1.0;

        // tf_broadcaster_ 객체를 사용해 world_to_odom 메시지를 브로드캐스트합니다.
        tf_broadcaster_->sendTransform(world_to_odom);

        // TransformStamped 메시지를 생성하여 tf 변환을 설정합니다 (robot1_base_link -> robot1_lidar).
        geometry_msgs::msg::TransformStamped base_to_lidar;
        base_to_lidar.header.stamp = current_time;
        base_to_lidar.header.frame_id = robot_name_ + "_base_link";
        base_to_lidar.child_frame_id = robot_name_ + "_lidar";
        base_to_lidar.transform.translation.x = -0.1049; // LiDAR의 x 축 위치
        base_to_lidar.transform.translation.y = 0.0; // LiDAR의 y 축 위치
        base_to_lidar.transform.translation.z = 0.4435; // LiDAR의 z 축 위치
        base_to_lidar.transform.rotation.x = 0.0;
        base_to_lidar.transform.rotation.y = 0.0;
        base_to_lidar.transform.rotation.z = 0.0;
        base_to_lidar.transform.rotation.w = 1.0;

        // tf_broadcaster_ 객체를 사용해 base_to_lidar 메시지를 브로드캐스트합니다.
        tf_broadcaster_->sendTransform(base_to_lidar);

        // TransformStamped 메시지를 생성하여 tf 변환을 설정합니다 (robot1_base_link -> robot1_imu).
        geometry_msgs::msg::TransformStamped base_to_imu;
        base_to_imu.header.stamp = current_time;
        base_to_imu.header.frame_id = robot_name_ + "_base_link";
        base_to_imu.child_frame_id = robot_name_ + "_imu";
        base_to_imu.transform.translation.x = 0.0901; // IMU의 x 축 위치
        base_to_imu.transform.translation.y = 0.0; // IMU의 y 축 위치
        base_to_imu.transform.translation.z = 0.4385; // IMU의 z 축 위치

        // x 축을 기준으로 pi만큼 회전 (180도 회전)
        tf2::Quaternion imu_q;
        imu_q.setRPY(M_PI, 0, 0);
        base_to_imu.transform.rotation.x = imu_q.x();
        base_to_imu.transform.rotation.y = imu_q.y();
        base_to_imu.transform.rotation.z = imu_q.z();
        base_to_imu.transform.rotation.w = imu_q.w();

        // tf_broadcaster_ 객체를 사용해 base_to_imu 메시지를 브로드캐스트합니다.
        tf_broadcaster_->sendTransform(base_to_imu);
    }


    void TimerCallback()
    {
        // PWMCheck();
        AccelController(1,left_dir,left_wheel_pwm);
        AccelController(2,right_dir,right_wheel_pwm);
        PublishRpm();
        Calculate_Odom();
        PublishOdometry();
        InfoMotors();
    }

private:
    std::string robot_name_;
    float linear_velocity = 0.0;
    float angular_velocity = 0.0;
    float left_wheel_velocity = 0.0;
    float right_wheel_velocity = 0.0;
    float left_wheel_rpm = 0.0;
    float right_wheel_rpm = 0.0;
    int left_wheel_pwm = 0;
    int right_wheel_pwm = 0;
    bool left_dir = true;
    bool right_dir = true;
    float deadzone = 2.5;
    int max_rpm = 42;


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rpm_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    Initialize();
    std::string robot_name = "robot1"; // replace with "robot2", "robot3", etc., as needed
    auto communicator = std::make_shared<RosCommunicator>(robot_name);
    rclcpp::spin(communicator);
    rclcpp::shutdown();
    MotorController(1, true, 0);
    MotorController(2, true, 0);
    return 0;
}
