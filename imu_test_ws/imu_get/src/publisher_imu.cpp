#include <chrono>
#include <math.h>
#include "serial/serial.h" //前面安装的ROS2内置串口包
#include <memory.h>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
//#include "geometry_msgs/msg/detail/vector3__struct.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "transform.hpp"

serial::Serial ser;

using namespace std::chrono_literals;

class publisher_imu_node : public rclcpp::Node// 节点命名与类最好一致
{
public:
    std::string port;
    int baudrate;
    std::string imu_topic;
    std::string imu_offline_topic;
    transform_imu imu_fetch; // 初始值设为0
public:
    publisher_imu_node()
        : Node("publisher_imu_node")
    {
        int output_hz = 20; // 频率，看传感器说明书
        // timer_ms = millisecond(output_hz);
        std::chrono::milliseconds timer_ms{output_hz}; //  换算成ms
        port = "/dev/lidarIMU";                         // 串口号，主机可查看
        baudrate = 9600;                               // 波特率，imu在波特率115200返回数据时间大概是1ms,9600下大概是10ms，看imu参数设置
        try
        {
            ser.setPort(port);
            ser.setBaudrate(baudrate);
            serial::Timeout to = serial::Timeout::simpleTimeout(500);
            ser.setTimeout(to);
            ser.open();
        }
        catch (serial::IOException &e)
        {
            RCLCPP_INFO(this->get_logger(), "Unable to open port ");
            return;
        }

        if (ser.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "Serial Port initialized");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Serial Port ???");
            return;
        }

        pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("/imu_data", 20);
        //<sensor_msgs::msg::Imu>消息数据类型可自行查看对应文件，消息队列长度设20
        pub_imu_offline = this->create_publisher<sensor_msgs::msg::Imu>("/imu_offline_data", 20);
        // 这里创建了两个话题，一个是/imu_data，一个是/imu_offline_data

        // imu = transform_imu();//初始化对象
        //  ser.flush();
        //  int size;
        printf("Process working_1\n");
        timer_ = this->create_wall_timer(500ms, std::bind(&publisher_imu_node::timer_callback, this));//std::chrono::milliseconds timer_ms{output_hz}; //  换算成ms
        printf("Process working_2\n");   
    }

public:
    void timer_callback()
    {
        int count = ser.available(); // 读取到缓存区数据的字节数
        if (count != 0)
        {
            // ROS_INFO_ONCE("Data received from serial port.");
            int num;
            rclcpp::Time now = this->get_clock()->now(); // 获取时间戳
            std::vector<unsigned char> read_buf(count);//这里定义无符号，是应为read函数的形参是无符号
            //unsigned char read_buf[count];   // 开辟数据缓冲区，注意这里是无符号类型
            num = ser.read(&read_buf[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数

            std::vector<char> read_buf_char(count);//vector转char类型
            for(int i = 0;i < count;i++)
            {
                read_buf_char[i] = (char)read_buf[i];
            }
            imu_fetch.FetchData(read_buf_char, num);
            sensor_msgs::msg::Imu imu_data;         //
            sensor_msgs::msg::Imu imu_offline_data; //
            //------------------imu data----------------
            imu_data.header.stamp = now;
            imu_data.header.frame_id = "imu_frame";

            imu_data.linear_acceleration.x = imu_fetch.acc.x;
            imu_data.linear_acceleration.y = imu_fetch.acc.y;
            imu_data.linear_acceleration.z = imu_fetch.acc.z;

            imu_data.angular_velocity.x = imu_fetch.angle.r * 180.0 / M_PI;
            imu_data.angular_velocity.y = imu_fetch.angle.p * 180.0 / M_PI;
            imu_data.angular_velocity.z = imu_fetch.angle.y * 180.0 / M_PI;//数据结构里没有储存欧拉角的变量名称，用angular_velocity.z凑合

            tf2::Quaternion curr_quater;
            curr_quater.setRPY(imu_fetch.angle.r, imu_fetch.angle.p, imu_fetch.angle.y);
            // 欧拉角换算四元数
            RCLCPP_INFO(this->get_logger(), "Publishing: '");
            //RCLCPP_INFO(this->get_logger(), "angle: x=%f, y=%f, z=%f",imu.angle.r, imu.angle.p, imu.angle.y);

            imu_data.orientation.x = curr_quater.x();
            imu_data.orientation.y = curr_quater.y();
            imu_data.orientation.z = curr_quater.z();
            imu_data.orientation.w = curr_quater.w();
            // RCLCPP_INFO(this->get_logger(), "Quaternion: x=%f, y=%f, z=%f, w=%f",
            //   imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);

            //---------------imu offline data--------------
            imu_offline_data.header.stamp = now;
            // imu_offline_data.header.frame_id = imu_frame;

            imu_offline_data.linear_acceleration.x = imu_fetch.acc.x;
            imu_offline_data.linear_acceleration.y = imu_fetch.acc.y;
            imu_offline_data.linear_acceleration.z = imu_fetch.acc.z;

            imu_offline_data.angular_velocity.x = imu_fetch.gyro.x;
            imu_offline_data.angular_velocity.y = imu_fetch.gyro.y;
            imu_offline_data.angular_velocity.z = imu_fetch.gyro.z;

            imu_offline_data.orientation.x = curr_quater.x();
            imu_offline_data.orientation.y = curr_quater.y();
            imu_offline_data.orientation.z = curr_quater.z();
            imu_offline_data.orientation.w = curr_quater.w();

            pub_imu->publish(imu_data);
            pub_imu_offline->publish(imu_offline_data); // 发布话题，往两个话题放数据
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_offline;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<publisher_imu_node>()); // 单线程，调用所有可触发的回调函数，将进入循环，不会返回
    printf("Process exit\n");
    rclcpp::shutdown();
    return 0;
}

