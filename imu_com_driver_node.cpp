#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_driver_msgs/msg/imu_hwt9073can.hpp"
#include <serial/serial.h>
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include <string>
#include <sstream>
#include <cstdio>
#include <chrono>

/*驱动一台维特智能HWT9073-485惯性测量元件*/
class imuCOM : public rclcpp::Node{
 public:
  // 构造函数,有一个参数为节点名称
  explicit imuCOM(std::string name) : Node(name){
    // 创建发布
    rawData_publisher_ = this->create_publisher<std_msgs::msg::String>("/Sensor_msgs/IMU_Lidar/rawData", 10);
    ImuData_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/Sensor_msgs/IMU_Lidar/Imu", 10);
    ImuHwtData_publisher_ = this->create_publisher<sensor_driver_msgs::msg::ImuHwt9073can>("/Sensor_msgs/IMU_Lidar/ImuHwt9073can", 10);
    
    /*声明参数*/
    this->declare_parameter("baudrate_IMU", 230400);
    this->declare_parameter("IMU_ComName", "/dev/lidarIMU"); 
    this->declare_parameter("gravity_acc", 9.8); 
    this->declare_parameter("imu_frame_id", "/lidar_front/imu"); 
    
    /*获取参数*/
    this->get_parameter("baudrate_IMU", baudrate_IMU);
    this->get_parameter("IMU_ComName", ComName);
    this->get_parameter("gravity_acc", gravity_acc);
    this->get_parameter("imu_frame_id", imu_frame_id);
    Imu_data.header.frame_id = imu_frame_id;
    ImuHwt_data.header.frame_id = imu_frame_id;
    std::stringstream ss;
    ss.str("");
    ss << "\nbaudrate_IMU = " << baudrate_IMU;
    ss << "\nIMU_ComName = " << ComName;
    ss << "\ngravity_acc = " << gravity_acc << " m/s^2";
    std::string print_str = ss.str();
    RCLCPP_INFO(this->get_logger(), print_str.c_str());
    
    /*创建timeout*/
    serial::Timeout ComSerialTimeOut = serial::Timeout::simpleTimeout(100);
    
    /*设置需要打开的串口名字*/
    ComSerial.setPort(ComName);

    /*设置串口通信的波特率*/
    ComSerial.setBaudrate(baudrate_IMU);
    
    /*串口设置timeout*/
    ComSerial.setTimeout(ComSerialTimeOut);

    /*打开串口*/
    try{ComSerial.open();}
    catch(serial::IOException& e){
      RCLCPP_ERROR(this->get_logger(), "Unable to open IMU port");}
      if(ComSerial.isOpen()){
        RCLCPP_INFO(this->get_logger(), "IMU Port is opened. Serial name : %s", ComName.c_str());
      }else{
        RCLCPP_ERROR(this->get_logger(), "Open IMU Port failed! Serial name : %s", ComName.c_str());}

    /*启动成功*/
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());

    using namespace std::literals::chrono_literals;
    timer_ = this->create_wall_timer(
        0.1ms, std::bind(&imuCOM::read_serial, this));
  }

 private:
  /*设置串口基本参数*/
  int baudrate_IMU,int_i,int_n,int_bridge;
  float gravity_acc;
  /*创建serial类*/
  serial::Serial ComSerial;
  /*设置计数器*/
  bool flag_50 = false;
  bool flag_51 = false;
  bool flag_52 = false;
  bool flag_53_1 = false;
  bool flag_53_2 = false;
  bool flag_53_3 = false;
  bool flag_54 = false;
  uint8_t buffer_uint8_t[17];
  uint8_t buffer_uint8_t_1[8];
  uint8_t buffer_uint8_t_2[8];
  uint8_t buffer_uint8_t_3[8];
  uint8_t buffer_uint8_t_4[8];
  std::string ComName;
  std::string imu_frame_id;
  tf2::Quaternion orientation_tf;

  /*ROS2发布者、订阅者*/
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rawData_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ImuData_publisher_;
  rclcpp::Publisher<sensor_driver_msgs::msg::ImuHwt9073can>::SharedPtr ImuHwtData_publisher_;
  std_msgs::msg::String origin_data;
  std_msgs::msg::String result_data;
  sensor_msgs::msg::Imu Imu_data;
  sensor_driver_msgs::msg::ImuHwt9073can ImuHwt_data;

  void read_serial() {
    size_t n_IMU = ComSerial.available();
    /*检查是否有IMU数据*/
    if(n_IMU!=0){
      for(int_i=0;int_i<n_IMU;int_i++){
        buffer_uint8_t[int_i] = 0;
      } 
      n_IMU = ComSerial.read(buffer_uint8_t, n_IMU);
      int_n = int(n_IMU);
      origin_data.data = "n_IMU: " ;
      for(int_i = 0; int_i < int_n; int_i++){
        int_bridge = buffer_uint8_t[int_i];
        origin_data.data = origin_data.data + std::to_string(int_bridge) + " ";
      }
      RCLCPP_INFO(this->get_logger(), origin_data.data.c_str());
      rawData_publisher_->publish(origin_data);

      if(int_n==32){
        if(buffer_uint8_t[0]==0x55){
          if(buffer_uint8_t[1]==0x50){
            /*进入时间数据处理*/
            flag_50 = true;
            ImuHwt_data.time_year = buffer_uint8_t[2];
            ImuHwt_data.time_month = buffer_uint8_t[3];
            ImuHwt_data.time_day = buffer_uint8_t[4];
            ImuHwt_data.time_hour = buffer_uint8_t[5];
            ImuHwt_data.time_min = buffer_uint8_t[6];
            ImuHwt_data.time_sec = buffer_uint8_t[7];
          }
          if(buffer_uint8_t[9]==0x51){
            /*进入加速度数据处理*/
            flag_51 = true;
            if(buffer_uint8_t[11]/0x80){
              ImuHwt_data.acc_x_raw = -(buffer_uint8_t[11]%0x80*0x100 + buffer_uint8_t[10] - 0xffff);
            }else{
              ImuHwt_data.acc_x_raw = buffer_uint8_t[11]%0x80*0x100 + buffer_uint8_t[10];
            }
            ImuHwt_data.acc_x = ImuHwt_data.acc_x_raw/32768*16*gravity_acc;
            if(buffer_uint8_t[13]/0x80){
              ImuHwt_data.acc_y_raw = -(buffer_uint8_t[13]%0x80*0x100 + buffer_uint8_t[12] - 0xffff);
            }else{
              ImuHwt_data.acc_y_raw = buffer_uint8_t[13]%0x80*0x100 + buffer_uint8_t[12];
            }
            ImuHwt_data.acc_y = ImuHwt_data.acc_y_raw/32768*16*gravity_acc;
            if(buffer_uint8_t[15]/0x80){
              ImuHwt_data.acc_z_raw = -(buffer_uint8_t[15]%0x80*0x100 + buffer_uint8_t[14] - 0xffff);
            }else{
              ImuHwt_data.acc_z_raw = buffer_uint8_t[15]%0x80*0x100 + buffer_uint8_t[14];
            }
            ImuHwt_data.acc_z = ImuHwt_data.acc_z_raw/32768*16*gravity_acc;
          }
          if(buffer_uint8_t[17]==0x52){
            /*进入角速度数据处理*/
            flag_52 = true;
            if(buffer_uint8_t[19]/0x80){
              ImuHwt_data.a_vel_x_raw = -(buffer_uint8_t[19]%0x80*0x100 + buffer_uint8_t[18] - 0xffff);
            }else{
              ImuHwt_data.a_vel_x_raw = buffer_uint8_t[19]%0x80*0x100 + buffer_uint8_t[18];
            }
            ImuHwt_data.a_vel_x = ImuHwt_data.a_vel_x_raw/32768*2000;
            if(buffer_uint8_t[21]/0x80){
              ImuHwt_data.a_vel_y_raw = -(buffer_uint8_t[21]%0x80*0x100 + buffer_uint8_t[20] - 0xffff);
            }else{
              ImuHwt_data.a_vel_y_raw = buffer_uint8_t[21]%0x80*0x100 + buffer_uint8_t[20];
            }
            ImuHwt_data.a_vel_y = ImuHwt_data.a_vel_y_raw/32768*2000;
            if(buffer_uint8_t[23]/0x80){
              ImuHwt_data.a_vel_z_raw = -(buffer_uint8_t[23]%0x80*0x100 + buffer_uint8_t[22] - 0xffff);
            }else{
              ImuHwt_data.a_vel_z_raw = buffer_uint8_t[23]%0x80*0x100 + buffer_uint8_t[13];
            }
            ImuHwt_data.a_vel_z = ImuHwt_data.a_vel_z_raw/32768*2000;
          }
          if(buffer_uint8_t[25]==0x53){
            /*进入角度数据处理*/
            if(buffer_uint8_t[26]==0x01){
              flag_53_1 = true;
              if(buffer_uint8_t[30]/0x80){
                ImuHwt_data.angle_x_raw = -(((buffer_uint8_t[31]%0x80*0x100 + buffer_uint8_t[30])*0x100 + buffer_uint8_t[29])*0x100 + buffer_uint8_t[28] - 0xffffffff);
              }else{
                ImuHwt_data.angle_x_raw = ((buffer_uint8_t[31]%0x80*0x100 + buffer_uint8_t[30])*0x100 + buffer_uint8_t[29])*0x100 + buffer_uint8_t[28];
              }
              ImuHwt_data.angle_x = ImuHwt_data.angle_x_raw / 1000;
            }
          }
          if(buffer_uint8_t[9]==0x53){
            if(buffer_uint8_t[10]==0x02){
              flag_53_2 = true;
              if(buffer_uint8_t[14]/0x80){
                ImuHwt_data.angle_y_raw = -(((buffer_uint8_t[15]%0x80*0x100 + buffer_uint8_t[14])*0x100 + buffer_uint8_t[13])*0x100 + buffer_uint8_t[12] - 0xffffffff);
              }else{
                ImuHwt_data.angle_y_raw = ((buffer_uint8_t[15]%0x80*0x100 + buffer_uint8_t[14])*0x100 + buffer_uint8_t[13])*0x100 + buffer_uint8_t[12];
              }
              ImuHwt_data.angle_y = ImuHwt_data.angle_y_raw / 1000;
            }
            if(buffer_uint8_t[18]==0x03){
              flag_53_3 = true;
              if(buffer_uint8_t[22]/0x80){
                ImuHwt_data.angle_z_raw = -(((buffer_uint8_t[23]%0x80*0x100 + buffer_uint8_t[22])*0x100 + buffer_uint8_t[21])*0x100 + buffer_uint8_t[20] - 0xffffffff);
              }else{
                ImuHwt_data.angle_z_raw = ((buffer_uint8_t[23]%0x80*0x100 + buffer_uint8_t[22])*0x100 + buffer_uint8_t[21])*0x100 + buffer_uint8_t[20];
              }
              ImuHwt_data.angle_z = ImuHwt_data.angle_z_raw / 1000;
            }
          }
          if(buffer_uint8_t[25]==0x54){
            /*进入磁场数据处理*/
            flag_54 = true;
            if(buffer_uint8_t[27]/0x80){
              ImuHwt_data.mag_x = -(buffer_uint8_t[27]%0x80*0x100 + buffer_uint8_t[26] - 0xffff);
            }else{
              ImuHwt_data.mag_x = buffer_uint8_t[27]%0x80*0x100 + buffer_uint8_t[26];
            }
            if(buffer_uint8_t[29]/0x80){
              ImuHwt_data.mag_y = -(buffer_uint8_t[29]%0x80*0x100 + buffer_uint8_t[28] - 0xffff);
            }else{
              ImuHwt_data.mag_y = buffer_uint8_t[29]%0x80*0x100 + buffer_uint8_t[28];
            }
            if(buffer_uint8_t[31]/0x80){
              ImuHwt_data.mag_z = -(buffer_uint8_t[31]%0x80*0x100 + buffer_uint8_t[30] - 0xffff);
            }else{
              ImuHwt_data.mag_z = buffer_uint8_t[31]%0x80*0x100 + buffer_uint8_t[30];
            }
          }

        result_data.data = "result: " ;
        result_data.data = result_data.data + std::to_string(flag_50) + " ";
        result_data.data = result_data.data + std::to_string(flag_51) + " ";
        result_data.data = result_data.data + std::to_string(flag_52) + " ";
        result_data.data = result_data.data + std::to_string(flag_53_1) + " ";
        result_data.data = result_data.data + std::to_string(flag_53_2) + " ";
        result_data.data = result_data.data + std::to_string(flag_53_3) + " ";
        result_data.data = result_data.data + std::to_string(flag_54) + " ";
        RCLCPP_INFO(this->get_logger(), result_data.data.c_str());


          if(flag_50&&flag_51&&flag_52&&flag_53_1&&flag_53_2&&flag_53_3&&flag_54){
            /*检测是否有新数据，并发布相关话题*/
            orientation_tf.setRPY(ImuHwt_data.angle_x,ImuHwt_data.angle_y,ImuHwt_data.angle_z);
            Imu_data.orientation.x = orientation_tf.x();
            Imu_data.orientation.y = orientation_tf.y();
            Imu_data.orientation.z = orientation_tf.z();
            Imu_data.orientation.w = orientation_tf.w();
            Imu_data.orientation_covariance = {-1,0,0,0,0,0,0,0,0};
            Imu_data.angular_velocity.x = ImuHwt_data.a_vel_x;
            Imu_data.angular_velocity.y = ImuHwt_data.a_vel_y;
            Imu_data.angular_velocity.z = ImuHwt_data.a_vel_z;
            Imu_data.angular_velocity_covariance = {-1,0,0,0,0,0,0,0,0};
            Imu_data.linear_acceleration.x = ImuHwt_data.acc_x;
            Imu_data.linear_acceleration.y = ImuHwt_data.acc_y;
            Imu_data.linear_acceleration.z = ImuHwt_data.acc_z;
            Imu_data.linear_acceleration_covariance = {-1,0,0,0,0,0,0,0,0};

            ImuHwt_data.header.stamp = this->get_clock()->now();
            ImuHwtData_publisher_->publish(ImuHwt_data);
            Imu_data.header.stamp = this->get_clock()->now();
            ImuData_publisher_->publish(Imu_data);
            flag_50 = false;
            flag_51 = false;
            flag_52 = false;
            flag_53_1 = false;
            flag_53_2 = false;
            flag_53_3 = false;
            flag_54 = false;
          }
        }else{
          RCLCPP_INFO(this->get_logger(), "Wrong data id, passed this section.");
          return;
        }
      }else{
        RCLCPP_INFO(this->get_logger(), "Wrong data length, passed this section.");
        return;
      }
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  /*创建对应节点的共享指针对象*/
  auto node = std::make_shared<imuCOM>("imuDriver_node");
  /* 运行节点，并检测退出信号*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
