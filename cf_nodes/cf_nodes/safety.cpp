#include <memory>
#include <iostream>
#include <fstream>
#include <chrono>
#include <functional>
#include <Eigen/Dense>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "motion_capture_tracking_interfaces/msg/named_pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "crazyflie_interfaces/msg/position.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/notify_setpoints_stop.hpp"
// #include "crazyflie_interfaces/srv/Stop.hpp"
//#include "crazyflie_interfaces/srv/goto.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace Eigen;

struct Limit
{
  public:
    float x_min,x_max,y_min,y_max,z_min,z_max;
    Limit(float xmi,float xma,float ymi,float yma,float zmi,float zma):x_min(xmi),x_max(xma),y_min(ymi),y_max(yma),z_min(zmi),z_max(zma){};
};

Limit space(-3,3,-3,3,-0.5,8);
int count;
double PRED_HOR = 0.5;

class Safety : public rclcpp::Node
{
  public:
    Safety()
    : Node("safety")
    {
      land_started_ = false;
      cf2_pose_.x = 1.0;
      cf2_pose_.y = -1.0;
      cf2_pose_.z = 0.0;
      cf2_pose_.yaw = M_PI*3/4;
      rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1));
      qos.best_effort();
      qos.durability_volatile();
      //qos.lifespan(rclcpp::Duration(1000000,0));
      //qos.deadline(rclcpp::Duration(1000000,0));
      rmw_qos_liveliness_policy_t liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
      qos.liveliness(liveliness);
      //qos.liveliness_lease_duration(rclcpp::Duration(1000000,0));

      sub_mocap = this->create_subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>(
      "/poses", qos, std::bind(&Safety::cb_mocap, this, _1));
      sub_cmd_pos = this->create_subscription<crazyflie_interfaces::msg::Position>(
      "/cf2/cmd_position", qos, std::bind(&Safety::cb_cmd_pos, this, _1));
      sub_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cf2/cmd_vel_safe", qos, std::bind(&Safety::cb_cmd_vel, this, _1));

      pub_cmd_pos = this->create_publisher<crazyflie_interfaces::msg::Position>("/cf2/cmd_position", qos);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&Safety::timer_callback, this));
      pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cf2/cmd_vel_legacy", 10);

      client_land_ = this->create_client<crazyflie_interfaces::srv::Land>("cf2/land");
      client_notify_setpoints_stop_ = this->create_client<crazyflie_interfaces::srv::NotifySetpointsStop>("notify_setpoints_stop");
      // client_stop = this->create_client<crazyflie_interfaces::srv::Stop>("cf2/emergency");
      //client_GoTo = this->create_client<crazyflie_interfaces::srv::GoTo>("cf2/goto");
    }

  private:
    void cmd_land(){
      std::cout << "here" << std::endl;
      land_started_ = true;
      rclcpp::Rate rate(20);
      auto message = crazyflie_interfaces::msg::Position();
      message.x = cf2_pose_.x;
      message.y = cf2_pose_.y;
      message.z = cf2_pose_.z;
      while(message.z > 0.03){
        std::cout << "in loop" << std::endl;
        if (message.z - cf2_pose_.z < 0.1){
          message.z = message.z -0.02;
        }
        pub_cmd_pos->publish(message);
        rate.sleep();
      }
      for (size_t i = 0; i < 10; i++){
        pub_cmd_pos->publish(message);
        rate.sleep();
      }
      
      auto message_vel = geometry_msgs::msg::Twist();
      message_vel.linear.y = 0.0;
      message_vel.linear.x = 0.0;
      message_vel.linear.z = 0.0;
      message_vel.angular.z = 0.0;
      pub_cmd_vel_->publish(message_vel);
      // auto request1 = std::make_shared<crazyflie_interfaces::srv::NotifySetpointsStop::Request>();
      // request1->remain_valid_millisecs = 1;
      // request1->group_mask = 0;
      // client_notify_setpoints_stop_->async_send_request(request1);

      // auto request2 = std::make_shared<crazyflie_interfaces::srv::Land::Request>();
      // //request2->group_mask = 0;
      // request2->height = 0.04;
      // request2->duration = rclcpp::Duration::from_seconds(3);
      // rclcpp::sleep_for(2ms);
      // client_land_->async_send_request(request2);
      // rclcpp::sleep_for(3500ms);

      // auto request = std::make_shared<crazyflie_interfaces::srv::Stop::Request>();
      // //request->group_mask = 0;
      // client_stop->async_send_request(request);
      // rclcpp::sleep_for(2000ms);
    }
    void timer_callback(){	// If a cmd has not been sent to the cf for 0.5 seconds, the land client will call the land service. The subscriber
      count+=1;			// to cmd_position will reset the timer each time its callback is run.
      //if(count<30){
	//auto request std::make_shared<crazyflie_interfaces::srv::GoTo>;
	//request->Point = geometry_msgs::msg::Point
	//}
      //Call service for landing
      // std::cout << "Calling land service..." << std::endl;
      // auto request = std::make_shared<crazyflie_interfaces::srv::Land::Request>();
      // //request->group_mask = 0;
      // request->height = -0.03;
      // rclcpp::Duration dur = rclcpp::Duration::from_seconds(3.0);
      // request->duration = dur;
      // client_land->async_send_request(request);
      // rclcpp::sleep_for(10000ms);
      std::cout << "timer callback" << std::endl;
      if (land_started_){
        return;
      }
      std::cout << "calling cmd_land" << std::endl;
      cmd_land();

    }
    void cb_mocap(const motion_capture_tracking_interfaces::msg::NamedPoseArray & msg)
    {
      //float px, py, pz, cf2_x, cf2_y, cf2_z;
      Vector3f p_v;
      Vector3f cf2_v;
      auto message = crazyflie_interfaces::msg::Position();
      for (unsigned int i=0; i<msg.poses.size(); i++){
        // if (msg.poses[i].name == "platform"){
        //   p_v[0] = msg.poses[i].pose.position.x;
        //   p_v[1] = msg.poses[i].pose.position.y;
        //   p_v[2] = msg.poses[i].pose.position.z + 1.0;
        // }
        if (msg.poses[i].name == "cf2"){
          //cf2_v[0] = msg.poses[i].pose.position.x;
          //cf2_v[1] = msg.poses[i].pose.position.y;
          //cf2_v[2] = msg.poses[i].pose.position.z;
          cf2_pose_.x = msg.poses[i].pose.position.x;
          cf2_pose_.y = msg.poses[i].pose.position.y;
          cf2_pose_.z = msg.poses[i].pose.position.z;
        }
      }
    }
    void cb_cmd_pos(const crazyflie_interfaces::msg::Position & msg) const{ // Checks if a prohibited cmd command is published. If so, the callback will
                  						    // publish its own cmd, which will be the projection on the border to the
      count = 0;							    // prohibited zone.
      auto message = crazyflie_interfaces::msg::Position();
      message.x = msg.x;
      message.y = msg.y;
      message.z = msg.z;
      message.yaw = msg.yaw;
      if (this->hinder(message,msg)){
        pub_cmd_pos->publish(message);
      }
      timer_->reset();
    }
    void cb_cmd_vel(const geometry_msgs::msg::Twist& msg) const{
      
      // auto message = crazyflie_interfaces::msg::Position();
      // message.x = cf2_pose_.x;
      // message.y = cf2_pose_.y;
      // message.z = cf2_pose_.z;
      // message.yaw = cf2_pose_.yaw;

      //make prediction and run hinder()
      // auto pred = crazyflie_interfaces::msg::Position(message); // Right now the prediction is steady state
      // if (this->hinder(message,pred)){
      //   pub_cmd_pos->publish(message);
      // }
      // else{
      //   pub_cmd_vel_->publish(msg);
      // }
      auto message = geometry_msgs::msg::Twist();
      double r = space.x_max;
      double u_p = msg.linear.x/360*2*M_PI;
      double u_r = msg.linear.y/360*2*M_PI;
      double u_y = cf2_pose_.yaw;
      double p_x = cf2_pose_.x;
      double p_y = cf2_pose_.y;
      double w = cos(u_r/2)*cos(u_p/2)*cos(u_y/2) + sin(u_r/2)*sin(u_p/2)*sin(u_y/2);
      double x = sin(u_r/2)*cos(u_p/2)*cos(u_y/2) - cos(u_r/2)*sin(u_p/2)*sin(u_y/2);
      double y = cos(u_r/2)*sin(u_p/2)*cos(u_y/2) + sin(u_r/2)*cos(u_p/2)*sin(u_y/2);
      double z = cos(u_r/2)*cos(u_p/2)*sin(u_y/2) - sin(u_r/2)*sin(u_p/2)*cos(u_y/2);
      Eigen::Quaternion q_u = Eigen::Quaternion(w,x,y,z);
      Eigen::Matrix3d rot_mat = q_u.toRotationMatrix();
      Eigen::Vector3d normal = rot_mat*Eigen::Vector3d(0,0,1);
      double x_ang = atan(normal[0]/normal[2]);
      double y_ang = atan(normal[1]/normal[2]);
      double x_rot = 0;
      double y_rot = 0;
      if((x_ang>0 && p_x>0) || (x_ang<0 && p_x<0))
          x_rot = -x_ang*abs(1+(p_x-r)/r);
      if((y_ang>0 && p_y>0) || (y_ang<0 && p_y<0))
          y_rot = y_ang*abs(1+(p_y-r)/r);
      Eigen::Quaternion q_rot = Eigen::Quaternion(cos(y_rot/2),sin(y_rot/2),0.0,0.0)*Eigen::Quaternion(cos(x_rot/2),0.0,sin(x_rot/2),0.0);
      Eigen::Quaternion q_u_hat = q_rot*q_u;
      double u_r_hat = atan2(2*(q_u_hat.w()*q_u_hat.x()+q_u_hat.y()*q_u_hat.z()),1-2*(pow(q_u_hat.x(),2)+pow(q_u_hat.y(),2)));
      double u_p_hat = -M_PI/2+2*atan2(sqrt(1+2*(q_u_hat.w()*q_u_hat.y()-q_u_hat.x()*q_u_hat.z())),sqrt(1-2*(q_u_hat.w()*q_u_hat.y()-q_u_hat.x()*q_u_hat.z())));
      message.linear.y = u_r_hat/(2*M_PI)*360;
      message.linear.x = u_p_hat/(2*M_PI)*360;
      message.linear.z = msg.linear.z;
      message.angular.z = msg.angular.z;
      pub_cmd_vel_->publish(message);
      timer_->reset();
    }
    bool hinder(crazyflie_interfaces::msg::Position& message, const crazyflie_interfaces::msg::Position & msg) const {
      bool h = false;
      if (msg.x < space.x_min){
        h = true;
	      message.x = space.x_min;
      }
      else if (msg.x > space.x_max){
        h = true;
        message.x = space.x_max;
      }
      if (msg.y < space.y_min){
        h = true;
      	message.y = space.y_min;
      }
      else if (msg.y > space.y_max){
        h = true;
        message.y = space.y_max;
      }
      if (msg.z < space.z_min){
        h = true;
        message.z = space.z_min;
      }
      else if (msg.z > space.z_max){
        h = true;
        message.z = space.z_max;
      }
      return h;
    }
    bool land_started_;
    crazyflie_interfaces::msg::Position cf2_pose_;
    rclcpp::Subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>::SharedPtr sub_mocap;
    rclcpp::Subscription<crazyflie_interfaces::msg::Position>::SharedPtr sub_cmd_pos;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
    rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr pub_cmd_pos;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<crazyflie_interfaces::srv::Land>::SharedPtr client_land_;
    rclcpp::Client<crazyflie_interfaces::srv::NotifySetpointsStop>::SharedPtr client_notify_setpoints_stop_;
    // rclcpp::Client<crazyflie_interfaces::srv::Stop>::SharedPtr client_stop;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Safety>());
  rclcpp::shutdown();
  return 0;
}

