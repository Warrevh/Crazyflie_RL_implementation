#include <memory>
#include <iostream>
#include <fstream>
#include <chrono>
#include <functional>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "motion_capture_tracking_interfaces/msg/named_pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "crazyflie_interfaces/msg/position.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
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

class Safety : public rclcpp::Node
{
  public:
    Safety()
    : Node("safety")
    {
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
      "/cf1/cmd_position", qos, std::bind(&Safety::cb_cmd_pos, this, _1));
      pub_cmd_pos = this->create_publisher<crazyflie_interfaces::msg::Position>("/cf1/cmd_position", qos);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&Safety::timer_callback, this));
      client_land = this->create_client<crazyflie_interfaces::srv::Land>("cf1/land");
      //client_GoTo = this->create_client<crazyflie_interfaces::srv::GoTo>("cf1/goto");
    }

  private:
    void timer_callback(){	// If a cmd has not been sent to the cf for 0.5 seconds, the land client will call the land service. The subscriber
      count+=1;			// to cmd_position will reset the timer each time its callback is run.
      //if(count<30){
	//auto request std::make_shared<crazyflie_interfaces::srv::GoTo>;
	//request->Point = geometry_msgs::msg::Point
	//}
      //Call service for landing
      auto request = std::make_shared<crazyflie_interfaces::srv::Land::Request>();
      //request->group_mask = 0;
      request->height = 0.0;
      request->duration = rclcpp::Duration::from_seconds(4.0);
      client_land->async_send_request(request);
    }
    void cb_mocap(const motion_capture_tracking_interfaces::msg::NamedPoseArray & msg) const
    {								//I have no idea what i intended with this method...
      //float px, py, pz, cf1_x, cf1_y, cf1_z;
      Vector3f p_v;
      Vector3f cf1_v;
      auto message = crazyflie_interfaces::msg::Position();
      for (unsigned int i=0; i<msg.poses.size(); i++){
        if (msg.poses[i].name == "platform"){
          p_v[0] = msg.poses[i].pose.position.x;
          p_v[1] = msg.poses[i].pose.position.y;
          p_v[2] = msg.poses[i].pose.position.z + 1.0;
        }
        else if (msg.poses[i].name == "cf1"){
          cf1_v[0] = msg.poses[i].pose.position.x;
          cf1_v[1] = msg.poses[i].pose.position.y;
          cf1_v[2] = msg.poses[i].pose.position.z;
        }
      }
    }
    void cb_cmd_pos(const crazyflie_interfaces::msg::Position & msg) const{ // Checks if a prohibited cmd command is published. If so, the callback will
      bool hinder = false;						    // publish its own cmd, which will be the projection on the border to the
      count = 0;							    // prohibited zone.
      auto message = crazyflie_interfaces::msg::Position();
      message.x = msg.x;
      message.y = msg.y;
      message.z = msg.z;
      message.yaw = msg.yaw;
      if (msg.x < space.x_min){
        hinder = true;
	message.x = space.x_min;
      }
      else if (msg.x > space.x_max){
	hinder = true;
	message.x = space.x_max;
      }
      if (msg.y < space.y_min){
        hinder = true;
	message.y = space.y_min;
      }
      else if (msg.y > space.y_max){
	hinder = true;
	message.y = space.y_max;
      }
      if (msg.z < space.z_min){
	hinder = true;
	message.z = space.z_min;
      }
      else if (msg.z > space.z_max){
	hinder = true;
	message.z = space.z_max;
      }

      if (hinder){
        pub_cmd_pos->publish(message);
      }
      timer_->reset();
    }
    rclcpp::Subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>::SharedPtr sub_mocap;
    rclcpp::Subscription<crazyflie_interfaces::msg::Position>::SharedPtr sub_cmd_pos;
    rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr pub_cmd_pos;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<crazyflie_interfaces::srv::Land>::SharedPtr client_land;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Safety>());
  rclcpp::shutdown();
  return 0;
}

