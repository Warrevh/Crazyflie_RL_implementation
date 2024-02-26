#include <memory>
#include <iostream>
#include <fstream>
#include <chrono>
#include <functional>
#include <Eigen/Dense>
#include <rmw/qos_profiles.h>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "motion_capture_tracking_interfaces/msg/named_pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "crazyflie_interfaces/msg/position.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "yaml-cpp/yaml.h"


using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace Eigen;

class SwarmControl : public rclcpp::Node
{
  public:
    SwarmControl()
    : Node("swarm_control")
    {
        const char* config_path = "~/ros_ws/src/crazyswarm2/config/crazyflies.yaml";
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.best_effort();
        qos.durability_volatile();
        rmw_qos_liveliness_policy_t liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
        qos.liveliness(liveliness);
        subscription_ = this->create_subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>(
        "/poses", qos, std::bind(&SwarmControl::callback, this, _1));
        publisher_ = this->create_publisher<crazyflie_interfaces::msg::Position>("/cf2/cmd_position", qos);
        client_land = this->create_client<crazyflie_interfaces::srv::Land>("cf2/land");
        client_takeoff = this->create_client<crazyflie_interfaces::srv::Takeoff>("cf2/takeoff");

        auto request = std::make_shared<crazyflie_interfaces::srv::Takeoff::Request>();
        //request->group_mask = 0;
        request->height = 1.0;
        request->duration = rclcpp::Duration::from_seconds(2.0);
        client_takeoff->async_send_request(request);
        rclcpp::sleep_for(2500ms);
    }

  private:
    rclcpp::Subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>::SharedPtr subscription_;
    rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr publisher_;
    rclcpp::Client<crazyflie_interfaces::srv::Land>::SharedPtr client_land;
    rclcpp::Client<crazyflie_interfaces::srv::Takeoff>::SharedPtr client_takeoff;

    void read_config(const char* config_path)
    {
        YAML::Node config = YAML::LoadFile(config_path);
    }
    
    void callback(const motion_capture_tracking_interfaces::msg::NamedPoseArray & msg) const
    {
        //float px, py, pz, cf2_x, cf2_y, cf2_z;
        Vector3f p_v; 
        Vector3f cf2_v;
        auto message = crazyflie_interfaces::msg::Position();
        for (unsigned int i=0; i<msg.poses.size(); i++){
            if (msg.poses[i].name == "platform"){
            p_v[0] = msg.poses[i].pose.position.x;
        p_v[1] = msg.poses[i].pose.position.y;
        p_v[2] = msg.poses[i].pose.position.z + 1.0; 
        }
        else if (msg.poses[i].name == "cf2"){
        cf2_v[0] = msg.poses[i].pose.position.x;
        cf2_v[1] = msg.poses[i].pose.position.y;
        cf2_v[2] = msg.poses[i].pose.position.z;
        }
        }
        Vector3f d = p_v-cf2_v;
        float dist = d.norm();
        if (dist < 0.7){
            message.x = p_v[0]; 
            message.y = p_v[1];
            message.z = p_v[2];
        }
        else{
            Vector3f targ = cf2_v + (d/dist)*0.7;
            message.x = targ[0];
            message.y = targ[1];
            message.z = targ[2];
        }
        if (d[2] < -0.3){
            message.z = cf2_v[2]-0.3;
        }
        if (d[2] > 0.3){
            message.z = cf2_v[2]+0.3;
        }
        publisher_->publish(message);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwarmControl>());
    rclcpp::shutdown();
    return 0;
}

