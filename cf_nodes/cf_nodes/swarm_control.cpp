#include <memory>
#include <iostream>
#include <fstream>
#include <chrono>
#include <functional>
#include <Eigen/Dense>
#include <rmw/qos_profiles.h>
#include <fstream>
#include <vector>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>

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

struct CF
{
    float x;
    float y;
    float z;
    float yaw;
    float x_r;
    float y_r;
    float z_r;
    rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr pub;
};

struct Hand
{
    float x;
    float y;
    float z;
    tf2::Quaternion q;
};


class SwarmControl : public rclcpp::Node
{
  public:
    std::vector<std::string> names;
    std::vector<CF*> cfs;
    std::unordered_map<std::string,CF*> cfs_map;
    Hand hand;

    SwarmControl()
    : Node("swarm_control")
    {
        hand.x = 0.5;
        hand.y = 0.5;
        hand.z = 0.2;
        hand.q = tf2::Quaternion(0, 0, 0.258819, 0.9659258);
        float offset_x[4] = {1,0,-1,0};
        float offset_y[4] = {0,-1,0,1};
        const char* config_path = "/root/ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml";
        read_config(config_path);
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.best_effort();
        qos.durability_volatile();
        rmw_qos_liveliness_policy_t liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
        qos.liveliness(liveliness);
        std::cout << names.size() << "active crazyflies: " << std::endl;
        CF* cf;
        std::string topic;
        for (size_t i = 0; i < names.size(); i++)
        {   
            std::cout << names[i] << std::endl;
            cf = new CF();
            cf->x = 0;
            cf->y = 0;
            cf->z = 0;
            cf->x_r = offset_x[i];
            cf->y_r = offset_y[i];
            cf->z_r = 1;
            cf->yaw = 0;
            topic = "/cf/cmd_position";
            for (size_t j = 0; j < names[i].size()-2; j++) //Run the loop once if cfx, twice if cfxx and so on
            {
                topic.insert(3+j,1,names[i][2+j]);
            }          
            cf->pub = this->create_publisher<crazyflie_interfaces::msg::Position>(topic,qos); 
            cfs.push_back(cf);
            cfs_map[names[i]] = cf;
        }
        timer_ = this->create_wall_timer(
        10ms, std::bind(&SwarmControl::swarm_callback, this));
        subscription_ = this->create_subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>(
        "/poses", qos, std::bind(&SwarmControl::qtm_callback, this, _1));
        // auto request = std::make_shared<crazyflie_interfaces::srv::Takeoff::Request>();
        // //request->group_mask = 0;
        // request->height = 1.0;
        // request->duration = rclcpp::Duration::from_seconds(2.0);
        // client_takeoff->async_send_request(request);
        // rclcpp::sleep_for(2500ms);
    }

  private:
    rclcpp::Subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>::SharedPtr subscription_;
    rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr publisher_;
    rclcpp::Client<crazyflie_interfaces::srv::Takeoff>::SharedPtr client_takeoff;
    rclcpp::TimerBase::SharedPtr timer_;

    void read_config(const char* config_path)
    {
        // YAML::Node config = YAML::LoadFile(config_path);
        YAML::Node config = YAML::LoadFile(config_path);
        YAML::Node robots;
        YAML::Node attributes;
        robots = config["robots"];
        for(YAML::const_iterator it=robots.begin(); it!=robots.end(); ++it)
        {
            const std::string& robot_name=it->first.as<std::string>();
            if (robot_name[0]=='c' && robot_name[1]=='f') //Checks if the robot is a crazyflie
            {
                attributes = it->second;
                if(attributes["enabled"].as<bool>())
                {
                    names.push_back(robot_name);
                }
            }
        }
    }

    void swarm_callback()
    {
        CF cf;
        tf2::Quaternion cfq;
        float x;
        float y;
        float z;
        auto message = crazyflie_interfaces::msg::Position();
        tf2::Quaternion handq_prime = tf2::Quaternion(hand.q);
        handq_prime[0] = handq_prime[0]*-1; handq_prime[1] = handq_prime[1]*-1; handq_prime[2] = handq_prime[2]*-1;
        for (size_t i = 0; i < cfs.size(); i++)
        {
            cf = *(cfs[i]);
            cfq = tf2::Quaternion(cf.x_r,cf.y_r,cf.z_r,0);
            cfq = hand.q*cfq*handq_prime;
            // message.x = hand.x+cf.x_r;
            // message.y = hand.y+cf.y_r;
            // message.z = hand.z+cf.z_r;
            message.x = hand.x+cfq[0];
            message.y = hand.y+cfq[1];
            message.z = hand.z+cfq[2];
            // message.yaw = hand.yaw;
            cf.pub->publish(message);
        }
    }

    void qtm_callback(const motion_capture_tracking_interfaces::msg::NamedPoseArray & msg)
    {
        Vector3f p_v; 
        Vector3f cf2_v;
        auto message = crazyflie_interfaces::msg::Position();
        for (unsigned int i=0; i<msg.poses.size(); i++){
            if (msg.poses[i].name == "hand"){
                hand.x = msg.poses[i].pose.position.x;
                hand.y = msg.poses[i].pose.position.y;
                hand.z = msg.poses[i].pose.position.z;
                hand.q = tf2::Quaternion(msg.poses[i].pose.orientation.x,msg.poses[i].pose.orientation.y,msg.poses[i].pose.orientation.z,msg.poses[i].pose.orientation.w);
            }
            // else if (msg.poses[i].name[0]=='c' && msg.poses[i].name[1]=='f'){
            //     cfs_map[msg.poses[i].name]->x = msg.poses[i].pose.position.x;
            //     cfs_map[msg.poses[i].name]->y = msg.poses[i].pose.position.y;
            //     cfs_map[msg.poses[i].name]->z = msg.poses[i].pose.position.z;
            // }
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwarmControl>());
    rclcpp::shutdown();
    return 0;
}

