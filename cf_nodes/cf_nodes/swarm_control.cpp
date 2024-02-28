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


class SwarmControl : public rclcpp::Node
{
  public:
    std::vector<int> ids;
    std::vector<std::string> names;
    std::vector<CF> cfs;

    SwarmControl()
    : Node("swarm_control")
    {
        const char* config_path = "/root/ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml";
        read_config(config_path);
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.best_effort();
        qos.durability_volatile();
        rmw_qos_liveliness_policy_t liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
        qos.liveliness(liveliness);
        subscription_ = this->create_subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>(
        "/poses", qos, std::bind(&SwarmControl::qtm_callback, this, _1));
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
            cf->x_r = 0;
            cf->y_r = 0;
            cf->z_r = 0;
            cf->yaw = 0;
            topic = "/cf/cmd_position";
            for (size_t j = 0; j < names[i].size()-2; j++) //Run the loop once if cfx, twice if cfxx and so on
            {
                topic.insert(3+j,1,names[i][2+j]);
            }          
            // topic = "/cfx/cmd_position"; //OBS OBS OBS only handles ids up to 9
            // topic[3] = names[i][2];
            cf->pub = this->create_publisher<crazyflie_interfaces::msg::Position>(topic,qos); 
            cfs.push_back(*cf);
        }
        timer_ = this->create_wall_timer(
        10ms, std::bind(&SwarmControl::swarm_callback, this));
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
                    //ids.push_back(std::stoi(robot_name[2]));// OBS OBS OBS does not handle ids over 9
                    names.push_back(robot_name);
                }
            }
        }
    }

    void swarm_callback()
    {
        if(true)//New controller pose
        {
            return;//Calculate new cf poses
        }
        //Publish cf poses
    }

    std::vector<int> qtm_callback(const motion_capture_tracking_interfaces::msg::NamedPoseArray & msg) const
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

