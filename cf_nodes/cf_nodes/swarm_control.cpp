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

// QUATERNION CONVENTION USED: (x,y,z,w)

class CF
{
public:
    // Current pose of cf (currently not used)
    float x;
    float y;
    float z;
    float yaw;
    //  Cf pose relative to swarm origin
    float x_r;
    float y_r;
    float z_r;
    rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr pub;
    rclcpp::Client<crazyflie_interfaces::srv::Takeoff>::SharedPtr takeoff_cli;
    // Function to change cf's pose in the formation
    void set_ref(float x, float y, float z)
    {
        //TODO: put min/max values for X_r
        float rate = 0.01; // Scale factor for the speed of the chage of the formation
        x_r += x_r*x*rate; // Ex: hand_r is (0.1,0,0) relative to hand_stat and x_r is 0.5->
        y_r += y_r*y*rate; // x_r will increase with x_r*5 mm each time step
        z_r += z_r*z*rate;
    }
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
    Hand hand_r;
    Hand hand_l;
    Hand hand_stat; // Used to store the most recent pose of the right hand when in mode 1
    // Mode is changed with the angle of the left hand
    int mode; // 0: "Hold" the swarm does not react to the right hand
              // 1: "Follow" the pose of the swarm changes with the pose of the right hand
              // 2: "Formation" the formation of the swarm changes with the movements of the right hand
    // Swarm origin relative to the right hand
    float swarm_x;
    float swarm_y;
    float swarm_z;

    SwarmControl()
    : Node("swarm_control")
    {
        mode = 0;
        hand_r.x = 0.0;
        hand_r.y = 0.0;
        hand_r.z = 0.0;
        hand_r.q = tf2::Quaternion(0, 0, 0.258819, 0.9659258);
        hand_stat.x = 0.0;
        hand_stat.y = 0.0;
        hand_stat.z = 0.0;
        hand_stat.q = tf2::Quaternion(0, 0, 0, 1);
        swarm_x = 1.0;
        swarm_y = 0.0;
        swarm_z = 0.5;
        float offset_x[6] = {-0.5,-0.5,-0.5,0,0,0.5};
        float offset_y[6] = {0,-1,1,-0.5,0.5,0};
        float offset_z[6] = {0,0,0,0,0,0};
        const char* config_path = "/root/ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml";
        read_config(config_path);
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.best_effort();
        qos.durability_volatile();
        rmw_qos_liveliness_policy_t liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
        qos.liveliness(liveliness);
        std::cout << names.size() << " active crazyflies: " << std::endl;
        CF* cf;
        std::string cmd_topic;
        std::string service;
        auto request = std::make_shared<crazyflie_interfaces::srv::Takeoff::Request>();
        request->height = 0.5;
        request->duration = rclcpp::Duration::from_seconds(2.0);
        for (size_t i = 0; i < names.size(); i++)
        {   
            std::cout << names[i] << std::endl;
            cf = new CF();
            cf->x = 0;
            cf->y = 0;
            cf->z = 0;
            cf->x_r = offset_x[i];
            cf->y_r = offset_y[i];
            cf->z_r = offset_z[i];
            cf->yaw = 0;
            cmd_topic = "/cf/cmd_position";
            service = "/cf/takeoff";
            for (size_t j = 0; j < names[i].size()-2; j++) //Run the loop once if cfx, twice if cfxx and so on
            {
                cmd_topic.insert(3+j,1,names[i][2+j]);
                service.insert(3+j,1,names[i][2+j]);
            }          
            cf->pub = this->create_publisher<crazyflie_interfaces::msg::Position>(cmd_topic,qos); 
            cf->takeoff_cli = this->create_client<crazyflie_interfaces::srv::Takeoff>(service);
            cfs.push_back(cf);
            cfs_map[names[i]] = cf;
            cf->takeoff_cli->async_send_request(request);
        }
        rclcpp::sleep_for(2500ms);
        timer_ = this->create_wall_timer(
        10ms, std::bind(&SwarmControl::swarm_callback, this));
        timer_slow_ = this->create_wall_timer(
        100ms, std::bind(&SwarmControl::mode_callback, this));
        subscription_ = this->create_subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>(
        "/poses", qos, std::bind(&SwarmControl::qtm_callback, this, _1));
    }

  private:
    rclcpp::Subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>::SharedPtr subscription_;
    rclcpp::Client<crazyflie_interfaces::srv::Takeoff>::SharedPtr client_takeoff;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_slow_;
    bool choosing;

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
        switch (mode)// 0: hold, 1: pose, 2: formation
        {
        case 0:
            {
            tf2::Quaternion handq_prime = tf2::Quaternion(hand_stat.q);
            handq_prime[0] = -handq_prime[0]; handq_prime[1] = -handq_prime[1]; handq_prime[2] = -handq_prime[2];
            for (size_t i = 0; i < cfs.size(); i++)
            {
                cf = *(cfs[i]);
                //Transforming cf from swarm frame, to hand frame, to world frame
                cfq = tf2::Quaternion(cf.x_r+swarm_x,cf.y_r+swarm_y,cf.z_r+swarm_z,0);//Rotation of cf in world frame
                cfq = hand_stat.q*cfq*handq_prime;//Rotation
                message.x = hand_stat.x+cfq[0];//Translation
                message.y = hand_stat.y+cfq[1];
                message.z = hand_stat.z+cfq[2];
                // message.yaw = hand.yaw;
                cf.pub->publish(message);
            }
            break;
            }
        case 1:
            {
            tf2::Quaternion handq_prime = tf2::Quaternion(hand_r.q);
            handq_prime[0] = -handq_prime[0]; handq_prime[1] = -handq_prime[1]; handq_prime[2] = -handq_prime[2];
            for (size_t i = 0; i < cfs.size(); i++)
            {
                cf = *(cfs[i]);
                //Transforming cf from swarm frame, to hand frame, to world frame
                cfq = tf2::Quaternion(cf.x_r+swarm_x,cf.y_r+swarm_y,cf.z_r+swarm_z,0);//Rotation of cf in world frame
                cfq = hand_r.q*cfq*handq_prime;//Rotation
                message.x = hand_r.x+cfq[0];//Translation
                message.y = hand_r.y+cfq[1];
                message.z = hand_r.z+cfq[2];
                // message.yaw = hand.yaw;
                cf.pub->publish(message);
            }
            break;
            }
        case 2:
            {
            // TODO: add code for changing formation
            tf2::Quaternion handq_prime = tf2::Quaternion(hand_stat.q);
            handq_prime[0] = handq_prime[0]*-1; handq_prime[1] = handq_prime[1]*-1; handq_prime[2] = handq_prime[2]*-1;
            for (size_t i = 0; i < cfs.size(); i++)
            {
                cf = *(cfs[i]);
                //Transforming cf from swarm frame, to hand frame, to world frame
                cfq = tf2::Quaternion(cf.x_r+swarm_x,cf.y_r+swarm_y,cf.z_r+swarm_z,0);//Rotation of cf in world frame
                cfq = hand_stat.q*cfq*handq_prime;//Rotation
                message.x = hand_stat.x+cfq[0];//Translation
                message.y = hand_stat.y+cfq[1];
                message.z = hand_stat.z+cfq[2];
                // message.yaw = hand.yaw;
                cf.pub->publish(message);
            }
            break;
            }
        case 3:
            break;
        default:
            break;
        }
    }

    void mode_callback()
    {
        float y; float x; // z-components of the y and x unit vectors of the local coordinate system
        x = 2*(hand_l.q[0]*hand_l.q[2] - hand_l.q[3]*hand_l.q[1]);
        y = 2*(hand_l.q[1]*hand_l.q[2] + hand_l.q[3]*hand_l.q[0]);
        if(x > 0.7)
        {
            mode = 0;
        }
        else if(y < -0.7)
        {
            mode = 2;
        }
        else
        {
            if (mode != 1)
            {
                swarm_x += (hand_stat.x-hand_r.x);
                swarm_y += (hand_stat.y-hand_r.y);
                swarm_z += (hand_stat.z-hand_r.z);
            }
            hand_stat.x = hand_r.x;
            hand_stat.y = hand_r.y;
            hand_stat.z = hand_r.z;
            hand_stat.q = tf2::Quaternion(hand_r.q[0], hand_r.q[1], hand_r.q[2], hand_r.q[3]);
            mode = 1;
        }
        std::cout << "Mode: " << mode << std::endl;
        std::cout << "Hand right: " << hand_r.x << "," << hand_r.y << "," << hand_r.z << std::endl;
        std::cout << "Hand static: " << hand_stat.x << "," << hand_stat.y << "," << hand_stat.z << std::endl;
        std::cout << "Swarm: " << swarm_x << "," << swarm_y << "," << swarm_z << std::endl;
    }

    // void mode_callback()
    // {
    //     if (choosing)
    //     {
    //         //Check "direction" of left hand
    //         if(hand_r.x-hand_l.x > 0.6)
    //         {
    //             mode = 1;
    //             hand_stat.x = hand_r.x;
    //             hand_stat.y = hand_r.y;
    //             hand_stat.z = hand_r.z;
    //             hand_stat.q = hand_r.q;
    //             choosing = false;
    //         }
    //         else if(hand_r.y-hand_l.y > 0.6)
    //         {
    //             mode = 2;
    //             hand_stat.x = hand_r.x;
    //             hand_stat.y = hand_r.y;
    //             hand_stat.z = hand_r.z;
    //             hand_stat.q = hand_r.q;
    //             choosing = false;
    //         }
    //         else if(hand_r.z-hand_l.z > 0.6)
    //         {
    //             mode = 3;
    //             hand_stat.x = hand_r.x;
    //             hand_stat.y = hand_r.y;
    //             hand_stat.z = hand_r.z;
    //             hand_stat.q = hand_r.q;
    //             choosing = false;
    //         }
    //         else if(hand_r.z-hand_l.z < -0.6)
    //         {
    //             mode = 0;
    //             choosing = false;
    //         }
    //     }
    //     else
    //     {
    //         //Check distance between hands
    //         Vector3f r; r[0]=hand_r.x; r[1]=hand_r.y; r[2]=hand_r.z;
    //         Vector3f l; l[0]=hand_l.x; l[1]=hand_l.y; l[2]=hand_l.z;
    //         Vector3f d = r-l;
    //         float dist = d.norm();
    //         if(dist<0.1)
    //         {
    //             choosing = true;
    //         }
    //     }
    // }

    void qtm_callback(const motion_capture_tracking_interfaces::msg::NamedPoseArray & msg)
    {
        Vector3f p_v; 
        Vector3f cf2_v;
        auto message = crazyflie_interfaces::msg::Position();
        for (unsigned int i=0; i<msg.poses.size(); i++){
            if (msg.poses[i].name == "hand_r"){
                hand_r.x = msg.poses[i].pose.position.x;
                hand_r.y = msg.poses[i].pose.position.y;
                hand_r.z = msg.poses[i].pose.position.z;
                hand_r.q = tf2::Quaternion(msg.poses[i].pose.orientation.x,msg.poses[i].pose.orientation.y,msg.poses[i].pose.orientation.z,msg.poses[i].pose.orientation.w);
            }
            else if (msg.poses[i].name == "hand_l"){
                hand_l.x = msg.poses[i].pose.position.x;
                hand_l.y = msg.poses[i].pose.position.y;
                hand_l.z = msg.poses[i].pose.position.z;
                hand_l.q = tf2::Quaternion(msg.poses[i].pose.orientation.x,msg.poses[i].pose.orientation.y,msg.poses[i].pose.orientation.z,msg.poses[i].pose.orientation.w);
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

