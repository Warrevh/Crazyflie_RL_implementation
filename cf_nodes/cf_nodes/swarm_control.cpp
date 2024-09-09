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
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "motion_capture_tracking_interfaces/msg/named_pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "crazyflie_interfaces/msg/position.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "crazyflie_interfaces/srv/go_to.hpp"
#include "yaml-cpp/yaml.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/transform_datatypes.h"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace Eigen;

// QUATERNION CONVENTION USED: (x,y,z,w)

class CF
{
public:
    // Current pose of cf (currently not used)
    float x, y, z;
    float yaw;
    std::string name;
    //  Cf pose relative to swarm origin
    float x_r, y_r, z_r;
    // Factor that decides how the relative pose chages if the swarm is expanded or minimized.
    float x_r_base, y_r_base, z_r_base;

    rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr pub;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;
private:
    std::queue<crazyflie_interfaces::msg::Position> cmd_buffer;
public:
    void updateCmdBuffer(float x, float y, float z, float yaw)
    {
        if (!cmd_buffer.empty() && cmd_buffer.size() >= 5)
        {
            cmd_buffer.pop();
        }
        crazyflie_interfaces::msg::Position msg;
        msg.x = x; msg.y = y; msg.z = z; msg.yaw = yaw;
        cmd_buffer.push(msg);
    }
    std::vector<crazyflie_interfaces::msg::Position> getCmdBuffer()
    {
        std::queue<crazyflie_interfaces::msg::Position> temp_buffer(cmd_buffer);
        std::vector<crazyflie_interfaces::msg::Position> cmd_vector;
        while (!temp_buffer.empty())
        {
            cmd_vector.push_back(temp_buffer.front());
            temp_buffer.pop();
        }
        return cmd_vector;
    }
    void set_ref(float x, float y, float z)
    {
        //TODO: put min/max values for X_r
        float rate = 0.1; // Scale factor for the speed of the chage of the formation
        x_r += x_r_base*x*rate; // Ex: hand_r is (0.1,0,0) relative to hand_stat and x_r_base is 0.5->
        y_r += y_r_base*y*rate; // x_r will increase with x_r*5 mm each time step
        z_r += z_r_base*z*rate;
    }
    void UpdateTransform(rclcpp::Time stamp)
    {
        geometry_msgs::msg::TransformStamped ts;
        ts.header.frame_id = "swarm";
        ts.header.stamp = stamp;
        ts.child_frame_id = name+"_ref";
        ts.transform.translation.x = x_r;
        ts.transform.translation.y = y_r;
        ts.transform.translation.z = z_r;
        ts.transform.rotation.x = 0.0;
        ts.transform.rotation.y = 0.0;
        ts.transform.rotation.z = 0.0;
        ts.transform.rotation.w = 1.0;
        tf_static_broadcaster->sendTransform(ts);
    }

};

class SwarmControl : public rclcpp::Node
{
  public:
    std::vector<std::string> names;
    std::vector<CF*> cfs;
    // Mode is changed with the angle of the left hand
    int mode; // 0: "Hold" the swarm does not react to the right hand. (NOT CURRENTLY USED)
              // 1: "Follow" the pose of the swarm changes with the pose of the right hand
              // 2: "Formation" the formation of the swarm changes with the movements of the right hand
    // Swarm origin relative to the right hand
    SwarmControl()
    : Node("swarm_control")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        mode = 1;
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        float swarm_x = 1.5;
        float swarm_y = 0.0;
        float swarm_z = -0.3;
        // Create static transform for swarm and hand_static
        // Hand static is a frame that is used as the base for the swarm frame, when in mode 2
        geometry_msgs::msg::TransformStamped t;
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = "hand_stat";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;
        tf_static_broadcaster_->sendTransform(t);
        t.header.frame_id = "hand_r";
        t.child_frame_id = "swarm";
        t.transform.translation.x = swarm_x;
        t.transform.translation.y = swarm_y;
        t.transform.translation.z = swarm_z;
        tf_static_broadcaster_->sendTransform(t);

        // The offset of the crazyflies relative to the swarm origin.
        // The node currently supports up to 20 crazyflies.
        float offset_x[19] = { 0.5, -0.5, -0.5,  0   , 0   , -0.5, 1, -1, -1,  0.5, 0.5,  0   , 0   , -0.5, -0.5, -1   , -1   , -1   , -1   };
        float offset_y[19] = { 0  , -0.5,  0.5, -0.25, 0.25,  0  , 0, -1,  1, -0.5, 0.5, -0.75, 0.75, -1  ,  1  , -0.75,  0.75, -0.25,  0.25};
        float offset_z[19] = { 0  ,  0  ,  0  ,  0   , 0   ,  0  , 0,  0,  0,  0  , 0  ,  0   , 0   ,  0  ,  0  ,  0   ,  0   ,  0   ,  0   };
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
        geometry_msgs::msg::TransformStamped transformStamped;
        for (size_t i = 0; i < names.size(); i++)
        {   
            std::cout << names[i] << std::endl;
            cf = new CF();
            cf->name = names[i];
            cf->tf_static_broadcaster = tf_static_broadcaster_;
            cf->x = 0;
            cf->y = 0;
            cf->z = 0;
            cf->x_r = offset_x[i]; cf->y_r = offset_y[i]; cf->z_r = offset_z[i];
            cf->x_r_base = offset_x[i]; cf->y_r_base = offset_y[i]; cf->z_r_base = offset_z[i];
            cf->yaw = 0;
            cmd_topic = "/cf/cmd_position";
            for (size_t j = 0; j < names[i].size()-2; j++) //Run the loop once if cfx, twice if cfxx and so on
                cmd_topic.insert(3+j,1,names[i][2+j]);
            // This is the reference frame for the crazyflies. In other words, the wanted position of the crazyflies in the swarm frame.
            t.header.frame_id = "swarm";
            t.child_frame_id = (names[i]+"_ref");
            t.transform.translation.x = offset_x[i];
            t.transform.translation.y = offset_y[i];
            t.transform.translation.z = offset_z[i];
            tf_static_broadcaster_->sendTransform(t);
            cf->pub = this->create_publisher<crazyflie_interfaces::msg::Position>(cmd_topic,qos); 
            cf->tf_static_broadcaster = tf_static_broadcaster_;
            cfs.push_back(cf);
        }
        rclcpp::sleep_for(2500ms);
        std::cout << "All crazyflies are initialized" << std::endl;
        timer_ = this->create_wall_timer(
        10ms, std::bind(&SwarmControl::swarm_callback, this));
        timer_slow_ = this->create_wall_timer(
        100ms, std::bind(&SwarmControl::mode_callback, this));
    }

  private:
    rclcpp::Subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_slow_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
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
                    names.push_back(robot_name);
            }
        }
    }

    void swarm_callback()
    {
        CF* cf;
        tf2::Quaternion cfq;
        float x,y,z;
        geometry_msgs::msg::PoseStamped cf_pose_swarm;
        cf_pose_swarm.header.frame_id = "swarm";
        geometry_msgs::msg::PoseStamped cf_pose_world;
        auto message = crazyflie_interfaces::msg::Position();

        switch (mode)// 0: hold, 1: pose, 2: formation
        {
        case 0:
        case 1:
            {        
            for (size_t i = 0; i < cfs.size(); i++)
            {
                cf = cfs[i];
                //Transforming cf from swarm frame to world frame
                geometry_msgs::msg::TransformStamped transformStamped = 
                tf_buffer_->lookupTransform("world", names[i]+"_ref", this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0));
                message.x = transformStamped.transform.translation.x;
                message.y = transformStamped.transform.translation.y;
                message.z = transformStamped.transform.translation.z;

                // Calculate the average of the message and the 5 previous messages
                float avg_x = 0.0;
                float avg_y = 0.0;
                float avg_z = 0.0;
                int count = 0;
                std::vector<crazyflie_interfaces::msg::Position> cf_buffer = cf->getCmdBuffer();
                // Iterate through the cf buffer
                for (int i = 0; i < cf_buffer.size(); i++) {
                    avg_x += cf_buffer[i].x;
                    avg_y += cf_buffer[i].y;
                    avg_z += cf_buffer[i].z;
                    count++;
                }

                // Add the current message to the average
                avg_x += message.x;
                avg_y += message.y;
                avg_z += message.z;
                count++;

                // Calculate the average
                avg_x /= count;
                avg_y /= count;
                avg_z /= count;

                // Update the message with the average values
                message.x = avg_x;
                message.y = avg_y;
                message.z = avg_z;
                // std::cout << "CF: " << names[i] << " x: " << message.x << " y: " << message.y << " z: " << message.z << std::endl;
                
                cf->pub->publish(message);
                cf->updateCmdBuffer(message.x, message.y, message.z, 0);
            }
            break;
            }
        case 2:
            {
            for (size_t i = 0; i < cfs.size(); i++)
            {
                cf = cfs[i];
                // Get the world coordinates from hand_r and hand_stat
                geometry_msgs::msg::TransformStamped hand_r_transformStamped = tf_buffer_->lookupTransform("hand_stat", "hand_r", this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0));
                float hand_r_x = -hand_r_transformStamped.transform.translation.x;
                float hand_r_y = -hand_r_transformStamped.transform.translation.y;
                float hand_r_z = -hand_r_transformStamped.transform.translation.z;

                cf->set_ref(hand_r_x, hand_r_y, hand_r_z);
                cf->UpdateTransform(this->get_clock()->now());
                
                //Transforming cf from swarm frame to world frame
                geometry_msgs::msg::TransformStamped transformStamped = 
                tf_buffer_->lookupTransform("world", names[i]+"_ref", this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0));
                message.x = transformStamped.transform.translation.x;
                message.y = transformStamped.transform.translation.y;
                message.z = transformStamped.transform.translation.z;
                // message.yaw = hand.yaw;

                float avg_x = 0.0;
                float avg_y = 0.0;
                float avg_z = 0.0;
                int count = 0;
                std::vector<crazyflie_interfaces::msg::Position> cf_buffer = cf->getCmdBuffer();
                // Iterate through the cf buffer
                for (int i = 0; i < cf_buffer.size(); i++) {
                    avg_x += cf_buffer[i].x;
                    avg_y += cf_buffer[i].y;
                    avg_z += cf_buffer[i].z;
                    count++;
                }

                // Add the current message to the average
                avg_x += message.x;
                avg_y += message.y;
                avg_z += message.z;
                count++;

                // Calculate the average
                avg_x /= count;
                avg_y /= count;
                avg_z /= count;

                // Update the message with the average values
                message.x = avg_x;
                message.y = avg_y;
                message.z = avg_z;
                
                cf->pub->publish(message);
                cf->updateCmdBuffer(message.x, message.y, message.z, 0);
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
        geometry_msgs::msg::TransformStamped transformStamped = 
                tf_buffer_->lookupTransform("world", "hand_l", this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0));
        x = 2*(transformStamped.transform.rotation.x*transformStamped.transform.rotation.z - transformStamped.transform.rotation.w*transformStamped.transform.rotation.y);
        y = 2*(transformStamped.transform.rotation.y*transformStamped.transform.rotation.z + transformStamped.transform.rotation.w*transformStamped.transform.rotation.x);
        if(x < -0.7)
        {
            if (mode != 0)
            {
                geometry_msgs::msg::TransformStamped transformStamped = 
                tf_buffer_->lookupTransform("hand_stat", "swarm", this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0));
                tf_static_broadcaster_->sendTransform(transformStamped);
            }
            mode = 0;
        }
        else if(y < -0.7)
        {
            if (mode != 2)
            {
                geometry_msgs::msg::TransformStamped transformStamped = 
                tf_buffer_->lookupTransform("hand_stat", "swarm", this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0));
                tf_static_broadcaster_->sendTransform(transformStamped);
            }
            mode = 2;
        }
        else
        {
            if (mode != 1)
            {
                //Add the translation. The displacement of the right hand since the "hold" was initiated.
                geometry_msgs::msg::TransformStamped transformStamped = 
                tf_buffer_->lookupTransform("hand_r", "swarm", this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0));
                tf_static_broadcaster_->sendTransform(transformStamped);
            }
            // Update the static hand pose.
            geometry_msgs::msg::TransformStamped transformStamped = 
                tf_buffer_->lookupTransform("world", "hand_r", this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0));
            transformStamped.child_frame_id = "hand_stat";
            tf_static_broadcaster_->sendTransform(transformStamped);
            mode = 1;
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

