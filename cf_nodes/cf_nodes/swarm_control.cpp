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
    float x_r_base, y_r_base, z_r_base;

    rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr pub;
    rclcpp::Client<crazyflie_interfaces::srv::Takeoff>::SharedPtr takeoff_cli;
    rclcpp::Client<crazyflie_interfaces::srv::GoTo>::SharedPtr goto_cli;
    // Function to change cf's pose in the formation
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
        msg.x = x;
        msg.y = y;
        msg.z = z;
        msg.yaw = yaw;
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

struct Pose
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
    Pose hand_r;
    Pose hand_l;
    Pose hand_stat; // Used to store the most recent pose of the right hand when in mode 1
    // Mode is changed with the angle of the left hand
    int mode; // 0: "Hold" the swarm does not react to the right hand
              // 1: "Follow" the pose of the swarm changes with the pose of the right hand
              // 2: "Formation" the formation of the swarm changes with the movements of the right hand
    // Swarm origin relative to the right hand
    Pose swarm;

    SwarmControl()
    : Node("swarm_control")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        mode = 1;
        hand_r.x = 0.0;
        hand_r.y = 0.0;
        hand_r.z = 0.0;
        hand_r.q = tf2::Quaternion(0, 0, 0, 1);
        hand_stat.x = 0.0;
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        hand_stat.y = 0.0;
        hand_stat.z = 0.0;
        hand_stat.q = tf2::Quaternion(0, 0, 0, 1);
        swarm.x = 1.5;
        swarm.y = 0.0;
        swarm.z = -0.3;
        // Create static transform for swarm and hand_static
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
        t.transform.translation.x = swarm.x;
        t.transform.translation.y = swarm.y;
        t.transform.translation.z = swarm.z;
        tf_static_broadcaster_->sendTransform(t);

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
        std::string takeoff_service, goto_service;
        auto request = std::make_shared<crazyflie_interfaces::srv::Takeoff::Request>();
        request->height = 0.5;
        request->duration = rclcpp::Duration::from_seconds(2.0);
        auto goto_request = std::make_shared<crazyflie_interfaces::srv::GoTo::Request>();
        geometry_msgs::msg::Point goal;
        goal.x = 0.0;
        goal.y = 0.0;
        goal.z = 1.0;
        goto_request->goal = goal;
        goto_request->yaw = 0.0;
        goto_request->duration = rclcpp::Duration::from_seconds(3.0);
        goto_request->relative = false;
        // goto_request->group_mask = 0;
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
            takeoff_service = "/cf/takeoff";
            goto_service = "/cf/go_to";
            for (size_t j = 0; j < names[i].size()-2; j++) //Run the loop once if cfx, twice if cfxx and so on
            {
                cmd_topic.insert(3+j,1,names[i][2+j]);
                takeoff_service.insert(3+j,1,names[i][2+j]);
                goto_service.insert(3+j,1,names[i][2+j]);
            }
            t.header.frame_id = "swarm";
            t.child_frame_id = (names[i]+"_ref");
            t.transform.translation.x = offset_x[i];
            t.transform.translation.y = offset_y[i];
            t.transform.translation.z = offset_z[i];
            tf_static_broadcaster_->sendTransform(t);
            cf->pub = this->create_publisher<crazyflie_interfaces::msg::Position>(cmd_topic,qos); 
            cf->takeoff_cli = this->create_client<crazyflie_interfaces::srv::Takeoff>(takeoff_service);
            cf->goto_cli = this->create_client<crazyflie_interfaces::srv::GoTo>(goto_service);
            cf->tf_static_broadcaster = tf_static_broadcaster_;
            cfs.push_back(cf);
            cfs_map[names[i]] = cf;
            rclcpp::sleep_for(1000ms); // Sleep to make sure the transform is published before the cf starts to takeoff
            transformStamped = 
                tf_buffer_->lookupTransform("world", names[i]+"_ref", this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0));
            // cf->takeoff_cli->async_send_request(request);
        }
        rclcpp::sleep_for(2500ms);
        // for (size_t i = 0; i < names.size(); i++)
        // {   
        //     transformStamped = 
        //         tf_buffer_->lookupTransform("world", names[i]+"_ref", this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0));
        //     goto_request->goal.x = transformStamped.transform.translation.x;
        //     goto_request->goal.y = transformStamped.transform.translation.y;
        //     goto_request->goal.z = transformStamped.transform.translation.z;
        //     cfs[i]->goto_cli->async_send_request(goto_request);
        // }
        // rclcpp::sleep_for(3500ms);
        std::cout << "All crazyflies are initialized" << std::endl;
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
                {
                    names.push_back(robot_name);
                }
            }
        }
    }

    void swarm_callback()
    {
        CF* cf;
        tf2::Quaternion cfq;
        float x;
        float y;
        float z;
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
        x = 2*(hand_l.q[0]*hand_l.q[2] - hand_l.q[3]*hand_l.q[1]);
        y = 2*(hand_l.q[1]*hand_l.q[2] + hand_l.q[3]*hand_l.q[0]);
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
        // std::cout << "Mode: " << mode << std::endl;
        // std::cout << "Hand right: " << hand_r.x << "," << hand_r.y << "," << hand_r.z << std::endl;
        // std::cout << "Hand static: " << hand_stat.x << "," << hand_stat.y << "," << hand_stat.z << std::endl;
        // std::cout << "Swarm: " << swarm.x << "," << swarm.y << "," << swarm.z << std::endl;
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
        // This method should probably be removed in the future, as all relevant information
        // can already be accessed through tf2
        // I think it can be removed now actually. No, hand_l is stll used.
        Vector3f target; 
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
            else if (msg.poses[i].name[0]=='c' && msg.poses[i].name[1]=='f'){
                cfs_map[msg.poses[i].name]->x = msg.poses[i].pose.position.x;
                cfs_map[msg.poses[i].name]->y = msg.poses[i].pose.position.y;
                cfs_map[msg.poses[i].name]->z = msg.poses[i].pose.position.z;
                // std::cout << msg.poses[i].name << ": " << cfs_map[msg.poses[i].name]->x << "," << cfs_map[msg.poses[i].name]->y << "," << cfs_map[msg.poses[i].name]->z << std::endl;
            }
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

