//#include <SDL2/SDL.h>
#include "SDL.h"
#include <cinttypes>
#include <iostream>
#include <string.h>

#include <cmath>

#include <memory>
#include <iostream>
#include <fstream>
#include <chrono>
#include <functional>
#include <Eigen/Dense>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "motion_capture_tracking_interfaces/msg/named_pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "crazyflie_interfaces/msg/position.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace Eigen;

void OnControllerButton( const SDL_ControllerButtonEvent sdlEvent ){
    // Button presses and axis movements both sent here as SDL_ControllerButtonEvent structures
	std::cout << "Button " << sdlEvent.button << std::endl;
}

void OnControllerAxis( const SDL_ControllerAxisEvent sdlEvent, SDL_GameController*& pad ){
    // Axis movements and button presses both sent here as SDL_ControllerAxisEvent structures
	std::cout << "Throttle: " << SDL_GameControllerGetAxis(pad, SDL_CONTROLLER_AXIS_LEFTX) << std::endl;
	std::cout << "Yaw : " << SDL_GameControllerGetAxis(pad, SDL_CONTROLLER_AXIS_RIGHTX) << std::endl;
	std::cout << "Pitch : " << SDL_GameControllerGetAxis(pad, SDL_CONTROLLER_AXIS_TRIGGERLEFT) << std::endl;
	std::cout << "Roll : " << SDL_GameControllerGetAxis(pad, SDL_CONTROLLER_AXIS_LEFTY) << std::endl;
}

class ControllerState{
	public:
		float th,v_x,v_y,v_yaw;
		SDL_GameController* pad;
		ControllerState(SDL_GameController* cont){
			th=0;
			v_x=0;
			v_y=0;
			v_yaw=0;
			pad = cont;
		}
		void update(){
			th = SDL_GameControllerGetAxis(pad, SDL_CONTROLLER_AXIS_LEFTX);
			std::cout << th << std::endl;
			th = (th+32768)/65535*45000;
			v_x = SDL_GameControllerGetAxis(pad, SDL_CONTROLLER_AXIS_TRIGGERLEFT);
			v_x = v_x/32767*2-1;
			v_y = SDL_GameControllerGetAxis(pad, SDL_CONTROLLER_AXIS_LEFTY);
			v_y = v_y/32768;
			v_yaw = SDL_GameControllerGetAxis(pad, SDL_CONTROLLER_AXIS_RIGHTX);
			v_yaw = v_yaw/32768;
		}
};

class TeleopFrSky : public rclcpp::Node
{
  public:
    SDL_GameController *pad;
    ControllerState* state;
    TeleopFrSky()
    : Node("teleop_frsky")
    {
      	publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cf1/cmd_vel_legacy", 10);
     	timer_ = this->create_wall_timer(20ms, std::bind(&TeleopFrSky::timer_callback,this));
	SDL_Init(SDL_INIT_GAMECONTROLLER);
	int id = 0;
	pad = SDL_GameControllerOpen(id);
	const char* instanceName = SDL_GameControllerName(pad);
	std::cout << instanceName << std::endl;
	SDL_Event sdlEvent;
	state = new ControllerState(pad);
    }
  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Twist();
      state->update();
      message.linear.x = state->v_x;
      message.linear.y = state->v_y;
      message.linear.z = state->th;
      message.angular.z = state->v_yaw;
      publisher_->publish(message);
	/*if( SDL_PollEvent( &sdlEvent ) ) {
		switch( sdlEvent.type ) {
		case SDL_CONTROLLERBUTTONDOWN:
		    OnControllerButton( sdlEvent.cbutton );
		case SDL_CONTROLLERBUTTONUP:
		    OnControllerButton( sdlEvent.cbutton );
		case SDL_CONTROLLERAXISMOTION:
		    OnControllerAxis( sdlEvent.caxis,pad );
		}
	    }
	    */
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopFrSky>());
  rclcpp::shutdown();
  SDL_Quit();
  return 0;
}

