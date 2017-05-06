#include "ros/ros.h"
#include "ros/service_callback_helper.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Empty.h"
#include "ardrone_autonomy/LedAnim.h"
#include <ardrone_autonomy/Navdata.h>
#include <string>
#include <sstream>
#include <geometry_msgs/Twist.h>       //For motion command, except takeoff, land and reset
#include <termios.h>                   //For keyboard

//====== Set a "class" for different kinds of functions ======
class ArdroneControl
{
  public:
    ArdroneControl();

  private:
    void Manual(int com_char);
    void takeoff();
    void land();
    void Send_command();
    int keyboard(); 
    bool flying;

    ros::NodeHandle n;                   // so that we dont make a copy while passing this as reference, and work on this one directly
    ros::Subscriber nav_sub;             // subscribing to the joy message and the navdata from the joystick and ardrone topic
    ros::Publisher cmd_vel_pub;          // this will be publishing the command velocity to the drone
    geometry_msgs::Twist twist_manual;   // the message we use to send command to the drone
    
    void nav_callback(const ardrone_autonomy::NavdataConstPtr& nav_msg);
};



//====== Set up all members in the "ArdroneControl class" ======
//---Initialize all parameters ---
ArdroneControl::ArdroneControl()
{
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  nav_sub = n.subscribe("/ardrone/navdata", 1, &ArdroneControl::nav_callback, this);
  twist_manual.linear.x = twist_manual.linear.y = twist_manual.linear.z = twist_manual.angular.z = 0.0;
  flying = 0;
}
 
//--- Function for "Take off" ---
void ArdroneControl::takeoff()
{
  ros::Rate poll_rate(100);
  ros::Publisher takeoff = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1, true);
  takeoff.publish(std_msgs::Empty());
  while(takeoff.getNumSubscribers() == 0)
      poll_rate.sleep();  
  takeoff.shutdown();
  ROS_INFO("TAKEOFF");
  flying = 1;
}

//--- Function for "Land" ---
void ArdroneControl::land()
{
  ros::Rate poll_rate(100);
  ros::Publisher landing = n.advertise<std_msgs::Empty>("ardrone/land", 1, true);
  landing.publish(std_msgs::Empty());
  while(landing.getNumSubscribers() == 0)
      poll_rate.sleep();  
  landing.shutdown();
  ROS_INFO("LAND");
  flying = 0;
}

//--- Function for detecting the keyboard input without "Enter" ---
int ArdroneControl::keyboard()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings
  int c = getchar();                         // read character (non-blocking)
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

//--- Function for sending commands to ARDrone --- 
void ArdroneControl::Send_command()
{
        cmd_vel_pub.publish(twist_manual);
        std::stringstream ss;
        ss << "Output: " <<twist_manual.linear.x<< " " <<twist_manual.linear.y<< " " <<twist_manual.linear.z<< " " <<twist_manual.angular.z;
        std::string s = ss.str();
	ROS_INFO(s.c_str());
        ROS_INFO("\n Command:");
}

//--- Function for manual control ---
void ArdroneControl::Manual(int com_char)
{
	if(ros::ok())
	{
		switch(com_char)
		{
			// Landing			
			case 'l':
			if(flying == 1)	
				land();
			break;

			case 't':
			if(flying == 0)
				takeoff();
			break;

			// Moving up
			case '+': 
			if( twist_manual.linear.z < 0.9 )
			{
				twist_manual.linear.z = twist_manual.linear.z + 0.1;
				ROS_INFO("move up");
				Send_command();
			}
			break;

			// Moving down
			case '-': 
			if( twist_manual.linear.z > -0.9 )
			{
				twist_manual.linear.z = twist_manual.linear.z - 0.1;
				ROS_INFO("move down");
				Send_command();
			}
			break;
		}
	  }
}


void ArdroneControl::nav_callback(const ardrone_autonomy::NavdataConstPtr& nav_msg)
{	
	int com_char = keyboard();       					
	ROS_INFO(" 't'->takeoff, 'l'->land, '+'->up, '-'->down :");
	Manual(com_char);
}


//------------Main function-----------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ARDrone_template");
  ROS_INFO("START");
  
  ArdroneControl ardrone_control;

  ros::spin();

  return 0;
}
