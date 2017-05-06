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
#include <stdio.h>  
#include <termios.h>  
#include <unistd.h>  
#include <fcntl.h>  


#define error_tag_X 	0
#define error_tag_Y 	1
#define error_tag_Z	2
#define error_tag_yaw	3
#define follow_distance 200.0	      // 100 cm

//--------------kbhit function model in Linux----------------
int kbhit(void)  
{  
  struct termios oldt, newt;  
  int ch;  
  int oldf;  
  tcgetattr(STDIN_FILENO, &oldt);  
  newt = oldt;  
  newt.c_lflag &= ~(ICANON | ECHO);  
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);  
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);  
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);  
  ch = getchar();  
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  
  fcntl(STDIN_FILENO, F_SETFL, oldf);  
  if(ch != EOF)  
  {  
    ungetc(ch, stdin);  
    return 1;  
  }  
  return 0;  
}  




//------Set a "class" for different kinds of functions--------
class ArdroneControl
{
  public:
    ArdroneControl();

  private:
    void Manual(int com_char);
    void takeoff();
    void land();
    void reset();
    void Tag_follower(int tag_index);
    int keyboard(); 
    int index2;
    int tag_follow_index;
    bool flying;
    float error[4];
    float last_error[4];
    float x_kp, x_ki, x_kd, y_kp, y_ki, y_kd, z_kp, z_ki, z_kd;
    float x_integral, y_integral, z_integral;
    float x_derivative, y_derivative, z_derivative;
    float x_output, y_output, z_output;
    char manual_selection;
    

    ros::NodeHandle n;                               // so that we dont make a copy while passing this as reference, and work on this one directly
    ros::Subscriber nav_sub;                         // subscribing to the joy message and the navdata from the joystick and ardrone topic
    ros::Publisher cmd_vel_pub;                      // this will be publishing the command velocity to the drone
    geometry_msgs::Twist twist_manual, twist_auto;   // the message we use to send command to the drone
    
    void nav_callback(const ardrone_autonomy::NavdataConstPtr& nav_msg);
};


//-------Set up all members in the "ArdroneControl class"------
ArdroneControl::ArdroneControl()
{
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  nav_sub = n.subscribe("/ardrone/navdata", 1, &ArdroneControl::nav_callback, this);
  twist_manual.linear.x = twist_manual.linear.y = twist_manual.linear.x = 0.0;
  twist_auto.linear.x = twist_auto.linear.y = twist_auto.linear.x = 0.0;
  flying = 0;
  index2 = 0;
  // Initialize PID control variables
  tag_follow_index = 0;
  x_kp = 0.5/10000.0;
  x_ki = 0.0;
  x_kd = 2.0/10000.0;
  x_integral = 0.0;
  x_derivative = 0.0;
  last_error[error_tag_X] = 0.0;
  x_output = 0.0;

  y_kp = 1.0;
  y_ki = 0.0;
  y_kd = 0.0;
  y_integral = 0.0;
  y_derivative = 0.0;
  last_error[error_tag_Y] = 0.0;
  y_output = 0.0;

  z_kp = 0.0005;
  z_ki = 0.0;
  z_kd = 0.001;
  z_integral = 0.0;
  z_derivative = 0.0;
  last_error[error_tag_Z] = 0.0;
  z_output = 0.0;
}

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

void ArdroneControl::reset()
{
  ROS_INFO("RESET");
  ros::Rate poll_rate(100);

  ros::Publisher reset = n.advertise<std_msgs::Empty>("ardrone/reset", 1, true);
  reset.publish(std_msgs::Empty());

  while(reset.getNumSubscribers() == 0)
      poll_rate.sleep();  
  reset.shutdown();

  
}

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


//------Function for detecting the keyboard input without "Enter"----------
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

void ArdroneControl::Manual(int com_char)
{
	if(ros::ok())
	{
	  	bool index = 0;
		switch(com_char)
		{
			case 't':
			if(flying == 0)
			{	
				takeoff();
				index = 0;
			}
			break;

			case 'r':
			if(flying == 0)
			{	
				reset();
				ROS_INFO("Reset Finish !!");
				index = 0;
			}
			break;

			case 'l':
			if(flying == 1)
			{	
				land();
				index = 0;
			}
			break;

			case 'e': //前
			if( twist_manual.linear.x < 0.9 )
			{                	
				twist_manual.linear.x = twist_manual.linear.x + 0.1;
				ROS_INFO("move forward");
				index = 1;
			}		
			break;

			case 'd': //後
			if( twist_manual.linear.x > -0.9 )
			{
				twist_manual.linear.x = twist_manual.linear.x - 0.1;
				ROS_INFO("move backward");
				index = 1;
			}
			break;

			case 's': //左
			if( twist_manual.linear.y < 0.9 )
			{
				twist_manual.linear.y = twist_manual.linear.y + 0.1;
				ROS_INFO("move left");
				index = 1;
			}
			break;

			case 'f': //右
			if( twist_manual.linear.y > -0.9 )
			{
				twist_manual.linear.y = twist_manual.linear.y - 0.1;
				ROS_INFO("move right");
				index = 1;
			}
			break;

			case '+': 
			if( twist_manual.linear.z < 0.9 )
			{
				twist_manual.linear.z = twist_manual.linear.z + 0.1;
				ROS_INFO("move up");
				index = 1;
			}
			break;

			case '-': 
			if( twist_manual.linear.z > -0.9 )
			{
				twist_manual.linear.z = twist_manual.linear.z - 0.1;
				ROS_INFO("move down");
				index = 1;
			}
			break;
		}
	  
		if( index == 1)
		{
		        cmd_vel_pub.publish(twist_manual);
                        std::stringstream ss;
		        ss << "Output: " <<twist_manual.linear.x<< " " <<twist_manual.linear.y<< " " <<twist_manual.linear.z;
		        std::string s = ss.str();
			ROS_INFO(s.c_str());
                        ROS_INFO("\n Command:");
		} 
	  }
}


void ArdroneControl::Tag_follower(int tag_index)
{
	if( tag_index == 0 )
	{		
		twist_auto.linear.x = 0.0;
		twist_auto.linear.y = 0.0;
		twist_auto.linear.z = 0.0;
		cmd_vel_pub.publish(twist_auto);
		ROS_INFO("Do not detect the tag!!");
	}	
	if( tag_index == 1 )
	{
		//PID control law for y-axis (for body)		
		x_integral = 0.7 * x_integral + error[error_tag_X];
		x_derivative = error[error_tag_X] - last_error[error_tag_X];
		x_output = x_kp * error[error_tag_X] + x_ki * x_integral + x_kd * x_derivative;
		twist_auto.linear.y = x_output;
		if( twist_auto.linear.y > 1.0 )
			twist_auto.linear.y = 1.0;
		if( twist_auto.linear.y < -1.0 )
			twist_auto.linear.y = -1.0;
		
		//PID control law for z-axis		
		y_integral = 0.7 * y_integral + error[error_tag_Y];
		y_derivative = error[error_tag_Y] - last_error[error_tag_Y];
		y_output = y_kp * error[error_tag_Y] + y_ki * y_integral + y_kd * y_derivative;
		twist_auto.linear.z = y_output/10000.0;
		if( twist_auto.linear.z > 1.0 )
			twist_auto.linear.z = 1.0;
		if( twist_auto.linear.z < -1.0 )
			twist_auto.linear.y = -1.0;
	
		//PID control law for x-axis		
		z_integral = 0.7 * z_integral + error[error_tag_Z];
		z_derivative = error[error_tag_Z] - last_error[error_tag_Z];
		z_output = z_kp * error[error_tag_Z] + z_ki * z_integral - z_kd * z_derivative;
		twist_auto.linear.x = z_output;
		if( twist_auto.linear.x > 1.0 )
			twist_auto.linear.x = 1.0;
		if( twist_auto.linear.x < -1.0 )
			twist_auto.linear.x = -1.0;


		/*char forward_back, left_right, up_down;		
		if( twist_auto.linear.x >= 0.0 )
			forward_back = 'f';
		if( twist_auto.linear.x < 0.0 )
			forward_back = 'b';

		if( twist_auto.linear.y >= 0.0 )
			left_right = 'l';
		if( twist_auto.linear.y < 0.0 )
			left_right = 'r';

		if( twist_auto.linear.z >= 0.0 )
			up_down = 'u';
		if( twist_auto.linear.z < 0.0 )
			up_down = 'd';		*/
		
		cmd_vel_pub.publish(twist_auto);		
		std::stringstream ss;
  		ss  << twist_auto.linear.x << " " << twist_auto.linear.y << " " << twist_auto.linear.z;
  		std::string s = ss.str();
		ROS_INFO(s.c_str());
  		
	}
}


void ArdroneControl::nav_callback(const ardrone_autonomy::NavdataConstPtr& nav_msg)
{
  	
        if(index2 == 0)        
	{
		ROS_INFO("Mode Selection--> 'a' for tag dection; 'n' for manual (x for \"STOP\" ):");
		manual_selection = getchar();
		if(manual_selection == 'a' || manual_selection == 'n')		
			index2 = 1;
	}
	if(manual_selection == 'a')
	{  	
		if(nav_msg->tags_width.empty())	
			tag_follow_index = 0;	
		if(!nav_msg->tags_width.empty())
		{		
			error[error_tag_X] = 500.0 - float(nav_msg->tags_xc[0]);
			error[error_tag_Y] = 500.0 - float(nav_msg->tags_yc[0]);
			error[error_tag_Z] = follow_distance - float(nav_msg->tags_distance[0]);
			tag_follow_index = 1;
		}
                
		if(kbhit())
		{
            		char iput = getchar();
            		if(iput=='x')
			{    ROS_INFO("Break");            		
			     index2 = 0;
			     tag_follow_index = 0;
                        }
			if(iput=='l')
			{                		
			     land();
			     ROS_INFO("LAND");			     
			     index2 = 0;
			     tag_follow_index = 0;
                        }
		} 
		Tag_follower(tag_follow_index);
	}
	
	if(manual_selection == 'n')
        {
		int com_char = keyboard();
		if( com_char == 'x')
		{
			ROS_INFO("Break");            		
			index2 = 0;
		}
		if( com_char != 'x')			
		{
		        ROS_INFO("Manual Mode--> e d s f; t:takeoff, l:land, r:reset  (x for \"STOP\" ):");
			Manual(com_char);
		}
	}  
	
}


//------------Main function-----------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_04");
  ROS_INFO("START");
  
  ArdroneControl ardrone_control;

  ros::spin();

  return 0;
}
