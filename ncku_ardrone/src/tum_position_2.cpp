#include "ros/ros.h"
#include "ros/service_callback_helper.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "tum_ardrone/filter_state.h"
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
#include <moveit_msgs/DisplayTrajectory.h> 
#include <math.h>

#define ptam_x      0   // Current x-position from ptam
#define ptam_y      1   // Unit of position is 'm'
#define ptam_z	    2
#define ptam_yaw    3   // Unit of yaw is degree, range is from -180 ~ 180
#define PI 3.14159265358979

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
    void takeoff();
    void land();
    void reset();
    void clear_targets();
    void Free_pointers();
    void Manual(int com_char);
    void targets2tum_ardrone(float x, float y, float z, float yaw);
    float Quaternion2Yaw(float qz, float qw);
    int keyboard(); 
    int index2;
    int path_follow_index;
    float Position[4];
    float Target[4];
    float *path_x;
    float *path_y; 
    float *path_z;
    float *path_w;
    
    char manual_selection;

    ros::NodeHandle n;                               // so that we dont make a copy while passing this as reference
    ros::Subscriber nav_sub;                         // subscribing to the pos_estimate_data from the tum_ardrone topic
    ros::Subscriber path_sub;			     // subscribing to the display_planned_path from Moveit topic
    ros::Publisher cmd_vel_pub;                      // this will be publishing the command velocity to the drone
    ros::Publisher tum_ardrone_pub;		     // this will be publishing the Target to the TUM package
    geometry_msgs::Twist twist_manual, twist_pos;    // the message we use to send command to the drone
    
    void nav_callback(const tum_ardrone::filter_stateConstPtr tum_msg);
    void path_callback(const moveit_msgs::DisplayTrajectory msg);
};


//-------Set up all members in the "ArdroneControl class"------
ArdroneControl::ArdroneControl()
{
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  nav_sub = n.subscribe("/ardrone/predictedPose", 50, &ArdroneControl::nav_callback, this);
  path_sub = n.subscribe("/move_group/display_planned_path", 1, &ArdroneControl::path_callback, this);
  tum_ardrone_pub = n.advertise<std_msgs::String>("tum_ardrone/com",50, true);

  twist_manual.linear.x = twist_manual.linear.y = twist_manual.linear.x = 0.0;
  Target[ptam_x] = Target[ptam_y] = Target[ptam_z] = Target[ptam_yaw] = 0.0;
  index2 = 0;
  path_follow_index = 0;

  path_x = (float*)malloc(20 * sizeof(float));
  path_y = (float*)malloc(20 * sizeof(float));
  path_z = (float*)malloc(20 * sizeof(float));
  path_w = (float*)malloc(20 * sizeof(float));
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
}


void ArdroneControl::clear_targets()
{
	std_msgs::String msg;
	std::stringstream ss;
	ss << "c clearCommands";
	msg.data = ss.str();
	tum_ardrone_pub.publish(msg);
}


void ArdroneControl::targets2tum_ardrone(float x, float y, float z, float yaw)
{
    std::stringstream ss;
    ss <<"c goto " << x << " " << y << " " << z << " "<< 0;
    std::string s = ss.str();
    ROS_INFO(s.c_str());
    std_msgs::String com;
    com.data = s.c_str();
    tum_ardrone_pub.publish(com);
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


//-------Function for Manual Control-----------
void ArdroneControl::Manual(int com_char)
{
	if(ros::ok())
	{
        bool index = 0;
		switch(com_char)
		{
			case 't':
				takeoff();
				index = 0;
			break;

			case 'r':
				reset();
				index = 0;
			break;

			case 'l':	
				land();
				index = 0;
			break;

            		case 'w': //forward
			if( twist_manual.linear.x < 0.9 )
			{                	
				twist_manual.linear.x = twist_manual.linear.x + 0.1;
				ROS_INFO("move forward");
				index = 1;
			}		
			break;

            		case 's': //backward
			if( twist_manual.linear.x > -0.9 )
			{
				twist_manual.linear.x = twist_manual.linear.x - 0.1;
				ROS_INFO("move backward");
				index = 1;
			}
			break;

            		case 'a': //left
			if( twist_manual.linear.y < 0.9 )
			{
				twist_manual.linear.y = twist_manual.linear.y + 0.1;
				ROS_INFO("move left");
				index = 1;
			}
			break;

            		case 'd': //right
			if( twist_manual.linear.y > -0.9 )
			{
				twist_manual.linear.y = twist_manual.linear.y - 0.1;
				ROS_INFO("move right");
				index = 1;
			}
			break;

            		case '+': //up
			if( twist_manual.linear.z < 0.9 )
			{
				twist_manual.linear.z = twist_manual.linear.z + 0.1;
				ROS_INFO("move up");
				index = 1;
			}
			break;

            		case '-': //down
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

//-----------Function for releasing the memory of pointers---------- 
void ArdroneControl::Free_pointers()
{
    if(path_x != NULL)
    {	
	free(path_x);
	path_x = NULL;
    }
    if(path_y != NULL)
    {	
	free(path_y);
	path_y = NULL;
    }
    if(path_z != NULL)
    {	
	free(path_z);
	path_z = NULL;
    }
    if(path_w != NULL)
    {	
	free(path_w);
	path_w = NULL;
    }
}

//-----------Function for calculating the Yaw angle(degree)---------
float ArdroneControl::Quaternion2Yaw(float qz, float qw)
{
    return(atan2(2*qz*qw,1-2*qz*qz)*180/PI);
}

//-----------Function for subscribing the path from MoveIt!------------
void ArdroneControl::path_callback(const moveit_msgs::DisplayTrajectory msg)
{
    int i = 0;
    int i_next = 0;
    int point_size = msg.trajectory[0].multi_dof_joint_trajectory.points.size();
    float qz = 0.0;
    float qw = 0.0;
    float distance = 0.0;
    
    Free_pointers();
    
    path_x = (float*)malloc(point_size * sizeof(float));
    path_y = (float*)malloc(point_size * sizeof(float));
    path_z = (float*)malloc(point_size * sizeof(float));
    path_w = (float*)malloc(point_size * sizeof(float));
    for( i = 0; i < point_size; i++)
    {
	path_x[i] = -msg.trajectory[0].multi_dof_joint_trajectory.points[i].transforms[0].translation.x;
	path_y[i] = -msg.trajectory[0].multi_dof_joint_trajectory.points[i].transforms[0].translation.y;	
	path_z[i] = msg.trajectory[0].multi_dof_joint_trajectory.points[i].transforms[0].translation.z;
	       qz = msg.trajectory[0].multi_dof_joint_trajectory.points[i].transforms[0].rotation.z;
	       qw = msg.trajectory[0].multi_dof_joint_trajectory.points[i].transforms[0].rotation.w;
        path_w[i] = Quaternion2Yaw(qz, qw);
    }
    ROS_INFO("Get the Path, the total size of points is: %d",point_size);

    if(path_follow_index == 1)
    {
	i = 0;	
	i_next = 1;
	
	clear_targets();
	
	ROS_INFO("Start to follow the path!!");
	while( i_next < point_size )
        {
	    distance = sqrtf( (path_x[i_next]-path_x[i])*(path_x[i_next]-path_x[i]) + (path_y[i_next]-path_y[i])*(path_y[i_next]-path_y[i]) + (path_z[i_next]-path_z[i])*(path_z[i_next]-path_z[i]) );
	    if(distance <= 0.2)
		i_next++;
	    if(distance > 0.2)
	    {
		targets2tum_ardrone(path_x[i_next],path_y[i_next],path_z[i_next],path_w[i_next]);		
		i = i_next;		
		i_next++;
	    }
        }
	ROS_INFO("All of targets were published!!");
    }
}



//----------------Call-back function bases on tum_rdrone's navadate---------------------
void ArdroneControl::nav_callback(const tum_ardrone::filter_stateConstPtr tum_msg)
{
  	
    if(index2 == 0)
	{
        ROS_INFO("Mode Selection--> 't'->position control, 'n'->manual control, 'p'->path follow:");
		manual_selection = getchar();
        if(manual_selection == 't' || manual_selection == 'n' || manual_selection == 'p')
			index2 = 1;
	}
    if(manual_selection == 't')
    {  	
        Position[ptam_x] = float(tum_msg->x);
        Position[ptam_y] = float(tum_msg->y);
        Position[ptam_z] = float(tum_msg->z);
        Position[ptam_yaw] = float(tum_msg->yaw);
        
        ROS_INFO("Please enter your target like-->0.5 -0.5 0 0 :");
        float parameters[5];
        char tar_command[15];
        gets(tar_command);
        if (tar_command[0] == 'x')
            index2 = 0;
        if (tar_command[0] == 'l')
            land();
        if(sscanf(tar_command,"%f %f %f %f",&parameters[0], &parameters[1], &parameters[2], &parameters[3]) == 4)
        {
		targets2tum_ardrone(parameters[0],parameters[1],parameters[2],parameters[3]);
                ROS_INFO("Set up the Target!!");
        }  
    }  
    if(manual_selection == 'n')
    {
	ROS_INFO("Manual Mode--> w a s d; t:takeoff, l:land, r:reset  (x for \"STOP\" ):");		
	int com_char = keyboard();
	if( com_char == 'x')
	{
		ROS_INFO("Break");            		
		index2 = 0;
	}
	if( com_char != 'x')			
    		Manual(com_char);
    }  
    if(manual_selection == 'p')
    {
	ROS_INFO("Path Mode--> s: start following the path, p: pause   (x for \"STOP\" ):");
        int com_char = keyboard();
	if( com_char == 'x')
	{
		ROS_INFO("Break"); 
		path_follow_index = 0;
		clear_targets();           		
		index2 = 0;
	}
        if( com_char == 's')
	{	
		path_follow_index = 1;
		ROS_INFO("START");
	}
	if( com_char == 'p')
	{
		path_follow_index = 0;
		ROS_INFO("Pause");
	}
    }
}


//------------Main function-----------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tum_position_2");
  ROS_INFO("START");
  ROS_INFO("Wait for connecting the ARDrone!!");  
  ArdroneControl ardrone_control;

  ros::AsyncSpinner spinner(2); // Use 2 threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
