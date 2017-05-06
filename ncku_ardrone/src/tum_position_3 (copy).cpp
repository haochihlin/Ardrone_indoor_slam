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

//Kbhit() and Keyboard()
#include <termios.h>                   
#include <stdio.h>   
#include <unistd.h>  
#include <fcntl.h> 

#include <moveit_msgs/DisplayTrajectory.h> 
#include <math.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

//MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//Define parameters
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
    void Plan_request(float start_x, float start_y, float start_qz, float start_qw, float end_x, float end_y, float end_yaw);
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
    float goal_x;
    float goal_y;
    float goal_yaw;
    float lsd_curr_pos[4];
    float manual_com[4];
    float min_corner[3];
    float max_corner[3];
    int replan_index;

    std::vector<std::string> joint_name;
    
    char manual_selection;

    ros::NodeHandle n;                               // so that we dont make a copy while passing this as reference
    ros::Subscriber nav_sub;                         // subscribing to the pos_estimate_data from the tum_ardrone topic
    ros::Subscriber lsd_pose_sub;
    ros::Subscriber lsd_PC2_sub;
    ros::Subscriber path_sub;			     // subscribing to the display_planned_path from Moveit topic
    ros::Publisher cmd_vel_pub;                      // this will be publishing the command velocity to the drone
    ros::Publisher tum_ardrone_pub;		     // this will be publishing the Target to the TUM package
    ros::Publisher plan_request_pub;
    geometry_msgs::Twist twist_manual, twist_pos;    // the message we use to send command to the drone
    
    void nav_callback(const tum_ardrone::filter_stateConstPtr tum_msg);
    void path_callback(const moveit_msgs::DisplayTrajectory msg);
    void lsd_pose_callback(const geometry_msgs::PoseStampedConstPtr pose_msg);
    void lsd_PC2_callback(const sensor_msgs::PointCloud2ConstPtr PC2_msg);
};


//-------Set up all members in the "ArdroneControl class"------
ArdroneControl::ArdroneControl()
{
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  nav_sub = n.subscribe("/ardrone/predictedPose", 50, &ArdroneControl::nav_callback, this);
  path_sub = n.subscribe("/move_group/display_planned_path", 1, &ArdroneControl::path_callback, this);
  lsd_pose_sub = n.subscribe("/lsd_slam/pose", 1, &ArdroneControl::lsd_pose_callback, this);
  lsd_PC2_sub = n.subscribe("/pointcloud2_BD", 1, &ArdroneControl::lsd_PC2_callback, this);
  tum_ardrone_pub = n.advertise<std_msgs::String>("tum_ardrone/com",50, true);
  plan_request_pub = n.advertise<moveit_msgs::MotionPlanRequest>("move_group/motion_plan_request",1, true); 

  twist_manual.linear.x = twist_manual.linear.y = twist_manual.linear.x = 0.0;
  Target[ptam_x] = Target[ptam_y] = Target[ptam_z] = Target[ptam_yaw] = 0.0;
  goal_x = goal_y = goal_yaw = 0.0;
  lsd_curr_pos[0] = lsd_curr_pos[1] = lsd_curr_pos[2] = lsd_curr_pos[3] = 0.0;
  manual_com[ptam_x] = manual_com[ptam_y] = manual_com[ptam_z] = manual_com[ptam_yaw] = 0.0;

  index2 = 0;
  path_follow_index = 0;
  replan_index = 0;
  joint_name.push_back(std::string("planar"));

  //robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  //robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  max_corner[0] = (ros::param::get("~max_corner_x", max_corner[0])) ? max_corner[0] : 10;
  max_corner[1] = (ros::param::get("~max_corner_y", max_corner[1])) ? max_corner[1] : 10;
  max_corner[2] = (ros::param::get("~max_corner_z", max_corner[2])) ? max_corner[2] : 10;
  min_corner[0] = (ros::param::get("~min_corner_x", min_corner[0])) ? min_corner[0] : -10;
  min_corner[1] = (ros::param::get("~min_corner_y", min_corner[1])) ? min_corner[1] : -10;
  min_corner[2] = (ros::param::get("~min_corner_z", min_corner[2])) ? min_corner[2] : -10;

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
    ss <<"c goto " << x << " " << y << " " << z << " "<< yaw;
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
		switch(com_char)
		{
			case 't':
				takeoff();
			break;

			case 'r':
				reset();
			break;

			case 'l':	
				land();
			break;
			
			case 'p':  //follow the path	
				clear_targets();				
				path_follow_index = 1;
			break;

            		case 'o': //back to the origine
				clear_targets();				
				targets2tum_ardrone(0.0, 0.0, 0.0 ,0.0);				
				ROS_INFO("back to the origine");		
			break;

            		case 'w': //forward
				clear_targets();
				manual_com[ptam_y] = manual_com[ptam_y] + 0.5;
				targets2tum_ardrone(manual_com[ptam_x],manual_com[ptam_y],manual_com[ptam_z],manual_com[ptam_yaw]);				
				ROS_INFO("move forward");		
			break;

            		case 's': //backward
				clear_targets();
				manual_com[ptam_y] = manual_com[ptam_y] - 0.5;
				targets2tum_ardrone(manual_com[ptam_x],manual_com[ptam_y],manual_com[ptam_z],manual_com[ptam_yaw]);				
				ROS_INFO("move backward");
			break;

            		case 'a': //left
				clear_targets();
				manual_com[ptam_x] = manual_com[ptam_x] - 0.5;
				targets2tum_ardrone(manual_com[ptam_x],manual_com[ptam_y],manual_com[ptam_z],manual_com[ptam_yaw]);				
				ROS_INFO("move left");
			break;

            		case 'd': //right
				clear_targets();
				manual_com[ptam_x] = manual_com[ptam_x] + 0.5;
				targets2tum_ardrone(manual_com[ptam_x],manual_com[ptam_y],manual_com[ptam_z],manual_com[ptam_yaw]);				
				ROS_INFO("move right");
			break;

            		case '+': //up
				clear_targets();
				manual_com[ptam_z] = manual_com[ptam_z] + 0.25;
				targets2tum_ardrone(manual_com[ptam_x],manual_com[ptam_y],manual_com[ptam_z],manual_com[ptam_yaw]);				
				ROS_INFO("move up");
			break;

            		case '-': //down
				clear_targets();
				manual_com[ptam_z] = manual_com[ptam_z] - 0.25;
				targets2tum_ardrone(manual_com[ptam_x],manual_com[ptam_y],manual_com[ptam_z],manual_com[ptam_yaw]);				
				ROS_INFO("move down");
			break;
 
           		case 'q': //Yaw left
				clear_targets();
				manual_com[ptam_yaw] = manual_com[ptam_yaw] - 10;
				targets2tum_ardrone(manual_com[ptam_x],manual_com[ptam_y],manual_com[ptam_z],manual_com[ptam_yaw]);				
				ROS_INFO("Yaw left");
			break;

            		case 'e': //Yaw right
				clear_targets();
				manual_com[ptam_yaw] = manual_com[ptam_yaw] + 10;
				targets2tum_ardrone(manual_com[ptam_x],manual_com[ptam_y],manual_com[ptam_z],manual_com[ptam_yaw]);				
				ROS_INFO("Yaw right");
			break;
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


//-----------Function for publishing the "plan_request"-------------
void ArdroneControl::Plan_request(float start_x, float start_y, float start_qz, float start_qw, float end_x, float end_y, float end_yaw)
{
	moveit_msgs::MotionPlanRequest Msg;
	Msg.workspace_parameters.min_corner.x = min_corner[0];
	Msg.workspace_parameters.min_corner.y = min_corner[1];
	Msg.workspace_parameters.min_corner.z = min_corner[2];
	Msg.workspace_parameters.max_corner.x = max_corner[0];
	Msg.workspace_parameters.max_corner.y = max_corner[1];	
	Msg.workspace_parameters.max_corner.z = max_corner[2];
	
	Msg.start_state.multi_dof_joint_state.header.frame_id = "/map";
	//Msg.start_state.multi_dof_joint_state.header.stamp = 0;
	//Msg.start_state.multi_dof_joint_state.header.seq = 0;		
	Msg.start_state.multi_dof_joint_state.joint_names = joint_name;

	Msg.start_state.multi_dof_joint_state.transforms.resize(1);
	Msg.start_state.multi_dof_joint_state.transforms[0].translation.x = 0;
	Msg.start_state.multi_dof_joint_state.transforms[0].translation.y = start_y;
	Msg.start_state.multi_dof_joint_state.transforms[0].translation.z = 0.0;
	Msg.start_state.multi_dof_joint_state.transforms[0].rotation.x = 0.0;
	Msg.start_state.multi_dof_joint_state.transforms[0].rotation.y = 0.0;
	Msg.start_state.multi_dof_joint_state.transforms[0].rotation.z = start_qz;	
	Msg.start_state.multi_dof_joint_state.transforms[0].rotation.w = start_qw;	
	Msg.start_state.is_diff = true;

	Msg.goal_constraints.resize(1);
	Msg.goal_constraints[0].joint_constraints.resize(3);
	Msg.goal_constraints[0].joint_constraints[0].joint_name = "planar/x";
	Msg.goal_constraints[0].joint_constraints[0].position = end_x;
	Msg.goal_constraints[0].joint_constraints[0].tolerance_above = 0.0001;
	Msg.goal_constraints[0].joint_constraints[0].tolerance_below = 0.0001;
	Msg.goal_constraints[0].joint_constraints[0].weight = 1.0;

	Msg.goal_constraints[0].joint_constraints[1].joint_name = "planar/y";
	Msg.goal_constraints[0].joint_constraints[1].position = end_y;
	Msg.goal_constraints[0].joint_constraints[1].tolerance_above = 0.0001;
	Msg.goal_constraints[0].joint_constraints[1].tolerance_below = 0.0001;
	Msg.goal_constraints[0].joint_constraints[1].weight = 1.0;

	Msg.goal_constraints[0].joint_constraints[2].joint_name = "planar/theta";
	Msg.goal_constraints[0].joint_constraints[2].position = end_yaw;
	Msg.goal_constraints[0].joint_constraints[2].tolerance_above = 0.0001;
	Msg.goal_constraints[0].joint_constraints[2].tolerance_below = 0.0001;
	Msg.goal_constraints[0].joint_constraints[2].weight = 1.0; 

	Msg.group_name = "altHold";
	Msg.num_planning_attempts = 1;
	Msg.allowed_planning_time = 1.0;

	//Msg.workspace_parameters.header.seq = 0;
	Msg.workspace_parameters.header.stamp =  ros::Time::now();
	Msg.workspace_parameters.header.frame_id = "/map";
	plan_request_pub.publish(Msg);
}


//-----------Function for subscribing the pose from lsd-slam-----------
void ArdroneControl::lsd_pose_callback(const geometry_msgs::PoseStampedConstPtr pose_msg)
{
	lsd_curr_pos[0] = pose_msg->pose.position.x;
	lsd_curr_pos[1] = pose_msg->pose.position.y;
	lsd_curr_pos[2] = pose_msg->pose.orientation.z;
	lsd_curr_pos[3] = pose_msg->pose.orientation.w;
}


//----------Function for detecting the PointCloud2 Topic---------------
void ArdroneControl::lsd_PC2_callback(const sensor_msgs::PointCloud2ConstPtr PC2_msg)
{
	ROS_INFO("Point Cloud2 BD was updated!!");	
	if(replan_index == 1)
	{
		ROS_INFO("START Replanning!!");		
		Plan_request(lsd_curr_pos[0], lsd_curr_pos[1], lsd_curr_pos[2], lsd_curr_pos[3], goal_x, goal_y, goal_yaw);
		ROS_INFO("Replanning!!");
	}
		
}

//-----------Function for subscribing the path from MoveIt!------------
void ArdroneControl::path_callback(const moveit_msgs::DisplayTrajectory msg)
{
    //---Copy the path data to pointers----
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
	path_x[i] = msg.trajectory[0].multi_dof_joint_trajectory.points[i].transforms[0].translation.x;
	path_y[i] = msg.trajectory[0].multi_dof_joint_trajectory.points[i].transforms[0].translation.y;	
	path_z[i] = msg.trajectory[0].multi_dof_joint_trajectory.points[i].transforms[0].translation.z;
	       qz = msg.trajectory[0].multi_dof_joint_trajectory.points[i].transforms[0].rotation.z;
	       qw = msg.trajectory[0].multi_dof_joint_trajectory.points[i].transforms[0].rotation.w;
        path_w[i] = Quaternion2Yaw(qz, qw);
    }
    ROS_INFO("Get the Path, the total size of points is: %d",point_size);

    //---Sned commands to the ARDrone---
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

    //---Update requirements of the goal---
    goal_x = -path_x[point_size-1];
    goal_y = -path_y[point_size-1];
    goal_yaw = path_w[point_size-1]*PI/180.0;
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
		clear_targets();
           	path_follow_index = 0;	
		index2 = 0;
	}
	if( com_char != 'x')			
    		Manual(com_char);
    }  
    if(manual_selection == 'p')
    {
	ROS_INFO("Path Mode--> s: start following the path, r: replanning, p: pause   (x for \"STOP\" ):");
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
        if( com_char == 'r')
	{	
		path_follow_index = 1;		
		replan_index = 1;		
		ROS_INFO("START");
	}
	if( com_char == 'p')
	{
		replan_index = 0;		
		path_follow_index = 0;
		ROS_INFO("Pause");
	}
    }
}


//------------Main function-----------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tum_position_3");
  ROS_INFO("START");
  ROS_INFO("Wait for connecting the ARDrone!!");  
  ArdroneControl ardrone_control;

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
