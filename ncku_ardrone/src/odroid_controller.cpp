#include "ros/ros.h"
#include "ros/service_callback_helper.h"
#include "std_srvs/Empty.h"
#include <string>
#include <sstream>
 
//Kbhit() and Keyboard()
#include <termios.h>                   
#include <stdio.h>   
#include <unistd.h>  
#include <fcntl.h> 
 
#include <moveit_msgs/DisplayTrajectory.h> 
#include <math.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>
 
//MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>


#define msf_x      0   // Current x-position from ptam
#define msf_y      1   // Unit of position is 'm'
#define msf_qz	   2
#define msf_qw     3   // Unit of yaw is degree, range is from -180 ~ 180
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
    void Free_pointers();
    void Plan_request(float end_x, float end_y, float end_yaw);
    float Quaternion2Yaw(float qz, float qw); 
    bool index2;
    bool path_follow_index;
    float *path_x;
    float *path_y; 
    float *path_z;
    float *path_w;
    float goal_x;
    float goal_y;
    float goal_yaw;
    float msf_curr_pos[4];
    bool replan_index;
    double time_curr;
    double time_last;

    std::vector<std::string> joint_name;
    
    char manual_selection;

    ros::NodeHandle n;                               // so that we dont make a copy while passing this as reference
    ros::Subscriber msf_pose_sub;
    ros::Subscriber lsd_PC2_sub;
    ros::Subscriber path_sub;			     // subscribing to the display_planned_path from Moveit topic
    ros::Publisher display_pub;
    
    void path_callback(const moveit_msgs::DisplayTrajectory msg);
    void msf_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr pose_msg);
    void lsd_PC2_callback(const sensor_msgs::PointCloud2ConstPtr PC2_msg);
};


//-------Set up all members in the "ArdroneControl class"------
ArdroneControl::ArdroneControl()
{
  path_sub = n.subscribe("/move_group/display_planned_path", 1, &ArdroneControl::path_callback, this);
  msf_pose_sub = n.subscribe("/msf_core/pose", 1, &ArdroneControl::msf_pose_callback, this);
  lsd_PC2_sub = n.subscribe("/pointcloud2_scaled", 1, &ArdroneControl::lsd_PC2_callback, this);
  display_pub = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  goal_x = goal_y = goal_yaw = 0.0;
  msf_curr_pos[msf_x] = msf_curr_pos[msf_y] = msf_curr_pos[msf_qz] = msf_curr_pos[msf_qw] = 0.0;
  index2 = 0;
  path_follow_index = 1;
  replan_index = 0;

  path_x = (float*)malloc(20 * sizeof(float));
  path_y = (float*)malloc(20 * sizeof(float));
  path_z = (float*)malloc(20 * sizeof(float));
  path_w = (float*)malloc(20 * sizeof(float));

  time_curr = ros::Time::now().toSec();
  time_last = time_curr;
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
void ArdroneControl::Plan_request(float end_x, float end_y, float end_yaw)  // meter and radian
{	
	//---Set up planning environment	
	moveit::planning_interface::MoveGroup group("altHold");
	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

	std::vector<double> group_variable_values;
	moveit::planning_interface::MoveGroup::Plan my_plan;

	group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

	group_variable_values[0] = end_x;
	group_variable_values[1] = end_y;
	group_variable_values[2] = end_yaw;
	group.setJointValueTarget(group_variable_values);

	group.setPlanningTime(2.5);
	
	//---Checking collision---
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	planning_scene::PlanningScene planning_scene(kinematic_model);

	collision_detection::CollisionRequest collision_request;
	collision_detection::CollisionResult collision_result;
	collision_request.group_name = "altHold";
	planning_scene.checkCollision(collision_request, collision_result);

	//---Time calculation---
	time_curr = ros::Time::now().toSec();

	if(collision_result.collision == false)
	{
		ROS_INFO("Current state is not in collision");		
		if(time_curr - time_last >= 5.0)  //The fastest updated frequency of planning is 0.2Hz		
		{
			group.asyncMove();  //Send the planning request to MoveIt without cheching execution 
			time_last = ros::Time::now().toSec();					
		}
		else
			ROS_INFO("Too fast to update the path");
	}		
	else
		ROS_INFO("Current state is in collision, stop planning !!");
}


//----------Function for detecting the PointCloud2 Topic---------------
void ArdroneControl::lsd_PC2_callback(const sensor_msgs::PointCloud2ConstPtr PC2_msg)
{
	//ROS_INFO("PointCloud2-scaled was updated!!");	
	if(replan_index == 1)
	{
		ROS_INFO("START Replanning!!");		
		Plan_request(goal_x, goal_y, goal_yaw);
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
	path_x[i] = -msg.trajectory[0].multi_dof_joint_trajectory.points[i].transforms[0].translation.x;
	path_y[i] = -msg.trajectory[0].multi_dof_joint_trajectory.points[i].transforms[0].translation.y;	
	path_z[i] = msg.trajectory[0].multi_dof_joint_trajectory.points[i].transforms[0].translation.z;
	       qz = msg.trajectory[0].multi_dof_joint_trajectory.points[i].transforms[0].rotation.z;
	       qw = msg.trajectory[0].multi_dof_joint_trajectory.points[i].transforms[0].rotation.w;
        path_w[i] = Quaternion2Yaw(qz, qw);  //degree
    }
    ROS_INFO("Get the Path, the total size of points is: %d",point_size);

    //---Sned commands to the ARDrone---
    if(path_follow_index == 1)
    {
	i = 0;	
	i_next = 1;
	
	ROS_INFO("Start to follow the path!!");
	while( i_next < point_size )
        {
	    distance = sqrtf( (path_x[i_next]-path_x[i])*(path_x[i_next]-path_x[i]) + (path_y[i_next]-path_y[i])*(path_y[i_next]-path_y[i]) + (path_z[i_next]-path_z[i])*(path_z[i_next]-path_z[i]) );
	    if(distance <= 0.2)
		i_next++;
	    if(distance > 0.2)
	    {
		ROS_INFO("x:%f, y:%f, z:%f, yaw:%f",path_x[i_next],path_y[i_next],path_z[i_next],path_w[i_next]);		
		i = i_next;		
		i_next++;
	    }
        }
	ROS_INFO("All of targets were published!!");
    }

    //---Update requirements of the goal---
    goal_x = -path_x[point_size-1];
    goal_y = -path_y[point_size-1];
    goal_yaw = path_w[point_size-1]*PI/180.0;  //radian
}


//-----------Function for subscribing the pose from msf_update-----------
void ArdroneControl::msf_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr pose_msg)
{
	msf_curr_pos[msf_x] = pose_msg->pose.pose.position.x;
	msf_curr_pos[msf_y] = pose_msg->pose.pose.position.y;
	msf_curr_pos[msf_qz] = pose_msg->pose.pose.orientation.z;
	msf_curr_pos[msf_qw] = pose_msg->pose.pose.orientation.w;
  	
	
	if(kbhit())
	{
		manual_selection = getchar();

		if( index2 == 0 || manual_selection == 't')
		{ 
			ROS_INFO("Please enter your target (x y z yaw) like-->0.5 -0.5 0 180 :");
			float parameters[5];
			char tar_command[15];
			gets(tar_command);
			if(sscanf(tar_command,"%f %f %f %f",&parameters[0], &parameters[1], &parameters[2], &parameters[3]) == 4)
			{
				parameters[3] = parameters[3]*PI/180.0; //degree to radian				
				index2 = 1;			
				Plan_request(parameters[0],parameters[1],parameters[3]);				
				ROS_INFO("Set up the Target and start path planning!!");
			} 
			manual_selection == 'x';  
		}  


		if(manual_selection == 'p')
		{	
			path_follow_index = 1;
			ROS_INFO("Send the path to PIXHAWK"); 
			manual_selection == 'x';      		
		}
		if(manual_selection == 'r')
		{
			replan_index = 1;		
			ROS_INFO("Replan = True"); 
			manual_selection == 'x'; 
		}   
		if(manual_selection == 's')
		{
			replan_index = 0;		
			ROS_INFO("Replan = False"); 
			manual_selection == 'x'; 
		}    
		ROS_INFO("Mode Selection 't'->Specify the target , 'p'->path follow, 'r'->replan, 's'->stop replan:");
	}	
}


//------------Main function-----------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "odroid_controller");
  ROS_INFO("START");
  ROS_INFO("Wait for receiving data from msf_core!!");  
  ArdroneControl ardrone_control;

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
