#include <iostream>

#include "ros/ros.h"

#include <actionlib/client/simple_action_client.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include "std_msgs/Bool.h"

#include "../include/robot_control.h"
#include "../include/path_planning.h"

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    TrajClient;

int main(int argc, char** argv)
{
    // Initiate sync_test node 
    ros::init(argc, argv, "cyton_control"); 

    // Create ros node handle
    ros::NodeHandle node_handle;

    // Initiate asychronous callback check
    ros::AsyncSpinner spinner(0);
    spinner.start();

    /***************************** Robot control **************************/
    // Create follow joint trajectroy action
    std::string traj_client_name = "arm_controller/follow_joint_trajectory";
    TrajClient traj_client(traj_client_name, true);

    // Wait to connect to the server
    while(!traj_client.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the joint_trajectory_action server");
    }

    // Create kinematic robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    // Initialize robot
    RobotControl robot_control(kinematic_model, traj_client);

    /************************* Execute trajectory **************************/
    
    // Generate cartesian path (wrt current robot's pose)
    PathPlanning path_plan;
    auto cart_path = path_plan.generate_trajectory(robot_control.get_ee_pose());

    // Execute trajectory
    robot_control.goto_cartesian_position(cart_path, traj_client);

    /************************* Main loop **************************/
    while(ros::ok())
    {
        /********************* Shutdown routine ************************/
        if(traj_client.getState().isDone()) {break;}

        /***************************************************************/

        ros::spinOnce();
    }

    robot_control.goto_initial_position(traj_client);

    // Shutdown node
    ros::shutdown();

    return 0;
}