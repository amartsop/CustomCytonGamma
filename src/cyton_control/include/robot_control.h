#pragma once

#include <iostream>
#include <vector>
#include <math.h>

#include "ros/ros.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/client/simple_action_client.h"
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>

#include "cmsgs.h"
#include "euler_rotations.h"

#include "std_msgs/Bool.h"
#include "control_msgs/JointTrajectoryControllerState.h"

typedef control_msgs::FollowJointTrajectoryGoal fktg;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    TrajClient;


class RobotControl
{
    public:
        RobotControl(robot_model::RobotModelPtr kinematic_model,
            TrajClient& traj_client);
        ~RobotControl();

    public:
        // Go to cartesian position (one point)
        void goto_cartesian_position(const cmsgs::Cartesian& cartesian_state,
            TrajClient& traj_client);

        // Go to cartesian position (multiple points)
        void goto_cartesian_position(const std::vector<cmsgs::Cartesian>&
            cartesian_state, TrajClient& traj_client);

    public:
        // Go to joint position (one point)
        void goto_joint_position(const cmsgs::Joint& joint_state,
            TrajClient& traj_client);

        // Go to joint positions (multiple points)
        void goto_joint_position(const std::vector<cmsgs::Joint>& joint_state,
            TrajClient& traj_client);
        
    public:
        // Get end effector position (m)
        std::vector<double> get_ee_position(void);

        // Get end effector orientation, euler angles phi, theta, psi (z, y', x'')
        std::vector<double> get_ee_orientation(void);

        // Get end-effector pose
        cmsgs::Cartesian get_ee_pose(void);

        // Get joint angles values
        std::vector<double> get_joint_angles(void);

        // End effector name
        std::string m_ee_name;

    public:

        // Go to initial position 
        void goto_initial_position(TrajClient& traj_client);

    private:
        // Joint names
        std::vector<std::string> m_joint_names;

        // Link names
        std::vector<std::string> m_link_names;

        // Kinematic state ptr handle 
        robot_state::RobotState* m_kinematic_state;

        // Group joint model
        robot_state::JointModelGroup* m_joint_model_group;

    private:
        
        const std::vector<double> m_init_pos = {
            0.0 * M_PI / 180.0,
            60.0 * M_PI / 180.0, 
            0.0 * M_PI / 180.0, 
            -60.0 * M_PI / 180.0,
            90.0 * M_PI / 180.0,
            90.0 * M_PI / 180.0};

        std::vector<double> m_init_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        std::vector<double> m_init_accel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        double m_t_init = 3.0;

        // Initial path
        cmsgs::Joint m_init_path;

    private:
        // Set joint state goal (one point)
        fktg set_joint_state_goal(const cmsgs::Joint& joint_state);

        // Set joint state goal (multiple points)
        fktg set_joint_state_goal(const std::vector<cmsgs::Joint>& joint_state);
    
    private:
        // Convert an std::vector<double> to Eigen::VectorXd
        Eigen::VectorXd vec_to_eigen(const std::vector<double>& a);

        // Unwraps angle x (due to discontinuity of atan function)
        // x is the current value of angle
        // y is the previous value of the angle
        double unwrapping(double x, double y, double tol=1.0e-4);

        // Joint calibration (this is specific to robai cyton brl)
        std::vector<double> joint_calibration(const std::vector<double>
            joint_angles);
};
