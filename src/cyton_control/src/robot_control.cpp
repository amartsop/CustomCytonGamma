#include "../include/robot_control.h"

RobotControl::RobotControl(robot_model::RobotModelPtr kinematic_model,
    TrajClient& traj_client)
{
    // Kinematic state handle
    m_kinematic_state = new robot_state::RobotState(kinematic_model);

    // Create joint model group
    m_joint_model_group = kinematic_model->getJointModelGroup("cyton_arm");

    // Set joint and link names 
    m_joint_names = m_joint_model_group->getActiveJointModelNames();
    m_link_names = m_joint_model_group->getLinkModelNames();

    // Initialize robot
    m_init_path.time = m_t_init;
    m_init_path.position = m_init_pos;
    m_init_path.velocity = m_init_vel;
    m_init_path.acceleration = m_init_accel;

    goto_joint_position(m_init_path, traj_client);

    // Get end effector name
    m_ee_name = m_link_names.back();
}

// Go to cartesian position (multiple points)
void RobotControl::goto_cartesian_position(const std::vector<cmsgs::Cartesian>&
    cartesian_state, TrajClient& traj_client)
{
    // Get end effector link id
    auto ee_link = m_kinematic_state->getLinkModel(m_ee_name);

    // Valid solution
    bool valid_solution = true;

    // Create joint object 
    std::vector<cmsgs::Joint> joint_angles(cartesian_state.size());

    // Generate joint trajectory
    for (size_t i = 0; i < cartesian_state.size(); i++)
    {
        // Geometry message
        geometry_msgs::Pose cart_pose;

        // Set position
        cart_pose.position.x = cartesian_state[i].position[0];
        cart_pose.position.y = cartesian_state[i].position[1];
        cart_pose.position.z = cartesian_state[i].position[2];

        // Set orientation
        double phi = cartesian_state[i].euler_position[0];
        double theta = cartesian_state[i].euler_position[1];
        double psi = cartesian_state[i].euler_position[2];

        EulerRotations::Quaternions quatern =
            EulerRotations::euler_to_quaternions(phi, theta, psi);

        cart_pose.orientation.w = quatern.w; cart_pose.orientation.x = quatern.x;
        cart_pose.orientation.y = quatern.y; cart_pose.orientation.z = quatern.z;

        // Joint values in Joint format
        joint_angles[i].time = cartesian_state[i].time;

        // Solve inerse kinematics
        bool found_ik = m_kinematic_state->setFromIK(m_joint_model_group,
            cart_pose, m_ee_name);

        // Now, we can print out the IK solution (if found):
        if (found_ik)
        {
            /******************* Set joint positions ********************/
            m_kinematic_state->copyJointGroupPositions(m_joint_model_group,
                joint_angles[i].position);

            /******************* Initialize joint velocities (zero) ********************/
            // Resize velocity vector
            joint_angles[i].velocity.resize(joint_angles[i].position.size());
            
            /******************* Initilize joint acceleration (zero) ********************/
            // Resize acceleration vector
            joint_angles[i].acceleration.resize(joint_angles[i].position.size());
        }
        else
        {   
            valid_solution = false;
            break;
        }

        // Print current state
        ROS_INFO("%d out of: %d", (unsigned int) i, (unsigned int) cartesian_state.size());
    }

    if(valid_solution)
    {
        ROS_INFO("Path found! Generating position...");

        goto_joint_position(joint_angles, traj_client);
    }
    else
    {
        ROS_INFO("Inverse kinematics failed!");
    }
}

// Go to cartesian position (one point)
void RobotControl::goto_cartesian_position(const cmsgs::Cartesian&
    cartesian_state, TrajClient& traj_client)
{
    // Get end effector link id
    auto ee_link = m_kinematic_state->getLinkModel(m_ee_name);

    // Geometry message
    geometry_msgs::Pose cart_pose;

    // Set position
    cart_pose.position.x = cartesian_state.position[0];
    cart_pose.position.y = cartesian_state.position[1];
    cart_pose.position.z = cartesian_state.position[2];

    // Set orientation
    EulerRotations::Quaternions quatern =
        EulerRotations::euler_to_quaternions(cartesian_state.euler_position[0], 
            cartesian_state.euler_position[1], cartesian_state.euler_position[2]);

    cart_pose.orientation.w = quatern.w;
    cart_pose.orientation.x = quatern.x;
    cart_pose.orientation.y = quatern.y;
    cart_pose.orientation.z = quatern.z;

    // Joint values in Joint format
    cmsgs::Joint joints_values;
    joints_values.time = cartesian_state.time;
    
    // Solve inverse kinematics
    bool found_ik = m_kinematic_state->setFromIK(m_joint_model_group,
        cart_pose, m_ee_name);


    // Now, we can print out the IK solution (if found):
    if (found_ik)
    {
        /******************* Set joint positions ********************/
        m_kinematic_state->copyJointGroupPositions(m_joint_model_group,
            joints_values.position);
        
        /******************* Initilize joint velocities (zero) ********************/
        // Resize velocity vector
        joints_values.velocity.resize(joints_values.position.size());

        /******************* Initialize joint accelerations (zero) ***************/
        joints_values.acceleration.resize(joints_values.position.size());

        // Go to joint position
        goto_joint_position(joints_values, traj_client);
    }
    else
    {
        ROS_INFO("Did not find IK solution");
    }
}


// Go to joint position (one point)
void RobotControl::goto_joint_position(const cmsgs::Joint& joint_state,
    TrajClient& traj_client)
{
    // Create goal 
    fktg goal = set_joint_state_goal(joint_state);

    // Send goal to server
    traj_client.sendGoal(goal);
}


// Go to joint positions (multiple points)
void RobotControl::goto_joint_position(const std::vector<cmsgs::Joint>&
    joint_state, TrajClient& traj_client)
{
    // Create goal 
    fktg goal = set_joint_state_goal(joint_state);

    // Send goal to server
    traj_client.sendGoal(goal);
}


// Get end effector position
std::vector<double> RobotControl::get_ee_position(void)
{
    const Eigen::Affine3d &end_effector_state =
        m_kinematic_state->getGlobalLinkTransform(m_ee_name);

    Eigen::Vector3d pos = end_effector_state.translation();

    // Position
    std::vector<double> pos_vec(3);
    pos_vec.at(0) = pos(0); pos_vec.at(1) = pos(1); pos_vec.at(2) = pos(2);
    
    // Return position
    return pos_vec;
}


// Get end effector orientation, euler angles phi, theta, psi (z, y', x'')
std::vector<double> RobotControl::get_ee_orientation(void)
{
    const Eigen::Affine3d &end_effector_state =
        m_kinematic_state->getGlobalLinkTransform(m_ee_name);

    Eigen::Matrix3d rot = end_effector_state.rotation();
    Eigen::Quaterniond quatern(rot);

    // Quaternions to euler angles
    EulerRotations::Euler eul =
        EulerRotations::quaternions_to_euler(quatern.w(), quatern.x(),
        quatern.y(), quatern.z());

    std::vector<double> eul_vec(3);
    eul_vec.at(0) = eul.phi; eul_vec.at(1) = eul.theta; eul_vec.at(2) = eul.psi;

    // Return orientation
    return eul_vec;
}


// Get end-effector pose
cmsgs::Cartesian RobotControl::get_ee_pose(void)
{
    // Get end-effector position
    std::vector<double> lin_pos = get_ee_position();

    // Get end-effector orientation
    std::vector<double> euler = get_ee_orientation();

    // Cartesian pose 
    cmsgs::Cartesian pose;

    // Set position
    pose.position = lin_pos;

    // Set orientation
    pose.euler_position = euler;

    // DEBUG -> Should find a way of changing velocities and acceleration
    return pose;
}

// Get joint angles values
std::vector<double> RobotControl::get_joint_angles(void)
{
    std::vector<double> joint_values;
    m_kinematic_state->copyJointGroupPositions(m_joint_model_group, joint_values);

    return joint_values;
}


// Set joint state goal (one point)
fktg RobotControl::set_joint_state_goal(const cmsgs::Joint& joint_state)
{
    // Create goal object 
    fktg goal;

    // Robot Initialization routine
    goal.trajectory.joint_names = m_joint_names;

    goal.trajectory.points.resize(1);

    // Set position
    goal.trajectory.points[0].positions =
        joint_calibration (joint_state.position);
    m_kinematic_state->setJointGroupPositions(m_joint_model_group,
        joint_state.position);

    // Set velocity
    goal.trajectory.points[0].velocities = joint_state.velocity;
    m_kinematic_state->setJointGroupVelocities(m_joint_model_group,
        joint_state.velocity);

    // Set acceleration
    goal.trajectory.points[0].accelerations = joint_state.acceleration;
    m_kinematic_state->setJointGroupAccelerations(m_joint_model_group,
        joint_state.acceleration);

    // Set time from start
    goal.trajectory.points[0].time_from_start = ros::Duration(joint_state.time);

    return goal;
}


// Set joint state goal (multiple points)
fktg RobotControl::set_joint_state_goal(const std::vector<cmsgs::Joint>&
    joint_state)
{
    // Create goal object 
    fktg goal;

    // Resize goal number
    goal.trajectory.points.resize(joint_state.size());

    // Set joint names
    goal.trajectory.joint_names = m_joint_names;

    for(size_t i = 0; i < joint_state.size(); i++)
    {
        // Set position
        goal.trajectory.points[i].positions = joint_calibration(
            joint_state.at(i).position);

        m_kinematic_state->setJointGroupPositions(m_joint_model_group,
            joint_calibration(joint_state.at(i).position));
        
        // Set velocity
        goal.trajectory.points[i].velocities = joint_state.at(i).velocity;

        m_kinematic_state->setJointGroupVelocities(m_joint_model_group,
            joint_state.at(i).velocity);

        // Set acceleartion
        goal.trajectory.points[i].accelerations = joint_state.at(i).acceleration;

        m_kinematic_state->setJointGroupAccelerations(m_joint_model_group,
            joint_state.at(i).acceleration);
        
        // Set time from start
        goal.trajectory.points[i].time_from_start =
            ros::Duration(joint_state.at(i).time);
    }

    return goal;
}

// Go to initial position 
void RobotControl::goto_initial_position(TrajClient& traj_client)
{
    goto_joint_position(m_init_path, traj_client);
}

// Joint calibration (this is specific to robai cyton brl)
std::vector<double> RobotControl::joint_calibration(const std::vector<double>
    joint_angles)
{
    // // Calibrated joint angles
    // std::vector<double> joint_angles_cal = joint_angles;

    // // Joint 1 offset
    // double offset_joint_1 = - 3.5 * M_PI / 180.0;
    // joint_angles_cal.at(1) += offset_joint_1;

    // // Joint 2 offset
    // double offset_joint_2 = 3.0 * M_PI / 180.0;
    // joint_angles_cal.at(2) += offset_joint_2;

    // // Joint 3 offset
    // double x = joint_angles_cal.at(3);

    // // joint_angles_cal.at(3) = 0.394328 * pow(x, 3.0) -
    // //     0.442034 * pow(x, 2.0) + 0.494424 * x + 0.029063;

    // joint_angles_cal.at(3) = 0.614131 * pow(x, 3.0) -
    //     2.339247 * pow(x, 2.0) + 3.301271 * x - 0.116082;

    // return joint_angles_cal;
    return joint_angles;
}

// Convert an std::vector<double> to Eigen::VectorXd
Eigen::VectorXd RobotControl::vec_to_eigen(const std::vector<double>& a)
{
    return Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(a.data(), a.size());
}

// Unwraps angle x (due to discontinuity of atan function)
// x is the current value of angle
// y is the previous value of the angle
double RobotControl::unwrapping(double x, double y, double tol)
{
   while ( (x - y + M_PI) <= tol ) 
   {
       x = x + 2.0 * M_PI;
   }

   while ((x - y - M_PI) >= tol )
   {
       x = x - 2.0 * M_PI;
   }

   return x;
}

// Destructor
RobotControl::~RobotControl()
{
    delete m_kinematic_state;
}