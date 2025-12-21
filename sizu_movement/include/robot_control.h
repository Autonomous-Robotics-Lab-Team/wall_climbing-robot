#ifndef robot_control_H
#define robot_control_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>
#include <urdf/model.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <map>
#include <vector>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <string>
#include <JetsonGPIO.h>
// #include <casadi/casadi.hpp>


class robot_control
{
public:
    robot_control();
 

    ros::NodeHandle nh_;

    // void createDefaultPoses();
    void initJointStatePublishers();
    Eigen::Matrix<double,3,1> Position_to_Angle(Eigen::Matrix<double,3,1> position);
    void inverseKinematics(Eigen::Matrix<double,3,4> Feet_position);
    Eigen::Matrix<double,3,4> init_Feet_position;
    void commandSend(Eigen::Matrix<double,3,4> All_Feet_position, std::vector<int> leg_sign);
    void stand();
    void get_parameters(double T, double dt);
    Eigen::Matrix3d computeJacobian(double theta1, double theta2, double theta3, double l1, double l2, double l3);

    double dt;
    double T;


    // 发布者
    ros::Publisher pub_motor_commands_;
    ros::Publisher pub_trajectory_;
    sensor_msgs::JointState motor_commands;
    Eigen::Matrix<double,3,4> last_positions_;
    
    // 关节状态
    trajectory_msgs::JointTrajectory traj_msg;
    trajectory_msgs::JointTrajectoryPoint point;
    
    Eigen::Matrix<double,3,4> home_pose_;
    Eigen::Matrix<double,3,4> current_Feet_angle_;
    Eigen::Matrix3d J;//雅可比矩阵
    std::vector<double> leg_length_;
    double box_length;
    double box_width;
    double leg_xy;//初始位姿腿在水平面的投影长度
};





#endif