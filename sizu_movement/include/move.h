#ifndef MOVE_H
#define MOVE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>
#include <urdf/model.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
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


class move {
public:
    move();
    ~move();
    void mainLoop();

    ros::NodeHandle nh_;

    // void createDefaultPoses();
    void initJointStatePublishers();
    void keyboardListenerThread();
    void commandPosition(std::vector<double> xyz, std::vector<int> sign);
    void runWaveMode(std::vector<double> xyz, int sign);
    void runRotationMode(std::vector<double> xyz, int sign);
    void RlMoveMode(std::vector<double> xyz, int sign);
    std::vector<double> get_Rotation_xyz(double angle);
    std::vector<double> inverseKinematics(std::vector<double> xyz, int sign);
    std::vector<double> get_poses(double x,double y,double z);

    
    ros::Rate rate_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;


    // 发布者
    ros::Publisher pub_joint_state_;
    ros::Publisher pub_motor_commands_;
    ros::Publisher pub_trajectory_;
    sensor_msgs::JointState motor_commands;
    std::map<std::string, ros::Publisher> controller_publishers_;
    std::vector<double> last_positions_;
    std::vector<std::string> controller_joint_names_;

    // 关节状态
    sensor_msgs::JointState message_joint_state_;
    trajectory_msgs::JointTrajectory traj_msg;
    trajectory_msgs::JointTrajectoryPoint point;
    
    
    
    std::vector<double> dir_;
    std::vector<double> home_pose_;
    std::vector<double> movement_pose_;
    std::vector<double> current_position_;
    std::vector<double> leg_length_;
    std::thread keyboard_thread_;
    struct termios oldt_, newt_;
    double leg_long_;
    double rate_time;
    double box_length;
    double box_width;
    double stride_length;
};

#endif // MOVE_H