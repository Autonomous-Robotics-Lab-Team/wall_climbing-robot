#ifndef EstimateParameters_H
#define EstimateParameters_H

#include <ros/ros.h>
#include <string>
#include <math.h>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include "robot_control.h"
#include <thread>
#include <map>
#include <mutex>
#include <ros/timer.h>

class EstimateParameters {
public:
    EstimateParameters();
    ~EstimateParameters();
        

        Eigen::Matrix<double,3,1> body_position_world;//世界坐标系下的机身位置下x,y,z
        Eigen::Matrix<double,3,1> body_velocity_world;//世界坐标系下的机身速度vx,vy,vz
        Eigen::Matrix<double,3,1> body_acceleration_world;//世界坐标系下的机身加速度ax,ay,az
        Eigen::Matrix<double,3,1> body_angular_velocity;//世界坐标系下的机身角速度wx,wy,wz
        Eigen::Matrix<double,3,1> body_angular;//机身坐标系下的机身旋转角度
        Eigen::Matrix<double,3,4> All_Feet_position;//所有足端在世界坐标系下的位置x,y,z
        Eigen::Matrix<double,3,4> All_Feet_angle;//所有足端在世界坐标系下的角度vx,vy,vz

        Eigen::Matrix<double,3,1> FL_position_world;//世界坐标系下的FL位置x,y,z
        Eigen::Matrix<double,3,1> FL_position_body;//机身坐标系下的FL位置x,y,z
        Eigen::Matrix<double,3,1> FL_velocity_world;//世界坐标系下的FL速度vx,vy,vz
        Eigen::Matrix<double,3,1> FL_velocity_body;//机身坐标系下的FL速度vx,vy,vz
        Eigen::Matrix<double,3,1> FL_angle_body;;//FL关节角度

        Eigen::Matrix<double,3,1> RL_position_world;//世界坐标系下的RL位置x,y,z
        Eigen::Matrix<double,3,1> RL_position_body;//机身坐标系下的RL位置x,y,z
        Eigen::Matrix<double,3,1> RL_velocity_world;//世界坐标系下的RL速度vx,vy,vz
        Eigen::Matrix<double,3,1> RL_velocity_body;//机身坐标系下的RL速度vx,vy,vz
        Eigen::Matrix<double,3,1> RL_angle_body;;//RL关节角度

        Eigen::Matrix<double,3,1> FR_position_world;//世界坐标系下的FR位置x,y,z
        Eigen::Matrix<double,3,1> FR_position_body;//机身坐标系下的FR位置x,y,z
        Eigen::Matrix<double,3,1> FR_velocity_world;//世界坐标系下的FR速度vx,vy,vz  
        Eigen::Matrix<double,3,1> FR_velocity_body;//机身坐标系下的FR速度vx,vy,vz
        Eigen::Matrix<double,3,1> FR_angle_body;;//FR关节角度

        Eigen::Matrix<double,3,1> RR_position_world;//世界坐标系下的RR位置x,y,z
        Eigen::Matrix<double,3,1> RR_position_body;//机身坐标系下的RR位置x,y,z
        Eigen::Matrix<double,3,1> RR_velocity_world;//世界坐标系下的RR速度vx,vy,vz
        Eigen::Matrix<double,3,1> RR_velocity_body;//机身坐标系下的RR速度vx,vy,vz
        Eigen::Matrix<double,3,1> RR_angle_body;;//RR关节角度

        Eigen::Vector3d eulerAngle;//欧拉角：[roll (x), pitch (y), yaw (z)]
        Eigen::Quaterniond quaternion;;//世界坐标系到机身坐标系的四元数
        Eigen::Matrix3d R_world_to_body;//世界坐标系到机身坐标系的旋转矩阵

        double dt;//时间间隔
        double T; //步态周期
        double loop_dt_;// 主循环积分时间步长
        double compensation_velocity;//速度补偿

        //机身参数
        double high_hop;//抬腿高度
        std::vector<double> leg_length_;;//各段腿长
        Eigen::Matrix<double,3,4> home_pose_; //初始站立姿态关节角度
        Eigen::Matrix<double,3,4> init_position;//初始站立姿态足端位置


        void CallBack(const geometry_msgs::Twist::ConstPtr& msg);
        void ImuCb(const sensor_msgs::Imu::ConstPtr& msg);
        void GetWorldPosition();
        // void GetVelocity();
        void Get_R_world_to_body();
        int GetFeetMovePos();
        int GetFeetRotatePos();
        void GetFeetVel();
        void Odom_init();
        void get_parameters(double T, double dt, double high_hop);
        void keyboardListenerThread();
        void mainloop();
        void UpdateTimerCb(const ros::TimerEvent& event);
        void disableCanonicalMode();  // 禁用规范模式（用于键盘控制）

        

        ros::NodeHandle nh;
        ros::Subscriber cmd_vel_sub;
        ros::Subscriber imu_sub;
        ros::Publisher parameter_pub;
        ros::Publisher odom_pub;
    
        geometry_msgs::TransformStamped odom_TF;
        nav_msgs::Odometry odom;
        tf::TransformBroadcaster odom_broadcaster;
        std::string odom_frame_id_ = "odom";
        std::string base_frame_id_ = "base_link";
        ros::Time _currentTime;
        ros::Time _lastTime;
        robot_control move;
        std::vector<int> leg_sign;

        // 键盘优先相关
        std::thread keyboard_thread_;
        struct termios oldt_, newt_;     
        bool keyboard_active_;         // 是否正在用键盘控制
        ros::Time last_key_time_;      // 最近一次按键时间
        double key_timeout_;           // 键盘多久不按视为无输入
        char last_key_;                // 上一次按下的键
        std::map<char, bool> key_pressed_; // 按键状态追踪

        // 并发保护与步态时间
        std::mutex state_mutex_;
        double gait_time_move_ = 0.0;
        double gait_time_rotate_ = 0.0;
        Eigen::Vector3d active_rotate_vel_ = Eigen::Vector3d::Zero();

        // 日志与滤波
        Eigen::Vector3d prev_lin_vel_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d prev_ang_vel_ = Eigen::Vector3d::Zero();
        bool first_log_ = true;
        Eigen::Vector3d smoothed_lin_cmd_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d smoothed_ang_cmd_ = Eigen::Vector3d::Zero();
        double lin_deadband_ = 0.02;
        double ang_deadband_ = 0.2;
        double smooth_alpha_ = 0.2;
        double lin_gain_ = 2.0; // 调整线速度指令放大/缩小
        double yaw_gain_ = 2.0; // 调整旋转步态对指令角速度的放大/缩小系数

        // IMU/定时器状态
        Eigen::Quaterniond imu_quat_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d imu_ang_vel_ = Eigen::Vector3d::Zero();
        bool imu_received_ = false;
        std::string imu_topic_ = "/imu";
        bool use_imu_orientation_ = true;

        ros::Timer update_timer_;
        double cmd_timeout_ = 0.5;
        double target_hz_ = 100.0;
        ros::Time last_cmd_time_;
        bool started_ = false;

};

#endif // RVIZ_KEYBOARD_H