#include <move.h>

using namespace GPIO; // optional
    move::move() : 
        rate_(100),
        tf_listener_(tf_buffer_) {
        
        ros::NodeHandle nh;
  
        dir_ = {0, 0, 0};  // [x, y, theta]
        home_pose_ = {  0, M_PI/4, 3*M_PI/5,
                        0, -M_PI/4, -3*M_PI/5,
                        0, M_PI/4, 3*M_PI/5,
                        0, -M_PI/4, -3*M_PI/5};//关节默认位置

        movement_pose_ ={0.0, 0.0, 0.0};//关节角度
        current_position_={0.0, 0.0, 0.0};//当前关节位置
        leg_length_ = {0.0633, 0.164, 0.3227};
        rate_time = 0.5; // rad/s
        stride_length = 0.08; // m

        box_length =0.2305;//主体长度
        box_width =0.2055;//主体宽度
        
        // 设置终端为非阻塞模式
        tcgetattr(STDIN_FILENO, &oldt_);
        newt_ = oldt_;
        newt_.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt_);

        // 初始化发布者
        initJointStatePublishers();

        pub_motor_commands_ = nh_.advertise<sensor_msgs::JointState>("motor_commands", 10);
        
        


        
        // 启动键盘监听线程
        keyboard_thread_ = std::thread(&move::keyboardListenerThread, this);
    }

    move::~move() {
        // 恢复终端设置
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
    }

    void move::initJointStatePublishers() {
        // 关节状态发布者
        pub_joint_state_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);
        
        message_joint_state_.header.frame_id = "";
        message_joint_state_.name = {
            "coxa_joint_FL", "femur_joint_FL", "tibia_joint_FL",
            "coxa_joint_RL", "femur_joint_RL", "tibia_joint_RL",
            "coxa_joint_FR", "femur_joint_FR", "tibia_joint_FR",
            "coxa_joint_RR", "femur_joint_RR", "tibia_joint_RR"
        };
        message_joint_state_.position.resize(12, 0.0);
        
        // 控制器命令发布者
        // leg_trajectory_controller expects a JointTrajectory on topic "leg_trajectory_controller/command"
        pub_trajectory_ = nh_.advertise<trajectory_msgs::JointTrajectory>("leg_trajectory_controller/command", 10);
        ROS_INFO("Initialized publisher for %s", "leg_trajectory_controller/command");
        traj_msg.header.stamp = ros::Time::now();
        // Use joint names that match the controller (with suffix _joint)
        traj_msg.joint_names = {
            "coxa_joint_FL", "femur_joint_FL", "tibia_joint_FL",
            "coxa_joint_RL", "femur_joint_RL", "tibia_joint_RL",
            "coxa_joint_FR", "femur_joint_FR", "tibia_joint_FR",
            "coxa_joint_RR", "femur_joint_RR", "tibia_joint_RR"
        };
        point.positions.resize(12, 0.0);
        

        motor_commands.header = std_msgs::Header();
        motor_commands.name = {
            "coxa_FL", "femur_FL", "tibia_FL",
            "coxa_RL", "femur_RL", "tibia_RL",
            "coxa_FR", "femur_FR", "tibia_FR",
            "coxa_RR", "femur_RR", "tibia_RR"
        };
        motor_commands.position.resize(12, 0);
        // initialize last_positions_ to current joint positions (all zeros initially)
        last_positions_.assign(message_joint_state_.position.size(), 0.0);
    }
    
    // 计算腿部关节角度
    std::vector<double>move::get_poses(double x,double y,double z) {
        double l=sqrt(x*x+y*y)-leg_length_[0];//投影长度
        double l_=sqrt(l*l+z*z);

        double theta_1=acos((l_*l_+leg_length_[1]*leg_length_[1]-leg_length_[2]*leg_length_[2])/(2*l_*leg_length_[1]))+atan2(z,l);
       
        double theta_2=M_PI-acos((leg_length_[1]*leg_length_[1]+leg_length_[2]*leg_length_[2]-l_*l_)/(2*leg_length_[1]*leg_length_[2]));

        return {theta_1,theta_2};
    }

    std::vector<double>move::inverseKinematics(std::vector<double> xyz, int sign) {

        double off_leg[4]={-M_PI/4,M_PI/4,M_PI/4,-M_PI/4};//腿部外展角偏移量
        double l=leg_length_[0]+leg_length_[1]*cos(home_pose_[1])+leg_length_[2]*sin(M_PI/2+home_pose_[1]-home_pose_[2]);//初始步长
        double h=leg_length_[2]*cos(M_PI/2+home_pose_[1]-home_pose_[2])-leg_length_[1]*sin(home_pose_[1]);//初始抬高高度
        std::vector<double> xyz_0={l*cos(M_PI/4), l*sin(M_PI/4), -h,
                                  -l*cos(M_PI/4), l*sin(M_PI/4), -h,
                                   l*cos(M_PI/4),-l*sin(M_PI/4), -h,
                                  -l*cos(M_PI/4),-l*sin(M_PI/4), -h};//初始足端位置
        int sign_=xyz_0[sign*3]*xyz_0[sign*3+1]/abs(xyz_0[sign*3+1]*xyz_0[sign*3]);//判断

        double theta_1=atan((xyz_0[sign*3+1]+xyz[1])/(xyz_0[sign*3]+xyz[0]))+off_leg[sign];//髋关节角度计算
        std::vector<double> theta_2=get_poses(xyz[0]+xyz_0[sign*3],xyz[1]+xyz_0[sign*3+1],xyz[2]+xyz_0[sign*3+2]);
        std::vector<double>  theta_={theta_1, sign_*theta_2[0], sign_*theta_2[1]};//关节角度结果存储

        return theta_;
    }
        


     void move::commandPosition(std::vector<double> xyz, std::vector<int> sign) {
       
        std::vector<std::string> leg_order = {"FL", "RL", "FR", "RR"};
        std::vector<double> dir_sign= { 1, 1, 1,
                                        1,-1,-1,
                                        1, 1, 1,
                                        1,-1,-1};//实机修正矩阵

        int j=0;
        for(int i=0;i<4;i++){
            if(sign[j]==i){ 
                std::vector<double> ang = inverseKinematics(xyz, sign[j]);


                    message_joint_state_.position[i*3] = ang[0]*dir_sign[i*3];    
                    message_joint_state_.position[i*3+1] = ang[1]*dir_sign[i*3+1];
                    message_joint_state_.position[i*3+2] = ang[2]*dir_sign[i*3+2];

                    point.positions[i*3] = message_joint_state_.position[i*3];
                    point.positions[i*3+1] = message_joint_state_.position[i*3+1];
                    point.positions[i*3+2] = message_joint_state_.position[i*3+2];
      
                    ROS_INFO("Leg %d angles (radians): coxa=%.4f, femur=%.4f, tibia=%.4f", 
                    sign[j], message_joint_state_.position[i*3], message_joint_state_.position[i*3+1], message_joint_state_.position[i*3+2]);

                   
                    double coxa_deg = message_joint_state_.position[i*3] * 180 / M_PI;
                    double femur_deg = message_joint_state_.position[i*3+1] * 180 / M_PI;
                    double tibia_deg = message_joint_state_.position[i*3+2] * 180 / M_PI;


                    int coxa_scaled = static_cast<int>(coxa_deg * 36 * 100);
                    int femur_scaled = static_cast<int>(femur_deg * 36 * 100);
                    int tibia_scaled = static_cast<int>(tibia_deg * 36 * 100);
                
                    motor_commands.position[i*3] = coxa_scaled;
                    motor_commands.position[i*3+1] = femur_scaled;
                    motor_commands.position[i*3+2] = tibia_scaled;
                

                    // ROS_INFO("Leg %d angles (scaled): coxa=%d, femur=%d, tibia=%d", 
                    //     sign[j], coxa_scaled, femur_scaled, tibia_scaled);

                    // We will publish a full JointTrajectory message for the trajectory controller later
                    // (individual per-joint position controllers are not started in launch)
                
                    j++;
                }

               }
       
        pub_motor_commands_.publish(motor_commands);

        
    
        // compute velocities as (pos - last_pos) / dt (use time_from_start as dt)
        ros::Duration dt = ros::Duration(0.1);
        point.time_from_start = dt;
        
        // point.velocities.resize(point.positions.size(), 0.0);
        // for (size_t k = 0; k < point.positions.size(); ++k) {
        //     double last = 0.0;
        //     if (k < last_positions_.size()) last = last_positions_[k];
        //     point.velocities[k] = (point.positions[k] - last) / dt.toSec();
        // }
        // update last_positions_
        last_positions_.clear();
        last_positions_ = point.positions;

        // ensure we don't accumulate points with non-increasing time_from_start
        traj_msg.header.stamp = ros::Time::now();
        traj_msg.points.clear();
        traj_msg.points.push_back(point);
        pub_trajectory_.publish(traj_msg);

        rate_.sleep();
    }

    
    void move::keyboardListenerThread() {
        ROS_INFO("Keyboard listener started. Use WASD to move, JK to rotate, Q to quit.");
        
        while (ros::ok()) {
            fd_set set;
            struct timeval timeout;
            FD_ZERO(&set);
            FD_SET(STDIN_FILENO, &set);
            timeout.tv_sec = 0;
            timeout.tv_usec = 10000;  // 10ms
            
            int result = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);
            
            if (result > 0) {
                char c;
                if (read(STDIN_FILENO, &c, 1) > 0) {
                    if (c == 'q') {
                        ROS_INFO("Quitting...");
                        ros::shutdown();
                        return;
                    }
                    
                    // 处理按键按下
                    switch (c) {
                        case 'w': dir_[1] = 1.0; break;  // 前进
                        case 's': dir_[1] = -1.0; break; // 后退
                        case 'a': dir_[0] = 1.0; break;  // 左移
                        case 'd': dir_[0] = -1.0; break; // 右移
                        case 'j': dir_[2] = 1.0; break;  // 左转
                        case 'k': dir_[2] = -1.0; break; // 右转
                    }
                    // ROS_INFO("Key pressed: %c, dir=[%.1f, %.1f, %.1f]", c, dir_[0], dir_[1], dir_[2]);
                }
            } else if (result == 0) {
                // 无按键输入时停止运动
                if (dir_[0] != 0 || dir_[1] != 0 || dir_[2] != 0) {
                    dir_ = {0, 0, 0};
                    // ROS_INFO("Key released. Stopping movement.");
                }
            }
        }
    }

    void move::runWaveMode(std::vector<double> xyz, int sign) {
        
        std::vector<int> leg_order= {0, 1, 2, 3}; //"FL", "RL", "FR", "RR"

            for(int i=0;i<2;i++){
            commandPosition({xyz[0]*sign/2, xyz[1]*sign/2, xyz[2]}, {i});
            commandPosition({xyz[0]*sign/2, xyz[1]*sign/2, xyz[2]}, {3-i});
            ros::Duration(0.1).sleep();
            commandPosition({xyz[0]*sign, xyz[1]*sign, 0}, {i});
            commandPosition({xyz[0]*sign, xyz[1]*sign, 0}, {3-i});
            ros::Duration(0.1).sleep();
            commandPosition({0, 0, 0}, {i});
            commandPosition({0, 0, 0}, {3-i});
            }

        // createDefaultPoses();
        // commandPosition({0.0, 0.0, 0.0}, leg_order); 
        // ros::Duration(0.1).sleep();
        
    }

    void move::runRotationMode(std::vector<double> xyz, int sign) {
       std::vector<int> leg_order= {0, 1, 2, 3}; //"FL", "RL", "FR", "RR"
       std::vector<int> Rotion_dir={-1, 1, -1, -1, 1, 1, 1, -1};
            for(int i=0;i<2;i++){
            commandPosition({xyz[0]*Rotion_dir[2*i]*sign, xyz[1]*Rotion_dir[2*i+1]*sign, xyz[2]}, {i});
            commandPosition({xyz[0]*Rotion_dir[2*(3-i)]*sign, xyz[1]*Rotion_dir[2*(3-i)+1]*sign, xyz[2]}, {3-i});
            ros::Duration(0.1).sleep();
            commandPosition({xyz[0]*Rotion_dir[2*i]*sign, xyz[1]*Rotion_dir[2*i+1]*sign, 0}, {i});
            commandPosition({xyz[0]*Rotion_dir[2*(3-i)]*sign, xyz[1]*Rotion_dir[2*(3-i)+1]*sign, 0}, {3-i});
            ros::Duration(0.1).sleep();
            
            }
       

        // createDefaultPoses();
        commandPosition({0.0, 0.0, 0.0}, leg_order);
        ros::Duration(0.1).sleep();
        
    }

    void move::RlMoveMode(std::vector<double> xyz, int sign) {
       
        std::vector<int> leg_order= {0, 1, 2, 3}; //"FL", "RL", "FR", "RR"
        
            for(int i=0;i<2;i++){
            commandPosition({xyz[0]*sign/2, xyz[1]*sign/2, xyz[2]}, {i});
            commandPosition({xyz[0]*sign/2, xyz[1]*sign/2, xyz[2]}, {3-i});
            ros::Duration(0.1).sleep();
            commandPosition({xyz[0]*sign, xyz[1]*sign, 0}, {i});
            commandPosition({xyz[0]*sign, xyz[1]*sign, 0}, {3-i});
            ros::Duration(0.1).sleep();
            }

        // createDefaultPoses();
        commandPosition({0.0, 0.0, 0.0}, leg_order); 
        ros::Duration(0.1).sleep();
    }
    
    std::vector<double> move::get_Rotation_xyz(double angle) {
        double l=leg_length_[0]+leg_length_[1]*cos(home_pose_[1])+leg_length_[2]*sin(M_PI/2+home_pose_[1]-home_pose_[2]);//初始步长
        double l_box=sqrt((box_length/2)*(box_length/2)+(box_width/2)*(box_width/2));
        double l_leg=sqrt((box_length/2+l*sin(M_PI/4))*(box_length/2+l*sin(M_PI/4))+(box_width/2+l*cos(M_PI/4))*(box_width/2+l*cos(M_PI/4)));
        double box_angle=acos((l_leg*l_leg+l*l-l_box*l_box)/(2*l_leg*l));//箱体与腿部连线夹角
        double theta_1=(M_PI-angle)/2;//旋转角度
        double l_1=2*l_leg*cos(theta_1);
        double theta_2=theta_1-box_angle-M_PI/4;
        double x=l_1*sin(theta_2);
        double y=l_1*cos(theta_2);

        return {x,y};
    }
     void move::mainLoop() {
        while (ros::ok()) {
            if (dir_[0] == 0 && dir_[1] == 0 && dir_[2] == 0) {
                // 静止状态
                commandPosition({0.0, 0.0, 0.0}, {0, 1, 2, 3}); 
            } 
            else if (dir_[2] != 0 && dir_[0] == 0 && dir_[1] == 0) {
                // 旋转模式
                std::vector<double> Rotion_xyz=get_Rotation_xyz(M_PI/12);
                runRotationMode({Rotion_xyz[0], Rotion_xyz[1], 0.07}, dir_[2]);
            } 
            else if (dir_[0] == 0 && dir_[1] != 0 && dir_[2] == 0) {
                // 前后移动模式
                runWaveMode({stride_length, 0.0, 0.07}, dir_[1]);
            }
            else {
                // 左右移动模式
                RlMoveMode({0.0, stride_length, 0.07}, dir_[0]);
            }
            
            ros::spinOnce();
            rate_.sleep();
        }
        // 恢复终端输入模式，防止终端异常
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);
        // 清理GPIO设置
        // GPIO::cleanup();
    }


    int main(int argc, char** argv) {
    ros::init(argc, argv, "move");
    move Move;
    Move.mainLoop();
    return 0;
}