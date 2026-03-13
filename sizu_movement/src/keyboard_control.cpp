#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <iostream>

// 终端设置备份
static struct termios cooked, raw;
static int kfd = 0;

// 捕获 Ctrl+C，恢复终端
void quit(int sig)
{
    (void)sig;
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

// 设置终端为非阻塞、无回显模式
void initTerminal()
{
    tcgetattr(kfd, &cooked);
    raw = cooked;
    raw.c_lflag &= ~(ICANON | ECHO);
    // 最少读取 0 个字符（非阻塞），等待时间 0
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_teleop");
    ros::NodeHandle nh("~");

    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    signal(SIGINT, quit);
    initTerminal();

    double linear_step, angular_step;
    nh.param("linear_step", linear_step, 0.1);   // 每次按键线速度增量 m/s
    nh.param("angular_step", angular_step, 0.3); // 每次按键角速度增量 rad/s

    geometry_msgs::Twist twist;

    std::cout << "----- Keyboard Teleop -----" << std::endl;
    std::cout << "w/s : linear x +/-" << std::endl;
    std::cout << "a/d : linear y -/+" << std::endl;
    std::cout << "q/e : angular z +/-" << std::endl;
    std::cout << "space: stop" << std::endl;
    std::cout << "x   : reset to zero (and exit)" << std::endl;

    ros::Rate rate(50); // 50 Hz 发布速度，保证 /cmd_vel 连续更新

    while (ros::ok())
    {
        char c = 0;
        int n = read(kfd, &c, 1); // 非阻塞读取一个字符

        if (n > 0)
        {
            switch (c)
            {
            case 'w':
                twist.linear.x += linear_step;
                break;
            case 's':
                twist.linear.x -= linear_step;
                break;
            case 'a':
                twist.linear.y += linear_step;
                break;
            case 'd':
                twist.linear.y -= linear_step;
                break;
            case 'q':
                twist.angular.z += angular_step;
                break;
            case 'e':
                twist.angular.z -= angular_step;
                break;
            case ' ':
                twist.linear.x = 0.0;
                twist.linear.y = 0.0;
                twist.linear.z = 0.0;
                twist.angular.x = 0.0;
                twist.angular.y = 0.0;
                twist.angular.z = 0.0;
                break;
            case 'x':
                twist = geometry_msgs::Twist(); // 全部清零
                cmd_pub.publish(twist);
                tcsetattr(kfd, TCSANOW, &cooked);
                return 0;
            default:
                break;
            }

            std::cout << "\rvx: " << twist.linear.x
                      << "  vy: " << twist.linear.y
                      << "  wz: " << twist.angular.z
                      << "       " << std::flush;
        }

        cmd_pub.publish(twist);

        ros::spinOnce();
        rate.sleep();
    }

    tcsetattr(kfd, TCSANOW, &cooked);
    return 0;
}