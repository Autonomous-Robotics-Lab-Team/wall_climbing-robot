#include "main.h"

Parameters::Parameters()
:rate_(10){
    // 初始化参数的代码
    T  = 0.6;
    dt = 0.15;
    rate_ = ros::Rate(20);
    high_hop = 0.08;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "main_node");
    ros::Time::init();
    Parameters Pa;
    EstimateParameters ep;
    ep.move.get_parameters(Pa.T, Pa.dt);
    ep.get_parameters(Pa.T, Pa.dt, Pa.high_hop);
    ep.mainloop();
  
    ros::spin();
    return 0;
}