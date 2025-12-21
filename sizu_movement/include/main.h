#ifndef MAIN_H
#define MAIN_H

#include <ros/ros.h>
#include <iostream>
#include "robot_control.h"
#include "EstimateParameters.h"

class Parameters
{
public:
    Parameters();

    double T;
    double dt;
    ros::Rate rate_;
    double high_hop;
};

#endif