#ifndef _IIWA_SLIDING_DS_
#define _IIWA_SLIDING_DS_

#include "ros/ros.h"
#include <ros/package.h>

#include <mutex>
#include <fstream>
#include <pthread.h>
#include <vector>

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
// #include "boost/bind.hpp" 
#include <Eigen/Dense>
// Iiwa tools
#include <iiwa_tools/iiwa_tools.h>

#define No_Robots 1
#define No_JOINTS 7
#define TOTAL_No_MARKERS 2

struct State {
    Eigen::Vector3d pos, vel, acc, angVel, angAcc;
    Eigen::Quaterniond quat;
};
struct Robot
{
    unsigned int no_joints = No_JOINTS;
    float jnt_position[No_JOINTS] = {0.0};
    Eigen::VectorXd jnt_velocity = Eigen::VectorXd(no_joints);
    float jnt_torque[No_JOINTS]   = {0.0};
    std::string name = "robot_";

    State ee;
    State ee_desired;

    // Eigen::Vector3d X_ee_attractor, V_ee_desired;

    Eigen::MatrixXd jacob       =Eigen::MatrixXd(6, No_JOINTS);
    Eigen::MatrixXd jacob_drv   =Eigen::MatrixXd(6, No_JOINTS);
    Eigen::MatrixXd jacob_t_pinv=Eigen::MatrixXd(No_JOINTS, 6);
};

class iiwaSlidingDs
{
    public:
        enum ControllerMode {Position_Mode = 0 , Torque_Mode = 1};

    private:
        //======================== Node ==================================//
        ros::NodeHandle _n;
        ros::Rate _loopRate;
        float _dt;

        //======================== Subscribers ============================//
        ros::Subscriber _subRobotStates[No_Robots];
        
        //======================== Publisher ==============================//
        ros::Publisher _TrqCmd;
        // ros::Publisher _PsCmd[No_Robots];

        //======================== Messeges================================//

        
        //======================== Robot===================================//
        std::vector<Robot> _robot;
        // Iiwa tools
        iiwa_tools::IiwaTools _tools;
        
        //======================== Control=================================//
        double _desired_jnt_torque[No_JOINTS]   = {0.0};
        std_msgs::Float64MultiArray _cmd_jnt_torque;
        ControllerMode _controllerMode;
        double dsGain;
        //======================== World ==================================//
        bool _optitrackOK;
        bool _firstOptitrackPose[TOTAL_No_MARKERS];
        
        //======================== Other variables ========================//
        std::mutex _mutex;
        static iiwaSlidingDs* me;
    
    public:
        iiwaSlidingDs(ros::NodeHandle &n,double frequency, ControllerMode controllerMode);
        bool init();
        void run();

    private:
        void updateRobotStates(const sensor_msgs::JointState::ConstPtr &msg, int k);
        void publishData();
        void updateRobotInfo();
        void computeCommand();
        void optitrackInitialization();





};

#endif