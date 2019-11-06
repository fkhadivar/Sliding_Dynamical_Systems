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

// Passive Ds 
#include <passive_ds_controller.h>
#include "Utils.h"


#define No_Robots 1
#define No_JOINTS 7
#define TOTAL_No_MARKERS 2

// struct State {
//     Eigen::Vector3d pos, vel, acc, angVel, angAcc;
//     Eigen::Quaterniond ee_quat;
//     public:
//         EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// };
struct Robot
{
    unsigned int no_joints = No_JOINTS;
    Eigen::VectorXd jnt_position = Eigen::VectorXd(no_joints);
    Eigen::VectorXd jnt_velocity = Eigen::VectorXd(no_joints);
    Eigen::VectorXd jnt_torque = Eigen::VectorXd(no_joints);
    
    Eigen::VectorXd nulljnt_position = Eigen::VectorXd(no_joints);
    std::string name = "robot_";

    Eigen::Vector3d ee_pos, ee_vel, ee_acc, ee_angVel, ee_angAcc;
    Eigen::Vector4d ee_quat;
    Eigen::Vector3d ee_des_pos, ee_des_vel, ee_des_acc, ee_des_angVel, ee_des_angAcc;
    Eigen::Vector4d ee_des_quat;
    // State ee;
    // State ee_desired;
    // State ee_p;

    // Eigen::Vector3d X_ee_attractor, V_ee_desired;

    Eigen::MatrixXd jacob       = Eigen::MatrixXd(6, No_JOINTS);
    Eigen::MatrixXd jacob_drv   = Eigen::MatrixXd(6, No_JOINTS);
    Eigen::MatrixXd jacob_t_pinv= Eigen::MatrixXd(No_JOINTS, 6);
    Eigen::MatrixXd jacobPos    = Eigen::MatrixXd(3, No_JOINTS);
    Eigen::MatrixXd jacobAng    = Eigen::MatrixXd(3, No_JOINTS);

    Eigen::MatrixXd pseudo_inv_jacob       = Eigen::MatrixXd(6,6);
    Eigen::MatrixXd pseudo_inv_jacobPos    = Eigen::MatrixXd(3,3);
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> pseudo_inverse(const MatT& mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto& singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance) {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}
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
        std_msgs::Float64MultiArray _plotVar;
        ros::Publisher _plotter;
        // ros::Publisher _PsCmd[No_Robots];

        //======================== Messeges================================//

        
        //======================== Robot===================================//
        std::vector<Robot> _robot;
        iiwa_tools::IiwaTools _tools;
        
        
        //======================== Control=================================//
        double _desired_jnt_torque[No_JOINTS]   = {0.0};
        std_msgs::Float64MultiArray _cmd_jnt_torque;
        
        ControllerMode _controllerMode;
        double dsGainPos;
        double dsGainAng;
        
        double _gainContPos;
        double _gainContAng;
        DSController * pdsCntrPos;          // Passive Ds controller
        DSController * pdsCntrAng;          // Passive Ds controller

        double sldGain;
        double nullCntrGainP;
        //======================== World ==================================//
        bool _optitrackOK;
        bool _firstOptitrackPose[TOTAL_No_MARKERS];
        
        //======================== Other variables ========================//
        std::mutex _mutex;
        static iiwaSlidingDs* me;
        double VELIMIT;
        double filterGain;
    
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