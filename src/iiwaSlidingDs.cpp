#include "iiwaSlidingDs.h"


iiwaSlidingDs* iiwaSlidingDs::me = NULL;

iiwaSlidingDs::iiwaSlidingDs(ros::NodeHandle &n,double frequency, ControllerMode controllerMode): 
  _n(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _controllerMode(controllerMode)
{
    me = this;
    _desired_jnt_torque[No_JOINTS]   = {0.0};
    _cmd_jnt_torque.data.resize(7);


}

bool iiwaSlidingDs::init()
{
    for (int i = 0; i < No_Robots;i++){
        _robot.push_back(Robot());
        _robot[i].name +=std::to_string(i);

        _robot[i].X_ee.setConstant(0.0);
        _robot[i].V_ee.setConstant(0.0);
        _robot[i].X_ee_attractor.setConstant(0.0); 
        _robot[i].V_ee_desired.setConstant(0.0);
        
    }
    _optitrackOK = true;
    for(int k = 0; k < TOTAL_No_MARKERS; k++)
      _firstOptitrackPose[k] = true;

    // Init Subscribers
    for(int i=0; i<No_Robots; i++)
    {
        //todo: the name of the subscribed topic has to change as it is not robot specific
        _subRobotStates[i]= _n.subscribe<sensor_msgs::JointState> ("/iiwa/joint_states", 1,
            boost::bind(&iiwaSlidingDs::updateRobotStates,this,_1,i),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
        
    }
    // init Publishers
    _TrqCmd = _n.advertise<std_msgs::Float64MultiArray>("/iiwa/TorqueController/command",1);
    // _PsCmd[i] = _n.advertise<std_msgs::Float64MultiArray>("/iiwa/XXXXX",1);
    
    // Ini controller
    dsGain = 7.0; //todo : late it should be replaced by a ds

    // Get jacobain
    // Get the URDF XML from the parameter server
    std::string urdf_string, full_param;
    std::string robot_description = "robot_description";
    std::string end_effector;

    // gets the location of the robot description on the parameter server
    if (!_n.searchParam(robot_description, full_param)) {
        ROS_ERROR("Could not find parameter %s on parameter server", robot_description.c_str());
        return false;
    }
    // search and wait for robot_description on param server
    while (urdf_string.empty()) {
        ROS_INFO_ONCE_NAMED("Controller", "Controller is waiting for model"
                                                        " URDF in parameter [%s] on the ROS param server.",
            robot_description.c_str());
        _n.getParam(full_param, urdf_string);
        usleep(100000);
    }
    ROS_INFO_STREAM_NAMED("Controller", "Received urdf from param server, parsing...");
    // Get the end-effector
    _n.param<std::string>("params/end_effector", end_effector, "iiwa_link_ee");
    
    // Initialize iiwa tools
    _tools.init_rbdyn(urdf_string, end_effector);

}

// --------robotstate-------------------------


// --------------define jacaboNIA--------------------
Eigen::MatrixXd jac(6, No_JOINTS);
Eigen::MatrixXd jac_deriv(6, No_JOINTS);

// --------------------------------------------

// -----------------------------------

void iiwaSlidingDs::run()
{
    while(ros::ok()){ 
        if (_firstOptitrackPose[0]&&_firstOptitrackPose[1]){
            _mutex.lock();      
            if(!_optitrackOK){
                // ROS_INFO("[ObjectGrasping]: Optitrack Initialization ...");
                optitrackInitialization();
            }else{
                // Do All Updating
                updateRobotInfo();

                // Compute and publish command
                computeCommand();
                publishData();
            }

            _mutex.unlock();
        }
        ros::spinOnce();
        _loopRate.sleep();
    }

    publishData();
    ros::spinOnce();
    _loopRate.sleep();

    ros::shutdown();
}

//

//================= Kowledge Updating ==================//
void iiwaSlidingDs::updateRobotStates(const sensor_msgs::JointState::ConstPtr &msg, int k){
    for (int i = 0; i < No_JOINTS; i++){
      _robot[k].jnt_position[i] = msg->position[i];
      _robot[k].jnt_velocity[i] = msg->velocity[i];
      _robot[k].jnt_torque[i]   = msg->effort[i];
   }
}
void iiwaSlidingDs::publishData(){   
    if(_controllerMode == Torque_Mode){
        for(int i = 0; i < No_JOINTS; i++)
            _cmd_jnt_torque.data[i] = _desired_jnt_torque[i];
        _TrqCmd.publish(_cmd_jnt_torque);
    }
    else if(_controllerMode == Position_Mode){}
        // _PsCmd[j].publish(_msgDesiredjntStates);

}
void iiwaSlidingDs::updateRobotInfo(){
    // ROS_INFO("It Works...");

    
    iiwa_tools::RobotState robot_state;
    robot_state.position.resize(No_JOINTS);
    robot_state.velocity.resize(No_JOINTS);
    for (size_t i = 0; i < No_JOINTS; i++) {
        robot_state.position[i] = _robot[0].jnt_position[i];
        robot_state.velocity[i] = _robot[0].jnt_position[i];
    }

    std::tie(_robot[0].jacob, _robot[0].jacob_drv) = _tools.jacobians(robot_state);
    // std::cout<< _robot[0].jacob << std::endl<< std::endl;

    // jac_t_pinv = pseudo_inverse(Eigen::MatrixXd(jac.transpose()));
    auto ee_state = _tools.perform_fk(robot_state);
    _robot[0].X_ee = ee_state.translation;

    // std::cout<< _robot[0].X_ee << std::endl<< std::endl;
    // Q_ee

    // _robot[0].V_ee

    // _robot[0].A_ee

    // _robot[0].W_ee

    // _robot[0].Wdot_ee

}
void iiwaSlidingDs::optitrackInitialization(){

}
//====================================================//
void iiwaSlidingDs::computeCommand(){
    // assuming all feedback and required info are updated:
    
    // Get desired task
    // for exmp computeDs()
    // A dummy Ds for test, //todo
    _robot[0].V_ee_desired   = dsGain * Eigen::Matrix3d::Identity(3,3) * (_robot[0].X_ee_attractor - _robot[0].X_ee);

    // get desired force in task space
        // position 

        // Orientation

    //nullspace Controller

    // Gravity Compensationn

    // Checking min and max joint trque limit

    for (int i = 0; i < No_JOINTS; i++)
        _desired_jnt_torque[i] =  0.1;
}

