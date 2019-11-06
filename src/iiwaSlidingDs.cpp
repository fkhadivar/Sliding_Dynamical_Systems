#include "iiwaSlidingDs.h"


iiwaSlidingDs* iiwaSlidingDs::me = NULL;

iiwaSlidingDs::iiwaSlidingDs(ros::NodeHandle &n,double frequency, ControllerMode controllerMode): 
  _n(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _controllerMode(controllerMode)
{
    me = this;
    filterGain = 0.4;
    VELIMIT = 1.0;
    _desired_jnt_torque[No_JOINTS]   = {0.0};
    _cmd_jnt_torque.data.resize(No_JOINTS);
    _plotVar.data.resize(3);


}

bool iiwaSlidingDs::init()
{
    for (int i = 0; i < No_Robots;i++){
        _robot.push_back(Robot());
        _robot[i].name +=std::to_string(i);
        _robot[i].jnt_position.setZero();
        _robot[i].jnt_velocity.setZero();
        _robot[i].jnt_torque.setZero();
        _robot[i].nulljnt_position.setZero();
        _robot[i].ee_pos.setZero(); 
        _robot[i].ee_vel.setZero();   
        _robot[i].ee_acc.setZero();

        
        double angle0 = 0.25*M_PI;
        _robot[i].ee_quat[0] = (std::cos(angle0/2));
        _robot[i].ee_quat.segment(1,3) = (std::sin(angle0/2))* Eigen::Vector3d::UnitZ();
        
        _robot[i].ee_angVel.setZero();
        _robot[i].ee_angAcc.setZero();
        _robot[i].ee_des_pos = {0.2 , 0.2, 0.5}; 
        _robot[i].ee_des_vel.setZero();   
        _robot[i].ee_des_acc.setZero();

        // _robot[i].ee_des_quat = Eigen::Quaterniond::Identity();
        // _robot[i].ee_des_quat.w() = -1*_robot[i].ee_des_quat.w(); 
        
         double angled = 0.25*M_PI;
        _robot[i].ee_des_quat[0] = (std::cos(angled/2));
        _robot[i].ee_des_quat.segment(1,3) = (std::sin(angled/2))* Eigen::Vector3d::UnitZ();
        
        _robot[i].ee_des_angVel.setZero();
        _robot[i].ee_des_angAcc.setZero();
        // _robot[i].ee_p.pos.setZero(); _robot[i].ee_p.vel.setZero();   _robot[i].ee_p.acc.setZero();
        // _robot[i].ee_p.quat = Eigen::Quaterniond::Identity();
        // _robot[i].ee_p.angVel.setZero();_robot[i].ee_p.angAcc.setZero();
        _robot[i].jacob.setZero();
        _robot[i].jacob.setZero();       
        _robot[i].jacob_drv.setZero();   
        _robot[i].jacob_t_pinv.setZero();
        _robot[i].jacobPos.setZero();   
        _robot[i].jacobAng.setZero();
        _robot[i].pseudo_inv_jacob.setZero();   
        _robot[i].pseudo_inv_jacobPos.setZero();

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
    _plotter = _n.advertise<std_msgs::Float64MultiArray>("/iiwa/plotvar",1);
    // _PsCmd[i] = _n.advertise<std_msgs::Float64MultiArray>("/iiwa/XXXXX",1);
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
    // Ini controller
    dsGainPos = 2.50; //todo : late it should be replaced by a ds
    dsGainAng = 2.50;

    // Init Passive Ds
   _gainContPos  = 5.0;
   _gainContAng  = 5.0;
   pdsCntrPos    = new DSController(3.0, _gainContPos, 5*_gainContPos);
   pdsCntrAng    = new DSController(3.0, _gainContAng, 5*_gainContAng);
   sldGain = 100.0;

   nullCntrGainP = 5.0;

}

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
      _robot[k].jnt_position[i] = (double)msg->position[i];
      _robot[k].jnt_velocity[i] = (double)msg->velocity[i];
      _robot[k].jnt_torque[i]   = (double)msg->effort[i];
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

    _plotter.publish(_plotVar);
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
    _robot[0].jacobPos =  _robot[0].jacob.bottomRows(3);
    _robot[0].jacobAng =  _robot[0].jacob.topRows(3);

    _robot[0].pseudo_inv_jacob    = pseudo_inverse(Eigen::MatrixXd(_robot[0].jacob * _robot[0].jacob.transpose()) );
    _robot[0].pseudo_inv_jacobPos = pseudo_inverse(Eigen::MatrixXd(_robot[0].jacobPos * _robot[0].jacobPos.transpose()) );

    // jac_t_pinv = pseudo_inverse(Eigen::MatrixXd(jac.transpose()));
    
    // _robot[0].ee.quat // _robot[0].ee.pos
    auto ee_state = _tools.perform_fk(robot_state);
    _robot[0].ee_pos = ee_state.translation;
    _robot[0].ee_quat[0] = ee_state.orientation.w();
    _robot[0].ee_quat.segment(1,3) = ee_state.orientation.vec();
    
    /// _robot[0].ee.vel // _robot[0].ee.angVel     
    Eigen::VectorXd vel = _robot[0].jacob * _robot[0].jnt_velocity;
    _robot[0].ee_vel    = vel.tail(3); // check whether this is better or filtering position derivitive

    _robot[0].ee_angVel = vel.head(3); // compare it with your quaternion derivitive equation
    // _robot[0].ee.acc// _robot[0].ee.angAcc
    // std::cout<< _robot[0].jacob << std::endl<< std::endl;
    // std::cout<< _robot[0].ee.quat.w() << std::endl<< _robot[0].ee.quat.vec()<< std::endl<< std::endl;

    for(int i = 0; i < 3; i++)
    {
        _plotVar.data[i] = (_robot[0].ee_des_pos - _robot[0].ee_pos)[i];
        // if (_robot[0].ee_vel[i] > 1.0){
        //     _plotVar.data[i]=1.0;
        // }else if (_robot[0].ee_vel[i] < -1.0){
        //     _plotVar.data[i]=-1.0;
        // }
    }
    // std::cout<< "velocity"<< std::endl<<_robot[0].ee.vel << std::endl<< std::endl;
    // std::cout<< "angular velocity"<< std::endl<<_robot[0].ee.angVel << std::endl<< std::endl;
}
void iiwaSlidingDs::optitrackInitialization(){

}
//====================================================//
void iiwaSlidingDs::computeCommand(){
    // assuming all feedback and required info are updated:
    
    // Get desired task
    // for exmp computeDs()
    
    //------------------------- A dummy Ds for test, //todo
    _robot[0].ee_des_vel   = dsGainPos * Eigen::Matrix3d::Identity(3,3) * (_robot[0].ee_des_pos - _robot[0].ee_pos);
    
    // ROS_INFO("It Works...");
    //angular velocity
    //change here
    // quaternionProduct(_qbinv.normalized(),_qo.normalized());
    Eigen::Vector4d dqd = Utils<double>::slerpQuaternion(_robot[0].ee_quat, _robot[0].ee_des_quat, 0.5);
    // _robot[0].ee_quat.slerp(0.5,_robot[0].ee_des_quat);
    Eigen::Vector4d deltaQ;
    // for some reason this for loop is necessary!
    for (int i =0; i<4; i++)
      deltaQ[i] = dqd[i] -  _robot[0].ee_quat[i];

    Eigen::Vector4d qconj = _robot[0].ee_quat;
    qconj.segment(1,3) = -1 * qconj.segment(1,3);
    Eigen::Vector4d temp_angVel = Utils<double>::quaternionProduct(qconj, deltaQ);
    _robot[0].ee_des_angVel    = 2 * dsGainAng * temp_angVel.segment(1,3);
    
    // -----------------------get desired force in task space
    // position 
    pdsCntrPos->Update(_robot[0].ee_vel,_robot[0].ee_des_vel);
    Eigen::Vector3d wrenchPos = pdsCntrPos->control_output();
    
    // adding sliding ds
    Eigen::Matrix<double,3,3> dampMat = Eigen::Matrix<double,3,3>(pdsCntrPos->damping_matrix());
    // wrenchPos += -sldGain * dampMat * (_robot[0].ee_pos - _robot[0].ee_des_pos);
    wrenchPos += sldGain * dampMat * (_robot[0].ee_des_pos);
    
    Eigen::VectorXd tmp_jnt_trq_pos = Eigen::VectorXd(No_JOINTS);
    tmp_jnt_trq_pos = _robot[0].jacobPos.transpose() * wrenchPos;
    // Orientation
    pdsCntrAng->Update(_robot[0].ee_angVel,_robot[0].ee_des_angVel);
    Eigen::Vector3d wrenchAng   = pdsCntrAng->control_output();
    Eigen::VectorXd tmp_jnt_trq_ang = Eigen::VectorXd(No_JOINTS);
    tmp_jnt_trq_ang = _robot[0].jacobAng.transpose() * wrenchAng;
    
    //sum up:
    Eigen::MatrixXd IdenMat = Eigen::MatrixXd::Identity(No_JOINTS,No_JOINTS);

    Eigen::VectorXd tmp_jnt_trq = Eigen::VectorXd(No_JOINTS);
    // first method(Walid's):
    tmp_jnt_trq = tmp_jnt_trq_pos + tmp_jnt_trq_ang;
    
    // second method(exploiting null space)
    // Eigen::MatrixXd tempMat = IdenMat - _robot[0].jacobPos.transpose()* _robot[0].pseudo_inv_jacobPos* _robot[0].jacobPos;
    // tmp_jnt_trq = tmp_jnt_trq_pos + tempMat * tmp_jnt_trq_ang;
    
    //----------------------nullspace Controller
    // jut position
    // Eigen::MatrixXd tempMat = IdenMat- _robot[0].jacobPos.transpose()* _robot[0].pseudo_inv_jacobPos* _robot[0].jacobPos;
    
    // position and orientation
    Eigen::MatrixXd tempMat2 =  IdenMat - _robot[0].jacob.transpose()* _robot[0].pseudo_inv_jacob* _robot[0].jacob;
    Eigen::VectorXd tmp_null_trq; 
    tmp_null_trq = -nullCntrGainP * (_robot[0].jnt_position -_robot[0].nulljnt_position);

    // ROS_INFO_STREAM(tmp_null_trq.rows()<<"x"<<tmp_null_trq.cols()<<" vs "<<tempMat.rows()<<"x"<<tempMat.cols());
    tmp_jnt_trq += tempMat2 * tmp_null_trq;
    // Gravity Compensationn

    // Checking min and max joint trque limit
    // double tmep[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.2};
    for (int i = 0; i < No_JOINTS; i++)
        _desired_jnt_torque[i] =  tmp_jnt_trq[i];
}



