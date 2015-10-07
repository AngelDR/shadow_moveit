/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Angel Delgado */

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>
// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <std_msgs/Float64.h>
#include <pr2_controllers_msgs/JointControllerState.h>
// PI
#include <boost/math/constants/constants.hpp>

// Shared robot_model & robot_state
robot_model::RobotModelPtr sharedKinematic_model;
robot_state::RobotStatePtr sharedKinematic_state; 

// joint group
/**const robot_state::JointModelGroup* ff_joint_model_group;
const robot_state::JointModelGroup* mf_joint_model_group;
const robot_state::JointModelGroup* rf_joint_model_group;
const robot_state::JointModelGroup* lf_joint_model_group;
const robot_state::JointModelGroup* th_joint_model_group;
*/

const robot_state::JointModelGroup* joint_model;
std::string* tip_name;

// GENERIC FINGER -> FIRST, MIDDLE AND RING FINGER
/**
* Joint position callback: j0  (only for first, middle, ring and little fingers)
*/
void _j0Callback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
{
    const double vSet_point= (double) msg->set_point;
    const std::vector<std::string> &joint_names = joint_model->getJointModelNames();
    sharedKinematic_state->setJointPositions(joint_names[3],&vSet_point);
    
    // j0 usado por ff, mf, rf y lf.  * Cambia el modo de obtener los nombres de las articulaciones, porque el numero de las articulaciones son distintos
    if((*tip_name == "fftip") || (*tip_name == "mftip") || (*tip_name == "rftip"))
      sharedKinematic_state->setJointPositions(joint_names[2],&vSet_point);
    else
      sharedKinematic_state->setJointPositions(joint_names[4],&vSet_point);
      
}


/**
* Joint position callback: j1  (only for thumb)
*/
void _j1Callback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
{
  const double vSet_point= (double) msg->set_point;
  const std::vector<std::string> &joint_names = joint_model->getJointModelNames();
  sharedKinematic_state->setJointPositions(joint_names[4],&vSet_point);
}

/**
* Joint position callback: j2 (only for thumb)
*/
void _j2Callback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
{
  const double vSet_point= (double) msg->set_point;
  const std::vector<std::string> &joint_names = joint_model->getJointModelNames();
  sharedKinematic_state->setJointPositions(joint_names[3],&vSet_point);  
}



/**
* Joint position callback: j3  (usado por todos los dedos -> )
* aquí se implementa la obtención the iK y Jacobian 
*/
void _j3Callback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
{
  const double vSet_point= (double) msg->set_point;
  const std::vector<std::string> &joint_names = joint_model->getJointModelNames();
  //sharedKinematic_state->setJointPositions(joint_names[1],&vSet_point);
  
  // j3 usado por th, ff, mf, rf y lf.  * Cambia el modo de obtener los nombres de las articulaciones, porque el numero de las articulaciones son distintos
  if((*tip_name == "lftip") || (*tip_name == "thtip"))
    sharedKinematic_state->setJointPositions(joint_names[2],&vSet_point);
  else
    sharedKinematic_state->setJointPositions(joint_names[1],&vSet_point);
  
  
  
      // Mostrar valor joint
    std::vector<double> joint_values;
    sharedKinematic_state->copyJointGroupPositions(joint_model, joint_values);
    for(std::size_t i = 0; i < joint_names.size(); ++i)
      {
	ROS_INFO("Joint state - %s: %f", joint_names[i].c_str(), joint_values[i]);
      }

    
    const Eigen::Affine3d &end_effector_state = sharedKinematic_state->getGlobalLinkTransform(*tip_name);
    

    // Print end-effector pose. Remember that this is in the model frame 
    ROS_INFO(" \n End effector state \n");
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation());
    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation());

    // Inverse Kinematics
    // ^^^^^^^^^^^^^^^^^^
    bool found_ik = sharedKinematic_state->setFromIK(joint_model, end_effector_state, 10, 0.1);

    // Now, we can print out the IK solution (if found):
    if (found_ik)
    {
      sharedKinematic_state->copyJointGroupPositions(joint_model, joint_values);
      for(std::size_t i=0; i < joint_names.size(); ++i)
      {
	ROS_INFO("IK para Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }   
    }
    else
    {
      ROS_INFO("Did not find IK solution");
    }
    
    // Get the Jacobian
    // ^^^^^^^^^^^^^^^^
    // We can also get the Jacobian from the :moveit_core:`RobotState`.
    Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
    Eigen::MatrixXd jacobian;
    sharedKinematic_state->getJacobian(joint_model, sharedKinematic_state->getLinkModel(joint_model->getLinkModelNames().back()),
				  reference_point_position,
				  jacobian);
      ROS_INFO_STREAM("Jacobian: \n" << jacobian);
}



/**
* Joint position callback: j4
*/
void _j4Callback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
{
  const double vSet_point= (double) msg->set_point;
  const std::vector<std::string> &joint_names = joint_model->getJointModelNames();
  //sharedKinematic_state->setJointPositions(joint_names[0],&vSet_point);
  
  // j3 usado por th, ff, mf, rf y lf.  * Cambia el modo de obtener los nombres de las articulaciones, porque el numero de las articulaciones son distintos
  if((*tip_name == "lftip") || (*tip_name == "thtip"))
    sharedKinematic_state->setJointPositions(joint_names[1],&vSet_point);
  else
    sharedKinematic_state->setJointPositions(joint_names[0],&vSet_point);
}


/**
* Joint position callback: j5
*/
void _j5Callback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
{
  const double vSet_point= (double) msg->set_point;
  const std::vector<std::string> &joint_names = joint_model->getJointModelNames();
  // j5 solo usado por lf, th
  sharedKinematic_state->setJointPositions(joint_names[0],&vSet_point);
}

/**

// FIRST FINGER

void ff_j0Callback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
{
    const double vSet_point= (double) msg->set_point;
    const std::vector<std::string> &joint_names = ff_joint_model_group->getJointModelNames();
    sharedKinematic_state->setJointPositions(joint_names[3],&vSet_point);
    sharedKinematic_state->setJointPositions(joint_names[2],&vSet_point);


    // Mostrar valor joint
    std::vector<double> joint_values;
    sharedKinematic_state->copyJointGroupPositions(ff_joint_model_group, joint_values);
    for(std::size_t i = 0; i < joint_names.size(); ++i)
      {
	ROS_INFO("Joint state - %s: %f", joint_names[i].c_str(), joint_values[i]);
      }

    
    const Eigen::Affine3d &end_effector_state = sharedKinematic_state->getGlobalLinkTransform("fftip");

    // Print end-effector pose. Remember that this is in the model frame 
    ROS_INFO(" \n End effector state \n");
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation());
    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation());

    // Inverse Kinematics
    // ^^^^^^^^^^^^^^^^^^
    bool found_ik = sharedKinematic_state->setFromIK(ff_joint_model_group, end_effector_state, 10, 0.1);

    // Now, we can print out the IK solution (if found):
    if (found_ik)
    {
      sharedKinematic_state->copyJointGroupPositions(ff_joint_model_group, joint_values);
      for(std::size_t i=0; i < joint_names.size(); ++i)
      {
	ROS_INFO("IK para Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }   
    }
    else
    {
      ROS_INFO("Did not find IK solution");
    }
    
    // Get the Jacobian
    // ^^^^^^^^^^^^^^^^
    // We can also get the Jacobian from the :moveit_core:`RobotState`.
    Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
    Eigen::MatrixXd jacobian;
    sharedKinematic_state->getJacobian(ff_joint_model_group, sharedKinematic_state->getLinkModel(ff_joint_model_group->getLinkModelNames().back()),
				  reference_point_position,
				  jacobian);
      ROS_INFO_STREAM("Jacobian: \n" << jacobian);
      
      
}
*/



/**
 * Node main
 * 
 */
int main(int argc, char **argv)
{
  ros::init (argc, argv, "shadow_get_ik");
  ros::NodeHandle nh;  
  
  //  definir subscribers
  ros::Subscriber subs_Th_J1;
  ros::Subscriber subs_Th_J2;
  ros::Subscriber subs_Th_J3;
  ros::Subscriber subs_Th_J4;
  ros::Subscriber subs_Th_J5;
  
  ros::Subscriber subs_Ff_J0;
  ros::Subscriber subs_Ff_J3;
  ros::Subscriber subs_Ff_J4;
  
  ros::Subscriber subs_Mf_J0;
  ros::Subscriber subs_Mf_J3;
  ros::Subscriber subs_Mf_J4;
  
  ros::Subscriber subs_Rf_J0;
  ros::Subscriber subs_Rf_J3;
  ros::Subscriber subs_Rf_J4;
  
  ros::Subscriber subs_Lf_J0;
  ros::Subscriber subs_Lf_J3;
  ros::Subscriber subs_Lf_J4;
  ros::Subscriber subs_Lf_J5;
  // >> PARAMETRO DEDO A UTILIZAR
  // th -> 1; ff -> 2; mf -> 3; rf -> 4; lf ->5;
  // >> Obtener numero de dedos que se usan en el experimento -> rosparam
  
  ROS_INFO("Subscribers iniciados");
  
  
  int finger;
  nh.getParam("/get_ik/dedo", finger);
  ROS_INFO("Dedo a usar : %d", finger);
  tip_name = new std::string;

  ROS_INFO("Dedo a usar: %d", finger);
  
  // Definir callbacks para cada joint publicada
  switch(finger){
    case 1:
          subs_Th_J1 = nh.subscribe("/sh_thj1_position_controller/state",1000, _j1Callback);
	         subs_Th_J2 = nh.subscribe("/sh_thj2_position_controller/state",1000, _j2Callback);
	         subs_Th_J3 = nh.subscribe("/sh_thj3_position_controller/state",1000, _j3Callback);
	         subs_Th_J4 = nh.subscribe("/sh_thj4_position_controller/state",1000, _j4Callback);
	         subs_Th_J5 = nh.subscribe("/sh_thj5_position_controller/state",1000, _j5Callback);
	break;
	
    case 2:
        	subs_Ff_J0 = nh.subscribe("/sh_ffj0_position_controller/state", 1000, _j0Callback);
        	subs_Ff_J3 = nh.subscribe("/sh_ffj3_position_controller/state", 1000, _j3Callback);
        	subs_Ff_J4 = nh.subscribe("/sh_ffj4_position_controller/state", 1000, _j4Callback);
	break;
	
    case 3:  
        	subs_Mf_J0 = nh.subscribe("/sh_mfj0_position_controller/state", 1000, _j0Callback);
        	subs_Mf_J3 = nh.subscribe("/sh_mfj3_position_controller/state", 1000, _j3Callback);
        	subs_Mf_J4 = nh.subscribe("/sh_mfj4_position_controller/state", 1000, _j4Callback);
	break;
	  
    case 4:
        	subs_Rf_J0 = nh.subscribe("/sh_rfj0_position_controller/state", 1000, _j0Callback);
        	subs_Rf_J3 = nh.subscribe("/sh_rfj3_position_controller/state", 1000, _j3Callback);
        	subs_Rf_J4 = nh.subscribe("/sh_rfj4_position_controller/state", 1000, _j4Callback);
	break;
	
    case 5:
        	subs_Lf_J0 = nh.subscribe("/sh_lfj0_position_controller/state", 1000, _j0Callback);
        	subs_Lf_J3 = nh.subscribe("/sh_lfj3_position_controller/state", 1000, _j3Callback);
        	subs_Lf_J4 = nh.subscribe("/sh_lfj4_position_controller/state", 1000, _j4Callback);
        	subs_Lf_J5 = nh.subscribe("/sh_lfj5_position_controller/state", 1000, _j5Callback);
	break;
  }

  
  
  // Cargar modelo
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  ROS_INFO("############################################################");
  sharedKinematic_model	= kinematic_model;

  // Definir kinematic state
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  sharedKinematic_state = kinematic_state;
  sharedKinematic_state->setToDefaultValues();
  
  // Definir joint groups
  /**
  ff_joint_model_group = kinematic_model->getJointModelGroup("first_finger");
  mf_joint_model_group = kinematic_model->getJointModelGroup("middle_finger");
  rf_joint_model_group = kinematic_model->getJointModelGroup("ring_finger");
  lf_joint_model_group = kinematic_model->getJointModelGroup("little_finger");
  th_joint_model_group = kinematic_model->getJointModelGroup("thumb");
  */
  
  
  // Definir callbacks para cada joint publicada
  switch(finger){
    case 1:
	joint_model = kinematic_model->getJointModelGroup("thumb");
	*tip_name = "thtip";
	break;
	
    case 2:
	joint_model = kinematic_model->getJointModelGroup("first_finger");
	*tip_name = "fftip";
	break;
	
    case 3:  
	joint_model = kinematic_model->getJointModelGroup("middle_finger");
	*tip_name = "mftip";
	break;
	  
    case 4:
	joint_model = kinematic_model->getJointModelGroup("ring_finger");
	*tip_name = "rftip";
	break;
	
    case 5:
	joint_model = kinematic_model->getJointModelGroup("little_finger");
	*tip_name = "lftip";
	break;
  }
   
  ros::spin();
  return 0;
}
