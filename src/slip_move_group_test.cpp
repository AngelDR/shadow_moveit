/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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
 *   * Neither the name of SRI International nor the names of its
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

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <pr2_mechanism_msgs/LoadController.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_controller_manager/controller_manager.h>

#include "std_msgs/Float64.h"
#include "tekscan_client/GetPressureMap.h"

#include <iostream>
#include <fstream>

using namespace std;


class trajectory_state{
  int pos_j0;
  int pos_j1;
  int pos_j2;
  int force;
  int centroid;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_test");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  
  // >> SERVICIOS PARA CAMBIAR CONTROLADORES
  // >> pr2_controller_manager -> usado por paquetes Shadow para cargar / arrancar / cambiar controladores
  ros::ServiceClient load_controller_client = node_handle.serviceClient<pr2_mechanism_msgs::LoadController>("pr2_controller_manager/load_controller");
  ros::ServiceClient switch_controller_client = node_handle.serviceClient<pr2_mechanism_msgs::SwitchController>("pr2_controller_manager/switch_controller");
  
  
  // >> PARAMETROS DE LOS EXPERIMENTOS
  // >> Obtener numero de dedos que se usan en el experimento -> rosparam
  int num_fingers_exp;
  if (node_handle.getParam("/experimento/numero_dedos", num_fingers_exp))
  {
    ROS_INFO("Numero de dedos para el experimento : %d", num_fingers_exp); 
  }
  
  
  // >> PRESSURE MAP SERVICE
  ros::ServiceClient pressure_client = node_handle.serviceClient<tekscan_client::GetPressureMap>("GetPressureMap");
  tekscan_client::GetPressureMap srv_pressure;
  srv_pressure.request.num = 0;
  
  // >> PUBLISHERS PARA CONTROLADORES DE POSICION
  ros::Publisher pos_ff_j0_pub = node_handle.advertise<std_msgs::Float64>("/sh_ffj0_position_controller/command", 1000);
  ros::Publisher pos_ff_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_ffj3_position_controller/command", 1000);
  ros::Publisher pos_ff_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_ffj4_position_controller/command", 1000);
  
  ros::Publisher pos_mf_j0_pub = node_handle.advertise<std_msgs::Float64>("/sh_mfj0_position_controller/command", 1000);
  ros::Publisher pos_mf_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_mfj3_position_controller/command", 1000);
  ros::Publisher pos_mf_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_mfj4_position_controller/command", 1000);
  
  ros::Publisher pos_th_j1_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj1_position_controller/command", 1000);
  ros::Publisher pos_th_j2_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj2_position_controller/command", 1000);
  ros::Publisher pos_th_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj3_position_controller/command", 1000);
  ros::Publisher pos_th_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj4_position_controller/command", 1000);
  ros::Publisher pos_th_j5_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj5_position_controller/command", 1000);
  
  ros::Publisher pos_wr_j1_pub = node_handle.advertise<std_msgs::Float64>("/sh_wrj1_position_controller/command", 1000);
  ros::Publisher pos_wr_j2_pub = node_handle.advertise<std_msgs::Float64>("/sh_wrj2_position_controller/command", 1000);
  
  
  // >> PUBLISHERS PARA CONTROLADORES DE FUERZA
  ros::Publisher eff_ff_j0_pub = node_handle.advertise<std_msgs::Float64>("/sh_ffj0_effort_controller/command", 1000);
  ros::Publisher eff_ff_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_ffj3_effort_controller/command", 1000);
  ros::Publisher eff_ff_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_ffj4_effort_controller/command", 1000);
  
  ros::Publisher eff_mf_j0_pub = node_handle.advertise<std_msgs::Float64>("/sh_mfj0_effort_controller/command", 1000);
  ros::Publisher eff_mf_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_mfj3_effort_controller/command", 1000);
  ros::Publisher eff_mf_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_mfj4_effort_controller/command", 1000);
  
  ros::Publisher eff_th_j1_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj1_effort_controller/command", 1000);
  ros::Publisher eff_th_j2_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj2_effort_controller/command", 1000);
  ros::Publisher eff_th_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj3_effort_controller/command", 1000);
  ros::Publisher eff_th_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj4_effort_controller/command", 1000);
  ros::Publisher eff_th_j5_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj5_effort_controller/command", 1000);
  
  ros::Publisher eff_wr_j1_pub = node_handle.advertise<std_msgs::Float64>("/sh_wrj1_effort_controller/command", 1000);
  ros::Publisher eff_wr_j2_pub = node_handle.advertise<std_msgs::Float64>("/sh_wrj2_effort_controller/command", 1000);

  
  /* This sleep is ONLY to allow Rviz to come up */
  sleep(20.0);
  bool success;
  
  
  // >> DEFINIR OBJETOS DE MOVEIT -> PLAN, GRUPOS, ESCENA
  moveit::planning_interface::MoveGroup::Plan my_plan;
  moveit::planning_interface::MoveGroup group_first_finger("first_finger");
  moveit::planning_interface::MoveGroup group_middle_finger("middle_finger");
  moveit::planning_interface::MoveGroup group_thumb("thumb");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  
  /**
  // >> MOSTRAR POR PANTALLA CONFIGURACIONES INICIALES:
  ROS_INFO("Reference frame for first finger: %s", group_first_finger.getPlanningFrame().c_str());
  ROS_INFO("Reference frame - end effector link: %s", group_first_finger.getEndEffectorLink().c_str());
  ROS_INFO("Reference frame for thumb finger: %s", group_thumb.getPlanningFrame().c_str());
  ROS_INFO("Reference frame - end effector link: %s", group_thumb.getEndEffectorLink().c_str());
  geometry_msgs::PoseStamped current_pose_ff = group_first_finger.getCurrentPose();
  geometry_msgs::PoseStamped current_pose_th = group_thumb.getCurrentPose();
  ROS_INFO("Pose actual fftip::  x: %f, y: %f, z: %f , Or_x: %f ,Or_y: %f, Or_z: %f, Or_w: %f", current_pose_ff.pose.position.x, current_pose_ff.pose.position.y, current_pose_ff.pose.position.z,
	   current_pose_ff.pose.orientation.x,current_pose_ff.pose.orientation.y,current_pose_ff.pose.orientation.z,current_pose_ff.pose.orientation.w);
  ROS_INFO("Pose actual thtip::  x: %f, y: %f, z: %f , Or_x: %f ,Or_y: %f, Or_z: %f, Or_w: %f", current_pose_th.pose.position.x, current_pose_th.pose.position.y, current_pose_th.pose.position.z,
	   current_pose_th.pose.orientation.x,current_pose_th.pose.orientation.y,current_pose_th.pose.orientation.z,current_pose_th.pose.orientation.w);
  
  */
  
  

  
  // >> IR A POSICION INICIAL CON POSICIONES ARTICULARES 
  // >> TODO : pasar a param o fichero

    
  // >> Situar wrist - hardcoded
  pos_wr_j1_pub.publish(-0.5934);
  sleep(0.1);
  pos_wr_j2_pub.publish(-0.0174);
  sleep(0.1);
  

  // >> Situar first-finger (se actualiza tambien el ModelGroup de MoveIt)
  std::vector<double> group_variable_values_ff;
  group_first_finger.getCurrentState()->copyJointGroupPositions(group_first_finger.getCurrentState()->getRobotModel()->getJointModelGroup(group_first_finger.getName()), group_variable_values_ff);
  for(std::size_t i = 0; i < group_variable_values_ff.size(); ++i)
  {
      ROS_INFO("Joint %d: %f", i, group_variable_values_ff[i]);
  }
  group_variable_values_ff[0] = -0.052;
  group_variable_values_ff[1] = 0.000;
  group_variable_values_ff[2] = 0.575;  
  group_variable_values_ff[3] = 0.575;
  
  pos_ff_j4_pub.publish(group_variable_values_ff[0]);
  sleep(0.1);
  pos_ff_j3_pub.publish(group_variable_values_ff[1]);
  sleep(0.1);
  pos_ff_j0_pub.publish(group_variable_values_ff[2]);
  sleep(0.1);

  group_first_finger.setJointValueTarget(group_variable_values_ff);
  //success = group_first_finger.plan(my_plan);
  //success = group_first_finger.execute(my_plan);
  // group_first_finger.move()  = plan & execute
  //group_first_finger.move();
  //ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(1.0);
  group_first_finger.getCurrentState()->copyJointGroupPositions(group_first_finger.getCurrentState()->getRobotModel()->getJointModelGroup(group_first_finger.getName()), group_variable_values_ff);
  for(std::size_t i = 0; i < group_variable_values_ff.size(); ++i)
  {
    ROS_INFO("Joint %d: %f", i, group_variable_values_ff[i]);
  }
  sleep(2.0);
  
  

  // >> SITUAR MIDDLE-FINGER (se actualiza tambien el ModelGroup de MoveIt)
  std::vector<double> group_variable_values_mf;
  group_middle_finger.getCurrentState()->copyJointGroupPositions(group_middle_finger.getCurrentState()->getRobotModel()->getJointModelGroup(group_middle_finger.getName()), group_variable_values_mf);
  for(std::size_t i = 0; i < group_variable_values_mf.size(); ++i)
  {
    ROS_INFO("Joint %d: %f", i, group_variable_values_mf[i]);
  }
  
  // situar solo si el experimento usa más de dos dedos
  if(num_fingers_exp > 2)
  {
      group_variable_values_mf[0] = -0.052;
      group_variable_values_mf[1] = 0.000;
      group_variable_values_mf[2] = 0.575;  
      group_variable_values_mf[3] = 0.575;
      
      pos_mf_j4_pub.publish(group_variable_values_mf[0]);
      sleep(0.1);
      pos_mf_j3_pub.publish(group_variable_values_mf[1]);
      sleep(0.1);
      pos_mf_j0_pub.publish(group_variable_values_mf[2]);
      sleep(0.1);

      group_middle_finger.setJointValueTarget(group_variable_values_mf);
      //success = group_first_finger.plan(my_plan);
      //success = group_first_finger.execute(my_plan);
      // group_first_finger.move()  = plan & execute
      //group_first_finger.move();
      //ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
      /* Sleep to give Rviz time to visualize the plan. */
      sleep(1.0);
      group_middle_finger.getCurrentState()->copyJointGroupPositions(group_middle_finger.getCurrentState()->getRobotModel()->getJointModelGroup(group_middle_finger.getName()), group_variable_values_mf);
      for(std::size_t i = 0; i < group_variable_values_mf.size(); ++i)
      {
	ROS_INFO("Joint %d: %f", i, group_variable_values_mf[i]);
      }
      sleep(2.0);
  
  }
  
  
  // >> SITUAR THUMB
  std::vector<double> group_variable_values_thumb;
  group_thumb.getCurrentState()->copyJointGroupPositions(group_thumb.getCurrentState()->getRobotModel()->getJointModelGroup(group_thumb.getName()), group_variable_values_thumb);
  group_variable_values_thumb[0] = 0.802;
  if(num_fingers_exp==2)
    group_variable_values_thumb[1] = 0.855;  
  if(num_fingers_exp==3)
    group_variable_values_thumb[1] = 1.22;  // para 3 dedos
  group_variable_values_thumb[2] = 0.157;
  group_variable_values_thumb[3] = -0.698;
  group_variable_values_thumb[4] = 0.261;
  
  pos_th_j5_pub.publish(group_variable_values_thumb[0]);
  sleep(0.1);
  pos_th_j4_pub.publish(group_variable_values_thumb[1]);
  sleep(0.1);
  pos_th_j3_pub.publish(group_variable_values_thumb[2]);
  sleep(0.1);
  pos_th_j2_pub.publish(group_variable_values_thumb[3]);
  sleep(0.1);
  pos_th_j1_pub.publish(group_variable_values_thumb[4]);
  sleep(0.1);
  group_thumb.setJointValueTarget(group_variable_values_thumb);
  //success = group_thumb.plan(my_plan);
  //success = group_thumb.execute(my_plan);
  //ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED"); 
  // Sleep to give Rviz time to visualize the plan.
  sleep(2.0);
  
  
  
  // >> Situada la mano, colocar en posicion...  y continuar
  std::cout << "Esperando...";
  std::cin.get();
 
  
  // >> Algoritmo de manipulacion:
  // >> Primer caso: cierre de espuma
  
  
  double wrj1_position = -0.5934;
  double position_step = 0.05;
  double position_step_tip = 0.03;
  
  
  
  /**
   * 
   * REAJUSTE DE POSICION
   * 
   */
  // >> while presion menor que un umbral de seguridad
  double min_pressure_threshold = 2.2;
  ofstream myfile;
  myfile.open ("/home/aurova/Desktop/pruebas/resultados/pressure_data.txt");
  if(myfile.is_open())
    ROS_INFO("Archivo abierto");
  
  int iteration = 0;

  do
  {
    if (pressure_client.call(srv_pressure))
    {
      ROS_INFO("/Reajuste - posición");

	  group_variable_values_ff[1] += position_step;
	  group_variable_values_ff[2] += position_step_tip; 
	  group_variable_values_mf[1] += position_step;
	  group_variable_values_mf[2] += position_step_tip; 
	  wrj1_position += -0.01;
	  
	  pos_ff_j0_pub.publish(group_variable_values_ff[2]);
	  //sleep(0.1);
	  pos_ff_j3_pub.publish(group_variable_values_ff[1]);
	  //sleep(0.1);
	  if (num_fingers_exp > 2)
	  {
	    pos_mf_j0_pub.publish(group_variable_values_mf[2]);
	    //sleep(0.1);
	    pos_mf_j3_pub.publish(group_variable_values_mf[1]);
	    //sleep(0.1);
	  }
	  pos_wr_j1_pub.publish(wrj1_position);
	  sleep(1.0); 
      
    }
    else
    {
      ROS_ERROR("Failed to call service pressure");
      return 1;
    }
     
    myfile << srv_pressure.response.applied_force[0];
    myfile << " ";
    myfile << srv_pressure.response.applied_force[1];
    myfile << " ";
    myfile << srv_pressure.response.applied_force[2];
    myfile << " ";
    myfile << iteration;
    myfile << "\n";
    iteration++;
    
  }while((srv_pressure.response.applied_force[0] <  min_pressure_threshold) && (srv_pressure.response.applied_force[1] <  min_pressure_threshold));
	    //&& (srv_pressure.response.applied_force[2] <  min_pressure_threshold));
  
  myfile.close();

  
  
  
  /**
   * PRIMERA FORMA: pruebas, de forma directa
   * 
  while( (group_variable_values_ff[1] < 0.85) || (group_variable_values_ff[2] < 1.4))
  {
    
    group_variable_values_ff[1] += position_step;
    group_variable_values_ff[2] += position_step; 
    group_variable_values_mf[1] += position_step;
    group_variable_values_mf[2] += position_step; 
    wrj1_position += -0.005;
    
    pos_ff_j0_pub.publish(group_variable_values_ff[2]);
    sleep(0.1);
    pos_ff_j3_pub.publish(group_variable_values_ff[1]);
    sleep(0.1);
    pos_mf_j0_pub.publish(group_variable_values_mf[2]);
    sleep(0.1);
    pos_mf_j3_pub.publish(group_variable_values_mf[1]);
    sleep(0.1);
    pos_wr_j1_pub.publish(wrj1_position);

    sleep(1.0); 
  }
  
  */
  
  
  sleep(2.0);
  
  
  /**
   * 
   * REAJUSTE DE FUERZA
   * 
   */
  
  // >> Cambiar a control de fuerza -> para reajuste de fuerza
  pr2_mechanism_msgs::LoadController srv_load;
  pr2_mechanism_msgs::SwitchController srv_switch;
  
  
  // Cargar controladores effort
  std::string list_controllers_to_load[6] = {"sh_ffj0_effort_controller","sh_ffj3_effort_controller","sh_thj1_effort_controller","sh_thj2_effort_controller", 
    "sh_mfj0_effort_controller","sh_mfj3_effort_controller" };
  std::vector<std::string> list_controllers_to_start;
  std::vector<std::string> list_controllers_to_stop;
  list_controllers_to_stop.push_back("sh_ffj0_position_controller");
  list_controllers_to_stop.push_back("sh_ffj3_position_controller");
  list_controllers_to_stop.push_back("sh_mfj0_position_controller");
  list_controllers_to_stop.push_back("sh_mfj3_position_controller");
  list_controllers_to_stop.push_back("sh_thj1_position_controller");
  list_controllers_to_stop.push_back("sh_thj2_position_controller");

  
  for(int i = 0; i<4; i++)
  {
    list_controllers_to_start.push_back(list_controllers_to_load[i]);
    srv_load.request.name = list_controllers_to_load[i];
    if (load_controller_client.call(srv_load))
    {
      if(srv_load.response.ok) {ROS_INFO("Cargados controladores de fuerza");}
    }
    else
    {
      ROS_ERROR("Failed to call service load Controller");
    }
  }
  

  
  // Switch controllers: parar controladores de posicion y arrancar controladores de fuerza

  srv_switch.request.start_controllers = list_controllers_to_start;
  srv_switch.request.stop_controllers = list_controllers_to_stop;
  srv_switch.request.strictness = 1;
  if (switch_controller_client.call(srv_switch))
  {
    if(srv_switch.response.ok) ROS_INFO("Activados controladores de fuerza");
    else ROS_INFO("NOT OK");
  }
  else
  {
    ROS_ERROR("Failed to call service switch Controller");
  }
  
  
  // Enviar comandos de fuerza  -> Reajuste de fuerza
  // >> while presion menor que un umbral de seguridad
    double max_pressure_threshold = 7.0;
    int it = 1;
    // Reajuste first - finger
    do
    {
      if (pressure_client.call(srv_pressure))
      {
	ROS_INFO("/Reajuste - fuerza - first finger");

	    //eff_ff_j0_pub.publish(400.0);
	    //sleep(0.1);
	    if(((max_pressure_threshold - srv_pressure.response.applied_force[1]) < 5.0) && (srv_pressure.response.applied_force[1] < max_pressure_threshold))
	      eff_ff_j3_pub.publish(900.0);
	    else	
	      break;
	    it++;
	    sleep(1.5); 
	
      }
      else
      {
	ROS_ERROR("Failed to call service pressure");
	return 1;
      }
      
    }while((srv_pressure.response.applied_force[1] <  max_pressure_threshold) && (it < 4));
    
    
    // Reajuste thumb - finger
    it = 1;
    do
    {
      if (pressure_client.call(srv_pressure))
      {
	ROS_INFO("/Reajuste - fuerza - thumb");

	    //eff_ff_j0_pub.publish(400.0);
	    //sleep(0.1);
	    if(((max_pressure_threshold - srv_pressure.response.applied_force[0]) < 5.0) && (srv_pressure.response.applied_force[0] < max_pressure_threshold))
		eff_th_j5_pub.publish(800.0);
	    else
		break;
	    it++;
	    sleep(0.8); 
	
      }
      else
      {
	ROS_ERROR("Failed to call service pressure");
	return 1;
      }
      
    }while((srv_pressure.response.applied_force[0] <  max_pressure_threshold) && (it < 4));
    
    
    
  // Desactivar controladores de fuerza. Restablecer controladores de posicion
     // Switch controllers: parar controladores de posicion y arrancar controladores de fuerza

  srv_switch.request.start_controllers = list_controllers_to_stop;
  srv_switch.request.stop_controllers = list_controllers_to_start;
  srv_switch.request.strictness = 1;
  if (switch_controller_client.call(srv_switch))
  {
    if(srv_switch.response.ok) ROS_INFO("Activados controladores de posicion");
    else ROS_INFO("NOT OK");
  }
  else
  {
    ROS_ERROR("Failed to call service switch Controller");
  }
    
  // >> Finalizado algoritmo
  std::cout << "Algoritmo finalizado";
  std::cin.get();
 
  //ros::shutdown();  
  return 0;
}