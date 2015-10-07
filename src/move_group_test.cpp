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
  
  ros::Publisher pos_rf_j0_pub = node_handle.advertise<std_msgs::Float64>("/sh_rfj0_position_controller/command", 1000);
  ros::Publisher pos_rf_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_rfj3_position_controller/command", 1000);
  ros::Publisher pos_rf_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_rfj4_position_controller/command", 1000);
  
  ros::Publisher pos_lf_j0_pub = node_handle.advertise<std_msgs::Float64>("/sh_lfj0_position_controller/command", 1000);
  ros::Publisher pos_lf_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_lfj3_position_controller/command", 1000);
  ros::Publisher pos_lf_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_lfj4_position_controller/command", 1000);
  ros::Publisher pos_lf_j5_pub = node_handle.advertise<std_msgs::Float64>("/sh_lfj5_position_controller/command", 1000);

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
  
  ros::Publisher eff_rf_j0_pub = node_handle.advertise<std_msgs::Float64>("/sh_rfj0_effort_controller/command", 1000);
  ros::Publisher eff_rf_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_rfj3_effort_controller/command", 1000);
  ros::Publisher eff_rf_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_rfj4_effort_controller/command", 1000);
  
  ros::Publisher eff_lf_j0_pub = node_handle.advertise<std_msgs::Float64>("/sh_lfj0_effort_controller/command", 1000);
  ros::Publisher eff_lf_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_lfj3_effort_controller/command", 1000);
  ros::Publisher eff_lf_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_lfj4_effort_controller/command", 1000);
  ros::Publisher eff_lf_j5_pub = node_handle.advertise<std_msgs::Float64>("/sh_lfj5_effort_controller/command", 1000);
  
  ros::Publisher eff_th_j1_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj1_effort_controller/command", 1000);
  ros::Publisher eff_th_j2_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj2_effort_controller/command", 1000);
  ros::Publisher eff_th_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj3_effort_controller/command", 1000);
  ros::Publisher eff_th_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj4_effort_controller/command", 1000);
  ros::Publisher eff_th_j5_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj5_effort_controller/command", 1000);
  
  ros::Publisher eff_wr_j1_pub = node_handle.advertise<std_msgs::Float64>("/sh_wrj1_effort_controller/command", 1000);
  ros::Publisher eff_wr_j2_pub = node_handle.advertise<std_msgs::Float64>("/sh_wrj2_effort_controller/command", 1000);

  bool success;
  
  /**
   * PRUEBA BUCLE
   * */
  //bool exit_ = false; 
  do{ 
    // >> PARAMETROS DE LOS EXPERIMENTOS
    // >> Obtener numero de dedos que se usan en el experimento -> rosparam
    int num_fingers_exp;
    if (node_handle.getParam("/grasp_reconfiguration/number_fingers", num_fingers_exp))
    {
      ROS_INFO("Numero de dedos para el experimento : %d", num_fingers_exp); 
    }

    // >> IR A POSICION INICIAL CON POSICIONES ARTICULARES       
    // >> Situar wrist - param
    double wrj1_position, wrj2_position;
    if (node_handle.getParam("/grasp_reconfiguration/wrist_angle_x", wrj1_position))
    {
      ROS_INFO("Angulo de la muñeca : %f", wrj1_position); 
    }
    else
    {
      wrj1_position = 0.2094;
    }

    if (node_handle.getParam("/grasp_reconfiguration/wrist_angle_y", wrj2_position))
    {
      ROS_INFO("Angulo de la muñeca : %f", wrj2_position); 
    }
    else
    {
      wrj1_position = 0.0;
    }

    // >> Apertura entre dedos - param
    double knuckle_aperture;
    if (node_handle.getParam("/grasp_reconfiguration/knuckle_aperture", knuckle_aperture))
    {
      ROS_INFO("Apertura del agarre : %f", knuckle_aperture); 
    }
    else
    {
      knuckle_aperture = 0.0;
    }

    // >> Apertura entre dedos (horizontal)- param
    double hub_aperture;
    if (node_handle.getParam("/grasp_reconfiguration/hub_aperture", hub_aperture))
    {
      ROS_INFO("Apertura entre dedos : %f", hub_aperture); 
    }
    else
    {
      hub_aperture = 0.0;
    }

    // >> Apertura tips
    double middle_aperture;
    if (node_handle.getParam("/grasp_reconfiguration/middle_aperture", middle_aperture))
    {
      ROS_INFO("Apertura entre dedos : %f", middle_aperture); 
    }
    else
    {
      middle_aperture = 0.0;
    }

    // >> Position little - param
    double little_finger_position;
    if (node_handle.getParam("/grasp_reconfiguration/little_finger_position", little_finger_position))
    {
      ROS_INFO("Apertura del agarre : %f", little_finger_position); 
    }
    else
    {
      little_finger_position = 0.0;
    }

    // Situar wrist
    // pos_wr_j1_pub.publish(-0.2034); // (en Javistruz)
    // pos_wr_j1_pub.publish(-0.5934); // (en PA10)
    pos_wr_j1_pub.publish(wrj1_position);sleep(0.1);// (con brazo horizontal)
    pos_wr_j2_pub.publish(wrj2_position);sleep(0.1);

    // >> Situar first-finger (se actualiza tambien el ModelGroup de MoveIt)
    std::vector<double> group_variable_values_ff;
    group_variable_values_ff.push_back(hub_aperture);
    group_variable_values_ff.push_back(knuckle_aperture);
    group_variable_values_ff.push_back(middle_aperture);
    group_variable_values_ff.push_back(middle_aperture);
    
    pos_ff_j4_pub.publish(group_variable_values_ff[0]);sleep(0.1);
    pos_ff_j3_pub.publish(group_variable_values_ff[1]);sleep(0.1);
    pos_ff_j0_pub.publish(group_variable_values_ff[2]);sleep(0.1);
    sleep(0.5);
    
    // >> SITUAR MIDDLE-FINGER (se actualiza tambien el ModelGroup de MoveIt)
    std::vector<double> group_variable_values_mf;
    // situar solo si el experimento usa más de dos dedos
    if(num_fingers_exp > 2)
    {
      group_variable_values_mf.push_back(hub_aperture / 2);
      group_variable_values_mf.push_back(knuckle_aperture);
      group_variable_values_mf.push_back(middle_aperture);
      group_variable_values_mf.push_back(middle_aperture);
      
      pos_mf_j4_pub.publish(group_variable_values_mf[0]);sleep(0.1);
      pos_mf_j3_pub.publish(group_variable_values_mf[1]);sleep(0.1);
      pos_mf_j0_pub.publish(group_variable_values_mf[2]);sleep(0.1);
      sleep(0.5);
    }
    else
    {
      group_variable_values_mf.push_back(0.0);
      group_variable_values_mf.push_back(0.0);
      group_variable_values_mf.push_back(0.0);
      group_variable_values_mf.push_back(0.0);
      
      pos_mf_j4_pub.publish(group_variable_values_mf[0]);sleep(0.1);
      pos_mf_j3_pub.publish(group_variable_values_mf[1]);sleep(0.1);
      pos_mf_j0_pub.publish(group_variable_values_mf[2]);sleep(0.1);
    }
    
    // >> SITUAR RING-FINGER (se actualiza tambien el ModelGroup de MoveIt)
    std::vector<double> group_variable_values_rf;
    // situar solo si el experimento usa más de tres dedos
    if(num_fingers_exp > 3)
    {
      group_variable_values_rf.push_back(hub_aperture / 2);
      group_variable_values_rf.push_back(knuckle_aperture);
      group_variable_values_rf.push_back(middle_aperture);
      group_variable_values_rf.push_back(middle_aperture);
      
      pos_rf_j4_pub.publish(group_variable_values_rf[0]);sleep(0.1);
      pos_rf_j3_pub.publish(group_variable_values_rf[1]);sleep(0.1);
      pos_rf_j0_pub.publish(group_variable_values_rf[2]);sleep(0.1);
      sleep(0.5);   
    } 
    else
    {
      group_variable_values_rf.push_back(0.0);
      group_variable_values_rf.push_back(0.0);
      group_variable_values_rf.push_back(0.0);
      group_variable_values_rf.push_back(0.0);
      
      pos_rf_j4_pub.publish(group_variable_values_rf[0]);sleep(0.1);
      pos_rf_j3_pub.publish(group_variable_values_rf[1]);sleep(0.1);
      pos_rf_j0_pub.publish(group_variable_values_rf[2]);sleep(0.1);
      sleep(0.5);  

    }
    
    // >> SITUAR LITTLE-FINGER (se actualiza tambien el ModelGroup de MoveIt)
    std::vector<double> group_variable_values_lf;
    // situar solo si el experimento usa más de cuatro dedos
    if(num_fingers_exp > 4)
    {
      group_variable_values_lf.push_back(little_finger_position);
      group_variable_values_lf.push_back(hub_aperture);
      group_variable_values_lf.push_back(knuckle_aperture);
      group_variable_values_lf.push_back(middle_aperture);
      group_variable_values_lf.push_back(middle_aperture);
      
      pos_lf_j5_pub.publish(group_variable_values_lf[0]);sleep(0.1);
      pos_lf_j4_pub.publish(group_variable_values_lf[1]);sleep(0.1);
      pos_lf_j3_pub.publish(group_variable_values_lf[2]);sleep(0.1);
      pos_lf_j0_pub.publish(group_variable_values_lf[3]);sleep(0.1);
      sleep(0.5); 
    }
    else
    {
      group_variable_values_lf.push_back(0.0);
      group_variable_values_lf.push_back(0.0);
      group_variable_values_lf.push_back(0.0);
      group_variable_values_lf.push_back(0.0);
      group_variable_values_lf.push_back(0.0);
      
      pos_lf_j5_pub.publish(group_variable_values_lf[0]);sleep(0.1);
      pos_lf_j4_pub.publish(group_variable_values_lf[1]);sleep(0.1);
      pos_lf_j3_pub.publish(group_variable_values_lf[2]);sleep(0.1);
      pos_lf_j0_pub.publish(group_variable_values_lf[3]);sleep(0.1);
      sleep(0.5); 
    }
     
    
    // >> SITUAR THUMB
    std::vector<double> group_variable_values_thumb;
    group_variable_values_thumb.push_back(0.802);
    /**if(num_fingers_exp==2)
      group_variable_values_thumb.push_back(0.855);  
    if(num_fingers_exp==3)
      group_variable_values_thumb.push_back(1.22);  // para 3 dedos*/
    group_variable_values_thumb.push_back(1.22);
    group_variable_values_thumb.push_back(0.157);
    group_variable_values_thumb.push_back(-0.698);
    group_variable_values_thumb.push_back(0.261);  
    /**if(num_fingers_exp==4)
      group_variable_values_thumb.push_back(1.047);*/
    
    pos_th_j5_pub.publish(group_variable_values_thumb[0]);sleep(0.1);
    pos_th_j4_pub.publish(group_variable_values_thumb[1]);sleep(0.1);
    pos_th_j3_pub.publish(group_variable_values_thumb[2]);sleep(0.1);
    pos_th_j2_pub.publish(group_variable_values_thumb[3]);sleep(0.1);
    pos_th_j1_pub.publish(group_variable_values_thumb[4]);sleep(0.1);
    sleep(0.5); 

    double position_step, position_step_tip, position_step_lf, wrist_step, thumb_step;
    if (node_handle.getParam("/grasp_reconfiguration/position_step", position_step))
    {
      ROS_INFO("Position step : %f", position_step); 
    }
    else{
      position_step = 0.025;
    }

    if (node_handle.getParam("/grasp_reconfiguration/tip_position_step", position_step_tip))
    {
      ROS_INFO("Position step tip: %f", position_step_tip); 
    }
    else{
      position_step_tip = 0.025;
    }

    if (node_handle.getParam("/grasp_reconfiguration/wrist_step", wrist_step))
    {
      ROS_INFO("Wrist Position step: %f", wrist_step); 
    }
    else{
      wrist_step = 0.025;
    }

    if (node_handle.getParam("/grasp_reconfiguration/thumb_step", thumb_step))
    {
      ROS_INFO("Thumb Position step: %f", thumb_step); 
    }
    else{
      thumb_step = 0.002;
    }

    if (node_handle.getParam("/grasp_reconfiguration/little_step", position_step_lf))
    {
      ROS_INFO("Little Position step: %f", position_step_lf); 
    }
    else{
      position_step_lf = 0.0;
    }
    //position_step_lf = position_step;
    //position_step_tip_lf = position_step;
    
    
    /**
     * 
     * REAJUSTE DE POSICION
     * 
     */
    // >> while presion menor que un umbral de seguridad
    // Threshold Min de presion
    //double min_pressure_threshold = 2.0;//0.3;//2.2;
    double min_pressure_threshold;
    if (node_handle.getParam("/grasp_reconfiguration/min_pressure_threshold", min_pressure_threshold))
    {
      ROS_INFO("Umbral minimo de presion : %f", min_pressure_threshold); 
    }
    else{
      min_pressure_threshold = 2.0;
    }

    int iteration = 0;

    // >> Situada la mano, colocar en posicion...  y continuar
    std::cout << "Esperando...";
    std::cin.get();
    do
    {
      if (pressure_client.call(srv_pressure))
      {
        ROS_INFO("/Reajuste - posición");
        
        
      // Llamar a servicio de vision -> 
  	  // obtener posicions actuales
  	  // if altura_bbox_actual > altura_bbox_previa
  	 // if posiciones_dedos < altura_bbox_actual
  	  group_variable_values_ff[1] += position_step;
  	  group_variable_values_ff[2] += position_step_tip; 
  	  group_variable_values_mf[1] += position_step;
  	  group_variable_values_mf[2] += position_step_tip; 
  	  group_variable_values_rf[1] += position_step;
  	  group_variable_values_rf[2] += position_step_tip;
  	  group_variable_values_lf[0] += position_step_lf;
      group_variable_values_lf[2] += position_step;
  	  group_variable_values_lf[3] += position_step_tip;
      group_variable_values_thumb[4] += thumb_step;
  	  wrj1_position += wrist_step;//experimento -0.002;  // original -0.01

      pos_th_j1_pub.publish(group_variable_values_thumb[4]);
  	  pos_ff_j0_pub.publish(group_variable_values_ff[2]);
  	  pos_ff_j3_pub.publish(group_variable_values_ff[1]);
  	  if (num_fingers_exp > 2)
  	  {
  	    pos_mf_j0_pub.publish(group_variable_values_mf[2]);
  	    pos_mf_j3_pub.publish(group_variable_values_mf[1]);
  	  }
  	  
  	  if (num_fingers_exp > 3)
  	  {
  	    pos_rf_j0_pub.publish(group_variable_values_rf[2]);
  	    pos_rf_j3_pub.publish(group_variable_values_rf[1]);
  	  }
  	  
  	  if (num_fingers_exp > 4)
  	  {
  	    pos_lf_j0_pub.publish(group_variable_values_lf[3]);
  	    pos_lf_j3_pub.publish(group_variable_values_lf[2]);
        pos_lf_j5_pub.publish(group_variable_values_lf[0]);
  	  }
  	  pos_wr_j1_pub.publish(wrj1_position);
  	  sleep(0.2); // sleep 0.2 
      }
      else
      {
        ROS_ERROR("Failed to call service pressure");
        return 1;
      }
       
    }while((srv_pressure.response.applied_force[0] <  min_pressure_threshold) && (srv_pressure.response.applied_force[1] <  min_pressure_threshold)
  	    && (srv_pressure.response.applied_force[2] <  min_pressure_threshold));
    sleep(0.4);  


    /**
     * 
     * REAJUSTE DE FUERZA
     * 
     */
    // !!!!!  rfj3 / lfj3 / lfj0 / thj1-> estan calibrados al reves 900 - (-900)
    // >> Cambiar a control de fuerza -> para reajuste de fuerza
    pr2_mechanism_msgs::LoadController srv_load;
    pr2_mechanism_msgs::SwitchController srv_switch;
    
    // Cargar controladores effort
    std::string list_controllers_to_load[10] = {"sh_ffj0_effort_controller","sh_ffj3_effort_controller","sh_thj1_effort_controller","sh_thj2_effort_controller", 
      "sh_mfj0_effort_controller","sh_mfj3_effort_controller","sh_rfj0_effort_controller","sh_rfj3_effort_controller","sh_lfj0_effort_controller","sh_lfj3_effort_controller" };
    std::vector<std::string> list_controllers_to_start;
    std::vector<std::string> list_controllers_to_stop;
    list_controllers_to_stop.push_back("sh_ffj0_position_controller");
    list_controllers_to_stop.push_back("sh_ffj3_position_controller");
    list_controllers_to_stop.push_back("sh_mfj0_position_controller");
    list_controllers_to_stop.push_back("sh_mfj3_position_controller");
    list_controllers_to_stop.push_back("sh_rfj0_position_controller");
    list_controllers_to_stop.push_back("sh_rfj3_position_controller");
    list_controllers_to_stop.push_back("sh_thj1_position_controller");
    list_controllers_to_stop.push_back("sh_thj2_position_controller");
    list_controllers_to_stop.push_back("sh_lfj0_position_controller");
    list_controllers_to_stop.push_back("sh_lfj3_position_controller");

    for(int i = 0; i<10; i++)
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
    sleep(3.0);
    
    // Enviar comandos de fuerza  -> Reajuste de fuerza
    // >> while presion menor que un umbral de seguridad
    //double max_pressure_threshold = 5.0; // 1; //5.0;
    double max_pressure_threshold;
    if (node_handle.getParam("/grasp_reconfiguration/max_pressure_threshold", max_pressure_threshold))
    {
      ROS_INFO("Umbral maximo de presion : %f", max_pressure_threshold); 
    }
    else{
      max_pressure_threshold = 5.0;
    }

    double effort = 300.0;  // -> máximo=900.0   
    // Reajuste thumb - finger
    do
    {
      if (pressure_client.call(srv_pressure))
      {
    	  //if((min_pressure_threshold < srv_pressure.response.applied_force[0]) && (srv_pressure.response.applied_force[0] < max_pressure_threshold))
        if((srv_pressure.response.applied_force[0] > 0.5) && (srv_pressure.response.applied_force[0] < max_pressure_threshold))
    	  {
    	      eff_th_j1_pub.publish(-effort);
    	      eff_th_j2_pub.publish(effort);
    	      ROS_INFO("/Reajuste - fuerza - thumb: %f", effort);
    	  }
    	  else
        {
            ROS_INFO("/Reajuste - fuerza - thumb: no");
    	      break;
        }
    	  //it++;
    	  effort += 100.0;
    	  sleep(0.1);  
      }
      else
      {
        ROS_ERROR("Failed to call service pressure");
        return 1;
      }
    }while(effort <= 600.0);
    
    
    // Reajuste first - finger
    effort = 300.0;
    do
    {
      if (pressure_client.call(srv_pressure))
      {
    	  //if((min_pressure_threshold < srv_pressure.response.applied_force[1]) && (srv_pressure.response.applied_force[1] < max_pressure_threshold))
        if((srv_pressure.response.applied_force[1] > 0.5) && (srv_pressure.response.applied_force[1] < max_pressure_threshold))
    	  {
    	    eff_ff_j3_pub.publish(effort);
    	    eff_ff_j0_pub.publish(effort);
    	    ROS_INFO("/Reajuste - fuerza - first finger: %f", effort);
    	  }
        else
        {
            ROS_INFO("/Reajuste - fuerza - first: nulo");
            break;
        }
    	  //it++;
    	  effort += 100.0;
    	  sleep(0.1); 
      }
      else
      {
        ROS_ERROR("Failed to call service pressure");
        return 1;
      }  
    }while(effort <= 600.0);
      
          
    // Reajuste middle - finger
    effort = 300.0;
    do
    {
      if (pressure_client.call(srv_pressure))
      {
    	  //if((min_pressure_threshold < srv_pressure.response.applied_force[2]) && (srv_pressure.response.applied_force[2] < max_pressure_threshold))
        if((srv_pressure.response.applied_force[2] > 0.5) && (srv_pressure.response.applied_force[2] < max_pressure_threshold))
    	  {
    	      eff_mf_j0_pub.publish(effort);
    	      eff_mf_j3_pub.publish(effort);
    	      ROS_INFO("/Reajuste - fuerza - middle finger: %f", effort);
    	  }
        else
        {
            ROS_INFO("/Reajuste - fuerza - middle: nulo");
            break;
        }
    	  //it++;
    	  effort+=100.0;
    	  sleep(0.1); 
      }
      else
      {
        ROS_ERROR("Failed to call service pressure");
        return 1;
      }
    }while(effort <= 600.0);
    
    
    // Reajuste ring - finger
    effort = 300.0;
    do
    {
      if (pressure_client.call(srv_pressure))
      {
    	  if((srv_pressure.response.applied_force[3] > 0.5) && (srv_pressure.response.applied_force[3] < max_pressure_threshold))
    	  {
    	      eff_rf_j0_pub.publish(effort);
    	      eff_rf_j3_pub.publish(-effort);
    	      ROS_INFO("/Reajuste - fuerza - ring finger: %f", effort);
    	  }
        else
        {
            ROS_INFO("/Reajuste - fuerza - ring: nulo");
            break;
        }
    	  //it++;
    	  effort += 100.0;
    	  sleep(0.1); 
      }
      else
      {
        ROS_ERROR("Failed to call service pressure");
        return 1;
      }   
    }while(effort <= 600.0);
    
      
    // Reajuste little - finger
    effort = 300.0;
    do
    {
      if (pressure_client.call(srv_pressure))
      {
    	  if((srv_pressure.response.applied_force[4] > 0.5) && (srv_pressure.response.applied_force[4] < max_pressure_threshold))
    	  {
    	      eff_lf_j0_pub.publish(-effort);
    	      eff_lf_j3_pub.publish(-effort);
    	      ROS_INFO("/Reajuste - fuerza - little finger: %f", effort);
    	  }
        else
        {
            ROS_INFO("/Reajuste - fuerza - little: nulo");
            break;
        }
    	  //it++;
    	  effort += 100.0;
    	  sleep(1.0); 
      }
      else
      {
        ROS_ERROR("Failed to call service pressure");
        return 1;
      }
    }while(effort <= 600.0);
        
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
  
  }while(true);
  /**
   *  PRUEBA BUCLE
   **/
  //ros::shutdown();  
  return 0;
}