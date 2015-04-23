#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
//#include <tf2_ros/buffer.h>
#include "tekscan_client/GetPressureMap.h"

#include <iostream>
#include <fstream>
using namespace std;


int main(int argc, char** argv){
  ros::init(argc, argv, "shadow_tf_listener");
  ros::NodeHandle node;
  
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener listener(tfBuffer);
  
  
  
  // >> PRESSURE MAP SERVICE
  ros::ServiceClient pressure_client = node.serviceClient<tekscan_client::GetPressureMap>("GetPressureMap");
  tekscan_client::GetPressureMap srv_pressure;
  srv_pressure.request.num = 0; 
  
  ros::Rate rate(10.0);
  
  ofstream myfile;
  myfile.open ("/home/aurova/Desktop/pruebas/resultados/positions.txt");
  if(myfile.is_open())
    ROS_INFO("Archivo abierto");
  
  int iteration = 0;
  while (node.ok()){
    //tf::StampedTransform transform;
    geometry_msgs::TransformStamped transform;
   
    // get timestamp
    
    try{
      // obtener posiciones tips: 
      
      // transform -> get pose ff
      transform = tfBuffer.lookupTransform("/forearm", "/thtip",ros::Time(1.0));
      
      //transform.getRotaton();
      /**
      myfile << transform.getOrigin().x();
      myfile << " ";
      myfile << transform.getOrigin().y();
      myfile << " ";
      myfile << transform.getOrigin().z();
      myfile << " ";
      */
      
      // transform -> get pose ff
      //transform = tfBuffer.lookupTransform("/forearm", "/fftip",ros::Time(0));
      
      /**
      myfile << transform.getOrigin().x();
      myfile << " ";
      myfile << transform.getOrigin().y();
      myfile << " ";
      myfile << transform.getOrigin().z();
      myfile << " ";
      */
      
      // transform -> get pose mf
      //transform = tfBuffer.lookupTransform("/forearm", "/mftip",ros::Time(0));
      /**
      myfile << transform.getOrigin().x();
      myfile << " ";
      myfile << transform.getOrigin().y();
      myfile << " ";
      myfile << transform.getOrigin().z();
      myfile << " ";
      */
      
      // transform -> get pose rf
      //transform = tfBuffer.lookupTransform("/forearm", "/rftip",ros::Time(0));
      /**
      myfile << transform.getOrigin().x();
      myfile << " ";
      myfile << transform.getOrigin().y();
      myfile << " ";
      myfile << transform.getOrigin().z();
      myfile << " ";
      */
      
      // transform -> get pose lf
      //transform = tfBuffer.lookupTransform("/forearm", "/lftip",ros::Time(0));
      
      /**
      myfile << transform.getOrigin().x();
      myfile << " ";
      myfile << transform.getOrigin().y();
      myfile << " ";
      myfile << transform.getOrigin().z();
      myfile << " ";
      */
      
      // Guardar en archivos posiciones (posth, posff, posmf, posrf, poslf, timestamp)
    
      /**myfile << "\n"; */
      iteration++;
      
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    
    
    // Obtener mapa de presión
    
    /**if (pressure_client.call(srv_pressure))
    {*/
      // guardar en archivos fuerza/presión

	  /**myfile << srv_pressure.response.applied_force[0];
	  myfile << srv_pressure.response.applied_force[1];
	  myfile << srv_pressure.response.applied_force[2];
	  myfile << srv_pressure.response.applied_force[3];
	  myfile << srv_pressure.response.applied_force[4];
	  */
    /**}
    else
    {
      ROS_ERROR("Failed to call service pressure");
      return 1;
    }*/
   

    rate.sleep();
  }
  myfile.close();
  
  return 0;
};