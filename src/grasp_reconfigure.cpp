#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <moveit_shadow_test/grasp_reconfigureConfig.h>

void callback(moveit_shadow_test::grasp_reconfigureConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %f %f", 
            config.numero_dedos, config.min_pressure_threshold, 
            config.max_pressure_threshold);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "grasp_reconfigure");

  dynamic_reconfigure::Server<moveit_shadow_test::grasp_reconfigureConfig> server;
  dynamic_reconfigure::Server<moveit_shadow_test::grasp_reconfigureConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}