#include <ros/ros.h>
#include <observation_registration_services/ObservationRegistrationService.h>

bool observataion_registration_service(
	observation_registration_services::ObservationRegistrationService::Request  &req,
        observation_registration_services::ObservationRegistrationService::Response &res)
{
  ROS_INFO("Received an observation registration request");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "observation_registration_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("observation_registration_server", observataion_registration_service);
  ROS_INFO("observation_registration_server started.");
  ros::spin();

  return 0;
}
