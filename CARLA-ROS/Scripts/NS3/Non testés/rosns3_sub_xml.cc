#include "ns3/core-module.h"
#include "ns3/rosns3-module.h"
#include "ns3/rosns3-helper.h"
#include "ns3/netanim-module.h"
#include "ros/console.h"
#include "ros/ros.h"
#include "rosns3_client/Waypoint.h"

using namespace ns3;

void positionCallback(const rosns3_client::Waypoint::ConstPtr& msg)
{
  // Create an AnimationInterface object
  AnimationInterface anim("animation.xml");

  // Set the mobility poll interval
  anim.SetMobilityPollInterval(Seconds(1));

  // Extract x, y, and z values
  double x = msg->position.x;
  double y = msg->position.y;
  double z = msg->position.z;

  // Create a new node and set its position
  Ptr<Node> n = CreateObject<Node>();
  anim.SetConstantPosition(n, x, y);

  anim.SetStartTime(Seconds(0.0));
  anim.SetStopTime(Seconds(10.0));
}

int main (int argc, char *argv[])
{
  ros::init(argc, argv, "rosns3_client");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/robot_1/current_state", 1000, positionCallback);

  ros::spin();

  return 0;
}
