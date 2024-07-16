#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/rosns3-module.h"

using namespace ns3;

int main (int argc, char *argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);

  NodeContainer nodes;
  nodes.Create (2);

  PointToPointHelper pointToPoint;
  pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  pointToPoint.SetChannelAttribute ("Delay", StringValue ("2ms"));

  NetDeviceContainer devices;
  devices = pointToPoint.Install (nodes);

  InternetStackHelper stack;
  stack.Install (nodes);

  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");

  Ipv4InterfaceContainer interfaces = address.Assign (devices);

  RosBridgeHelper rosBridge;
  rosBridge.SetAttribute ("Local", AddressValue (InetSocketAddress (Ipv4Address::GetAny (), 9)));
  rosBridge.SetAttribute ("Topic", StringValue ("/car_control"));
  rosBridge.SetAttribute ("ns3::PacketSocket::RecvBufSize", UintegerValue (20));

  ApplicationContainer rosApps = rosBridge.Install (nodes.Get (1));
  rosApps.Start (Seconds (1.0));
  rosApps.Stop (Seconds (10.0));

  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
