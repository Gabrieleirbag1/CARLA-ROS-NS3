#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"

using namespace ns3;

int main (int argc, char *argv[])
{
  NodeContainer nodes;
  nodes.Create (1); // Créez un seul nœud

  // Vos listes de coordonnées x et y
  double xCoordinates[31] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0, 231.0};
  double yCoordinates[31] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0, 231.0};

  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::WaypointMobilityModel");
  mobility.Install (nodes);

  Ptr<WaypointMobilityModel> waypointMobility = nodes.Get (0)->GetObject<WaypointMobilityModel> ();

  for (int i = 0; i < 31; i++)
  {
    Waypoint waypoint (Seconds (i), Vector (xCoordinates[i], yCoordinates[i], 0)); // Le nœud atteindra cette position à la seconde i
    waypointMobility->AddWaypoint (waypoint);
  }

  AnimationInterface anim ("animation.xml");

  Simulator::Stop (Seconds (30.0)); // Arrêtez la simulation après 30 secondes
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}