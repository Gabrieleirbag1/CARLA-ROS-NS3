#include <iostream>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/netanim-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-layout-module.h"
#include "ns3/rosns3-module.h"
#include "ns3/rosns3-helper.h"
#include <regex>
#include <list>
#include <array>

using namespace ns3;

void positionCallback(std::list<std::array<double, 3>> coordinates)
{
  NodeContainer nodes;
  nodes.Create (1); // Créez un seul nœud

  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::WaypointMobilityModel");
  mobility.Install (nodes);

  Ptr<WaypointMobilityModel> waypointMobility = nodes.Get (0)->GetObject<WaypointMobilityModel> ();

  for(list<double>::iterator it = coordinates.begin(); it != coordinates.end(); it++){
      double x = (*it)[0];
      double y = (*it)[1];
      Waypoint waypoint (Seconds (0), Vector (x, y, 0));
      waypointMobility->AddWaypoint (waypoint);

    }
}


int main (int argc, char *argv[])
{// Déclare une liste de tableaux de doubles
  std::list<std::array<double, 3>> coordinates;

  ROSNS3Server server(12345);
  // Démarrer le serveur
  if (!server.start()) {
    std::cerr << "Erreur lors du démarrage du serveur" << std::endl;
    return 1;
  }
  
  clock_t startTime = clock();

  while (server.server_running()){
    if (server.data_ready()) {
      // Obtenir les données
      recvdata_t data = server.get_data();
      // std::cout << "Données reçues : " << data.buffer << std::endl;
      std::string data_string = data.buffer;
      // Utiliser les regex pour extraire les valeurs de x, y et z

      std::string regex_pattern = "\\((-?\\d+\\.\\d+), (-?\\d+\\.\\d+), (-?\\d+\\.\\d+)\\)";
      std::regex r(regex_pattern);
      std::smatch matches;
      if (std::regex_search(data_string, matches, r)) {
          double x = std::stod(matches[1].str());
          double y = std::stod(matches[2].str());
          double z = std::stod(matches[3].str());
          std::cout << "Données reçues (string) : " << "x = " << x << ", y = " << y << ", z = " << z << std::endl;

          double tab_coordinates[3] = {x, y, z};
          coordinates.push_back(tab_coordinates);
      }
      // Envoyer une réponse
      const char* response = "Réponse du serveur";
      server.send_data((uint8_t*)response, strlen(response) + 1);

      clock_t currentTime = clock(); // Get the current time
      double elapsedTime = static_cast<double>(currentTime - startTime) / CLOCKS_PER_SEC; // Calculate the elapsed time in seconds

      std::cout << "Elapsed time: " << elapsedTime << " seconds" << std::endl;

  if (elapsedTime >= 30.0) {
    break; // Break the loop after 10 seconds
  }
}
  }

  server.kill();

  positionCallback(coordinates);

  AnimationInterface anim ("animation.xml");

  Simulator::Stop (Seconds (30.0));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}