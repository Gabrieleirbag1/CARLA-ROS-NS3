#include <iostream>
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/rosns3-module.h"
#include "ns3/rosns3-helper.h"
#include <regex>

using namespace ns3;

void positionCallback(std::vector<double> xCoordinates, std::vector<double> yCoordinates, std::vector<double> zCoordinates)
{
  NodeContainer nodes;
  nodes.Create (1); // Créez un seul nœud

  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::WaypointMobilityModel");
  mobility.Install (nodes);

  Ptr<WaypointMobilityModel> waypointMobility = nodes.Get (0)->GetObject<WaypointMobilityModel> ();
  // std::cout << "xCoordinates.size() = " << xCoordinates.size() << std::endl;
  for (size_t i = 0; i < xCoordinates.size(); i++)
  {
    Waypoint waypoint (Seconds (i), Vector (xCoordinates[i], yCoordinates[i], 0));
    waypointMobility->AddWaypoint (waypoint);
  }
}

int main (int argc, char *argv[])
{
  std::vector<double> xCoordinates;
  std::vector<double> yCoordinates;
  std::vector<double> zCoordinates;
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

          xCoordinates.push_back(x);
          yCoordinates.push_back(y);
          zCoordinates.push_back(z);
      }
      // Envoyer une réponse
      const char* response = "Réponse du serveur";
      server.send_data((uint8_t*)response, strlen(response) + 1);

      clock_t currentTime = clock(); // Get the current time
      double elapsedTime = static_cast<double>(currentTime - startTime) / CLOCKS_PER_SEC; // Calculate the elapsed time in seconds

      std::cout << "Elapsed time: " << elapsedTime << " seconds" << std::endl;

  if (elapsedTime >= 10.0) {
    break; // Break the loop after 10 seconds
  }
}
  }

  server.kill();

  positionCallback(xCoordinates, yCoordinates, zCoordinates);

  AnimationInterface anim ("animation.xml");

  Simulator::Stop (Seconds (xCoordinates.size()));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}