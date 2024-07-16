#include <iostream>
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/rosns3-module.h"
#include "ns3/rosns3-helper.h"
#include <regex>

using namespace ns3;

void positionCallback(std::vector<double> xCoordinates, std::vector<double> yCoordinates, std::vector<double> zCoordinates, double map_width, double map_height)
{
  NodeContainer nodes;
  nodes.Create (1); // Créez un seul nœud

  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::WaypointMobilityModel");
  mobility.Install (nodes);

  Ptr<WaypointMobilityModel> waypointMobility = nodes.Get (0)->GetObject<WaypointMobilityModel> ();
  // std::cout << "xCoordinates.size() = " << xCoordinates.size() << std::endl;
  // Find the minimum coordinates
  double minX = *std::min_element(xCoordinates.begin(), xCoordinates.end());
  double minY = *std::min_element(yCoordinates.begin(), yCoordinates.end());

  for (size_t i = 0; i < xCoordinates.size(); i++)
  {
    // Adjust the coordinates to make them all positive
    double adjustedX = xCoordinates[i] - minX;
    double adjustedY = yCoordinates[i] - minY;

    Waypoint waypoint (Seconds (i), Vector (adjustedX, adjustedY, 0));
    waypointMobility->AddWaypoint (waypoint);
  }

}

int main (int argc, char *argv[])
{
  bool map = false;
  double map_width = 0.0;
  double map_height = 0.0;

  std::vector<double> xCoordinates;
  std::vector<double> yCoordinates;
  std::vector<double> zCoordinates;

  clock_t startTime = 0; // Initialize the clock variable without starting it

  // Utiliser les regex pour extraire les valeurs de x, y et z
  std::string regex_pattern = "\\((-?\\d+\\.\\d+), (-?\\d+\\.\\d+), (-?\\d+\\.\\d+)\\)";
  std::regex r(regex_pattern);
  std::smatch matches;

  std::cout << "Starting the server..." << std::endl;

  ROSNS3Server server(12345);
  // Démarrer le serveur
  if (!server.start()) {
    std::cerr << "Erreur lors du démarrage du serveur" << std::endl;
    return 1;
  }

  while (server.server_running()){

    if (server.data_ready()) {
      // Obtenir les données
      recvdata_t data = server.get_data();
      // std::cout << "Données reçues : " << data.buffer << std::endl;
      std::string data_string = data.buffer;

      std::cout << "Données reçues (string) : " << data_string << std::endl;

      if (std::regex_search(data_string, matches, r)) {
        double x = std::stod(matches[1].str());
        double y = std::stod(matches[2].str());
        double z = std::stod(matches[3].str());

        if (!map) {
          std::cout << "Map : " << data_string << std::endl;
          std::cout << "Map : " << "x = " << x << ", y = " << y << ", z = " << z << std::endl;
          map_width = x;
          map_height = y;
          map = true;
          startTime = clock(); // Start the clock
        }

        else if (map) {
          std::cout << "Données reçues (string) : " << "x = " << x << ", y = " << y << ", z = " << z << std::endl;

          xCoordinates.push_back(x);
          yCoordinates.push_back(y);
          zCoordinates.push_back(z);
          // Envoyer une réponse
          const char* response = "Réponse du serveur";
          server.send_data((uint8_t*)response, strlen(response) + 1);

          clock_t currentTime = clock();
          double elapsedTime = static_cast<double>(currentTime - startTime) / CLOCKS_PER_SEC; // Calculate the elapsed time in seconds

          std::cout << "Elapsed time: " << elapsedTime << " seconds" << std::endl;

          if (elapsedTime >= 10.0) {
            break; // Break the loop after 10 seconds
          }
        }
      }
    }
  }

  server.kill();

    // ...
  positionCallback(xCoordinates, yCoordinates, zCoordinates, map_width, map_height);

  AnimationInterface anim ("animation.xml");

  Simulator::Stop (Seconds (xCoordinates.size()));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}