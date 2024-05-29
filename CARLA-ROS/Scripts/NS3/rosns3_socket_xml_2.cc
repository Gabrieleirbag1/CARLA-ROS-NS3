#include <iostream>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/netanim-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-layout-module.h"
#include "ns3/rosns3-module.h"
#include "ns3/rosns3-helper.h"
#include <regex>

using namespace ns3;

AnimationInterface *anim;
Ptr<Node> carNode;

void positionCallback(double x, double y, double z)
{
  // Update the position of the existing node
  anim->SetConstantPosition(carNode, x, y);
}

int main (int argc, char *argv[])
{
  // Initialize the ns-3 environment
  NodeContainer nodes;
  nodes.Create(1);
  
  // Assign the car node
  carNode = nodes.Get(0);

  // Initialize the AnimationInterface once
  anim = new AnimationInterface("animation.xml");

  // Setup the mobility poll interval
  anim->SetMobilityPollInterval(Seconds(1));

  // Set initial position (can be optional)
  anim->SetConstantPosition(carNode, 0.0, 0.0);

  // ROSNS3Server setup
  ROSNS3Server server(12345);
  
  if (!server.start()) {
    std::cerr << "Erreur lors du démarrage du serveur" << std::endl;
    return 1;
  }
  
  clock_t startTime = clock();

  while (server.server_running()){
    if (server.data_ready()) {
      recvdata_t data = server.get_data();
      std::string data_string = data.buffer;

      // Extract x, y, z values using regex
      std::string regex_pattern = "\\((-?\\d+\\.\\d+), (-?\\d+\\.\\d+), (-?\\d+\\.\\d+)\\)";
      std::regex r(regex_pattern);
      std::smatch matches;
      if (std::regex_search(data_string, matches, r)) {
          double x = std::stod(matches[1].str());
          double y = std::stod(matches[2].str());
          double z = std::stod(matches[3].str());
          std::cout << "Données reçues (string) : " << "x = " << x << ", y = " << y << ", z = " << z << std::endl;

          positionCallback(x, y, z);
      }

      // Send a response
      const char* response = "Réponse du serveur";
      server.send_data((uint8_t*)response, strlen(response) + 1);

      clock_t currentTime = clock();
      double elapsedTime = static_cast<double>(currentTime - startTime) / CLOCKS_PER_SEC;
      std::cout << "Elapsed time: " << elapsedTime << " seconds" << std::endl;

      if (elapsedTime >= 30.0) {
        break;
      }
    }
  }

  Simulator::Run ();
  Simulator::Destroy ();
  
  server.kill();

  delete anim; // Clean up

  return 0;
}
