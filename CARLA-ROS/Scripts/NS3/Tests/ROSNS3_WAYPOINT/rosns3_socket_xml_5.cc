#include <iostream>
#include <vector>
#include <algorithm>
#include <ctime>
#include <sstream>
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/rosns3-module.h"
#include "ns3/rosns3-helper.h"

using namespace ns3;

void positionCallback(int vehicle_number, std::vector<std::vector<std::vector<double>>> vehicleCoordinates, double map_width, double map_height)
{
  NodeContainer nodes;

  // Create the number of nodes specified by the vehicle_number
  nodes.Create (vehicle_number);

  // Create a mobility helper for the WaypointMobilityModel
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::WaypointMobilityModel");
  mobility.Install (nodes);

  for (int i = 0; i < vehicle_number; i++)
  {
    Ptr<WaypointMobilityModel> waypointMobility = nodes.Get (i)->GetObject<WaypointMobilityModel> ();

    for (size_t j = 0; j < vehicleCoordinates[i].size(); j++)
    {
      // Use the coordinates directly
      double x = vehicleCoordinates[i][j][0];
      double y = vehicleCoordinates[i][j][1];
      double z = vehicleCoordinates[i][j][2];

      Waypoint waypoint (Seconds (j), Vector (x, y, z));
      waypointMobility->AddWaypoint (waypoint);
    }
  }
}

int main (int argc, char *argv[])
{
  //Map
  double map_width = 0.0;
  double map_height = 0.0;

  //Vehicle
  int vehicle_number = 0;

  //Coordinates
  std::vector<std::vector<std::vector<double>>> vehicleCoordinates;
  
  // Initialize the clock variable without starting it
  clock_t startTime = 0;

  ROSNS3Server server(12345);
  // Start the server
  if (!server.start()) {
    std::cerr << "Error starting the server" << std::endl;
    return 1;
  }

  while (server.server_running()){

    if (server.data_ready()) {
      // Get the data
      recvdata_t data = server.get_data();
      std::string data_string = data.buffer;

      std::vector<std::string> msg;
      std::istringstream iss(data_string);
      std::string token;
      while (std::getline(iss, token, '|')) {
        msg.push_back(token);
      }

      std::cout << "Received data (string): " << data_string << std::endl;

      if (msg[0] == "MAP") {
        double x = std::stod(msg[1]);
        double y = std::stod(msg[2]);

        map_width = x;
        map_height = y;
        startTime = clock(); // Start the clock
      }

      else if (msg[0] == "COORDINATES") {
        int id = std::stoi(msg[1]);
        double x = std::stod(msg[2]);
        double y = std::stod(msg[3]);
        double z = std::stod(msg[4]);

        if (id < vehicle_number) {
          // Push the new coordinates to the corresponding vehicle's list
          vehicleCoordinates[id].push_back({x, y, z});
        }

        // Send a response
        const char* response = "Server response";
        server.send_data((uint8_t*)response, strlen(response) + 1);

        clock_t currentTime = clock();
        double elapsedTime = static_cast<double>(currentTime - startTime) / CLOCKS_PER_SEC; // Calculate the elapsed time in seconds

        if (elapsedTime >= 30.0) {
          break; // Break the loop after 2 seconds
        }
      }

      else if (msg[0] == "VEHICLE") {
        vehicle_number = std::stoi(msg[1]);
        // Resize the outer list to hold the number of vehicles
        vehicleCoordinates.resize(vehicle_number);
      }
    }
  }

  server.kill();

  positionCallback(vehicle_number, vehicleCoordinates, map_width, map_height);

  AnimationInterface anim ("animation.xml");

  Simulator::Stop (Seconds (vehicleCoordinates[0].size()));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}