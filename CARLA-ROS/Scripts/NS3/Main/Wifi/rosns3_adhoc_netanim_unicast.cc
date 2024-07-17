#include <iostream>
#include <algorithm>
#include <ctime>
#include <sstream>
#include <thread>
#include <vector>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/netanim-module.h"
#include "ns3/applications-module.h" 
#include "ns3/rosns3-module.h"
#include "ns3/rosns3-helper.h"

using namespace ns3;

// Add logging functionality to see the message content
NS_LOG_COMPONENT_DEFINE("WifiSimpleExample");

void ReceivePacket (Ptr<Socket> socket)
{
    Ptr<Packet> packet;
    while ((packet = socket->Recv ()))
        {
        uint8_t *buffer = new uint8_t[packet->GetSize ()];
        packet->CopyData(buffer, packet->GetSize ());
        std::string packetContent(buffer, buffer + packet->GetSize ());
        NS_LOG_UNCOND ("Received one packet: " << packetContent);
        delete[] buffer;
        }    
}

static void SendPacket(Ptr<Socket> socket, Ptr<Packet> packet)
{
  socket->Send(packet);
}

static void GenerateTraffic (Ptr<Socket> socket, std::string message, Time pktInterval)
{
  Ptr<Packet> packet = Create<Packet> ((uint8_t*)message.c_str(), message.length());
  Simulator::Schedule (pktInterval, &SendPacket, socket, packet);
  std::cout << "Sent one packet: " << message << std::endl;
}

void positionCallback(
    NodeContainer nodes, 
    std::tuple<double, double, double> coordinates, 
    int id, 
    double map_x, 
    double map_y, 
    double map_z,
    double elapsedTime
    )

{   
    // Create a mobility helper for the WaypointMobilityModel
    Ptr<WaypointMobilityModel> waypointMobility = nodes.Get(id)->GetObject<WaypointMobilityModel> ();

    double x = std::get<0>(coordinates) - map_x;
    double y = std::get<1>(coordinates) - map_y;
    double z = std::get<2>(coordinates) - map_z;

    double currentSimTime = Simulator::Now().GetSeconds();
    if (elapsedTime <= currentSimTime) {
        NS_LOG_UNCOND("Adjusted waypoint time from " << elapsedTime << " to " << currentSimTime + 0.1 << " to ensure positive delay.");
        elapsedTime = currentSimTime + 0.1; // Adjust elapsedTime to be slightly in the future
    }

    Waypoint waypoint (Seconds (elapsedTime), Vector (x, y, z));
    waypointMobility->AddWaypoint (waypoint);
}

void manage_data(
    std::string phyMode="DsssRate1Mbps", 
    double rss=-80, 
    uint32_t packetSize=1000, 
    uint32_t numPackets=1, 
    double interval=1.0,
    bool verbose=false,
    double time_duration=30.0)
{
    //Map
    double map_x = 0.0;
    double map_y = 0.0;
    double map_z = 0.0;

    //Vehicle
    int vehicle_number = 0;

    // Initialize the clock variable without starting it
    MobilityHelper mobility;
    clock_t startTime = 0;

    Time interPacketInterval = Seconds (interval);

    ROSNS3Server server(12345);
    // Start the server
    if (!server.start()) {
        std::cerr << "Error starting the server" << std::endl;
        return;
    }

    NodeContainer mapNode;
    mapNode.Create (1);

    NodeContainer wifiNodes;
    uint16_t port = 9;
    
    std::vector<std::vector<Ptr<Socket>>> sourcesNodes;
    std::vector<Ptr<Socket>> receiverNodes;

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
                double z = std::stod(msg[3]);
    
                map_x = x;
                map_y = y;
                map_z = z;

                Ptr<WaypointMobilityModel> waypointMobility = mapNode.Get (0)->GetObject<WaypointMobilityModel> ();

                Waypoint waypoint (Seconds (0.5), Vector (0, 0, 0));
                waypointMobility->AddWaypoint (waypoint);
                startTime = clock(); // Start the clock
            }
    
            else if (msg[0] == "COORDINATES") {
                int id = std::stoi(msg[1]);
                double x = std::stod(msg[2]);
                double y = std::stod(msg[3]);
                double z = std::stod(msg[4]);
                double elapsedTime = std::stod(msg[8]);

                std::tuple<double, double, double> coordinates = std::make_tuple(x, y, z);

                std::cout << "time : " << elapsedTime << std::endl;
                
                positionCallback(wifiNodes, coordinates, id, map_x, map_y, map_z, elapsedTime);
                
                for (int i = 0; i < vehicle_number - 1; i++) {
                    Ptr<Socket> source = sourcesNodes[id][i];
                    Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                            Seconds (elapsedTime), &GenerateTraffic,
                            source, data_string, interPacketInterval);
                }
                
                clock_t currentTime = clock();
                double elapsedTimeClock = static_cast<double>(currentTime - startTime) / CLOCKS_PER_SEC; // Calculate the elapsed time in seconds
                std::cout << "Elapsed time: " << elapsedTimeClock << std::endl;

                if (elapsedTimeClock >= time_duration) {
                    break;
                }
            }
    
            else if (msg[0] == "VEHICLE") {
                vehicle_number = std::stoi(msg[1]);
                // Resize the outer list to hold the number of vehicles
                wifiNodes.Create (vehicle_number);

                // Resize the outer vector to hold the number of vehicles
                sourcesNodes.resize(vehicle_number);

                // Configuration de la mobilité des nœuds
                mobility.SetMobilityModel ("ns3::WaypointMobilityModel");
                mobility.Install (mapNode);
                mobility.Install (wifiNodes);

                WifiHelper wifi;
                if (verbose)
                    {
                    wifi.EnableLogComponents ();  // Turn on all Wifi logging
                    }
                wifi.SetStandard (WIFI_PHY_STANDARD_80211b);

                YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
                // This is one parameter that matters when using FixedRssLossModel
                // set it to zero; otherwise, gain will be added
                wifiPhy.Set ("RxGain", DoubleValue (0) );
                // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
                wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);

                YansWifiChannelHelper wifiChannel;
                wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
                // The below FixedRssLossModel will cause the rss to be fixed regardless
                // of the distance between the two stations, and the transmit power
                wifiChannel.AddPropagationLoss ("ns3::FixedRssLossModel","Rss",DoubleValue (rss));
                wifiPhy.SetChannel (wifiChannel.Create ());

                // Add a mac and disable rate control
                WifiMacHelper wifiMac;
                wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                                "DataMode",StringValue (phyMode),
                                                "ControlMode",StringValue (phyMode));
                // Set it to adhoc mode
                wifiMac.SetType ("ns3::AdhocWifiMac");
                NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, wifiNodes);

                InternetStackHelper internet;
                internet.Install (wifiNodes);

                Ipv4AddressHelper ipv4;
                NS_LOG_INFO ("Assign IP Addresses.");
                ipv4.SetBase ("10.1.1.0", "255.255.255.0");
                Ipv4InterfaceContainer i = ipv4.Assign (devices);

                TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
                // Gestion du Receiver pour chaque nœud
                for (int j = 0; j < vehicle_number; j++) {
                    std::cout << "Creating receiver for node " << j << std::endl;
                    Ptr<Socket> recvSink = Socket::CreateSocket (wifiNodes.Get (j), tid);
                    InetSocketAddress local = InetSocketAddress (i.GetAddress(j), 80);
                    recvSink->Bind (local);
                    recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));
                    receiverNodes.push_back(recvSink);       
                }

                // Gestion du Sender pour chaque nœud
                for (int j = 0; j < vehicle_number; j++) {
                    for (int k = 0; k < vehicle_number; k++) {
                        Ptr<Socket> source = Socket::CreateSocket (wifiNodes.Get (j), tid);
                        if (j == k) {
                            continue;
                        }
                        InetSocketAddress remote = InetSocketAddress (i.GetAddress(k), 80);
                        source->SetAllowBroadcast (true);
                        source->Connect (remote);

                        std::cout << "list" << j << k << std::endl;
                        sourcesNodes[j].push_back(source);
                    }
                }

                // Tracing
                wifiPhy.EnablePcap ("wifi-simple-adhoc", devices);

            }
        }
    }

    const char* response = "STOP|";
    server.send_data((uint8_t*)response, strlen(response) + 1);

    server.kill();

    AnimationInterface anim("adhoc-wifi-animation.xml");

    Simulator::Run();
    Simulator::Destroy();
}

int main (int argc, char *argv[])
{
    std::string phyMode ("DsssRate1Mbps");
    double rss = -80;  // -dBm
    uint32_t packetSize = 1000; // bytes
    uint32_t numPackets = 1;
    double interval = 1.0; // seconds
    bool verbose = false;
    double time_duration = 30.0;

    CommandLine cmd;
    cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
    cmd.AddValue ("rss", "received signal strength", rss);
    cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
    cmd.AddValue ("numPackets", "number of packets generated", numPackets);
    cmd.AddValue ("interval", "interval (seconds) between packets", interval);
    cmd.AddValue ("time", "setup the time duration of the simulation", time_duration);
    cmd.Parse (argc, argv);

    manage_data(phyMode, rss, packetSize, numPackets, interval, verbose, time_duration);

    return 0;
}