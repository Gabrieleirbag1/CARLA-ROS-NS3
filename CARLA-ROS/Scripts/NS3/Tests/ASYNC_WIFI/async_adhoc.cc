#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring> // Pour memset
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h> // Pour close

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
  while (socket->Recv ())
    {
      NS_LOG_UNCOND ("Received one packet!");
    }
}

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic,
                           socket, pktSize,pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
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
    Ptr<WaypointMobilityModel> waypointMobility = nodes.Get(id+1)->GetObject<WaypointMobilityModel> ();

    double x = std::get<0>(coordinates) - map_x;
    double y = std::get<1>(coordinates) - map_y;
    double z = std::get<2>(coordinates) - map_z;

    Waypoint waypoint (Seconds (elapsedTime), Vector (x, y, z));
    waypointMobility->AddWaypoint (waypoint);

}

class AsyncUdpServer {
public:
    AsyncUdpServer(uint16_t port) : port(port), running(false) {}

    void start() {
        running = true;
        serverThread = std::thread(&AsyncUdpServer::run, this);
    }

    void stop() {
        running = false;
        if (serverThread.joinable()) {
            serverThread.join();
        }
    }

    ~AsyncUdpServer() {
        stop();
    }

private:
    void run() {
        int sockfd;
        struct sockaddr_in servaddr, cliaddr;

        // Création du socket
        if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            std::cerr << "socket creation failed" << std::endl;
            return;
        }

        memset(&servaddr, 0, sizeof(servaddr));
        memset(&cliaddr, 0, sizeof(cliaddr));

        // Remplissage des informations du serveur
        servaddr.sin_family = AF_INET; // IPv4
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port = htons(port);

        std::string phyMode ("DsssRate1Mbps");
        double rss = -80;  // -dBm
        uint32_t packetSize = 1000; // bytes
        uint32_t numPackets = 1;
        double interval = 1.0; // seconds
        bool verbose = false;

        CommandLine cmd;
        cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
        cmd.AddValue ("rss", "received signal strength", rss);
        cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
        cmd.AddValue ("numPackets", "number of packets generated", numPackets);
        cmd.AddValue ("interval", "interval (seconds) between packets", interval);
        cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);

        MobilityHelper mobility;
        //Map
        double map_x = 0.0;
        double map_y = 0.0;
        double map_z = 0.0;

        //Vehicle
        int vehicle_number = 0;
        
        // Initialize the clock variable without starting it
        clock_t startTime = 0;

        NodeContainer wifiNodes;
        Ipv4InterfaceContainer apInterface, staNodeInterfaces;
        uint16_t port = 9;

        // Liaison du socket avec l'adresse IP et le port
        if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
            std::cerr << "bind failed" << std::endl;
            return;
        }

        while (running) {
            char buffer[1024];
            unsigned int len = sizeof(cliaddr);
            int n = recvfrom(sockfd, (char *)buffer, 1024, MSG_WAITALL, (struct sockaddr *)&cliaddr, &len);
            buffer[n] = '\0';

            std::vector<std::string> msg;
            std::istringstream iss(buffer);
            std::string token;
            while (std::getline(iss, token, '|')) {
                msg.push_back(token);
            }
            std::cout << "Message Received: " << buffer << std::endl;
        
            if (msg[0] == "MAP") {
                double x = std::stod(msg[1]);
                double y = std::stod(msg[2]);
                double z = std::stod(msg[3]);
    
                map_x = x;
                map_y = y;
                map_z = z;

                Ptr<WaypointMobilityModel> waypointMobility = wifiNodes.Get (0)->GetObject<WaypointMobilityModel> ();

                Waypoint waypoint (Seconds (0), Vector (0, 0, 0));
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

                positionCallback(wifiNodes, coordinates, id, map_x, map_y, map_z, elapsedTime);

                // Respond to the client who sent the message
                std::string response = std::string(buffer);
                sendto(sockfd, response.c_str(), response.length(), MSG_CONFIRM, (const struct sockaddr *)&cliaddr, len);

                // if (elapsedTime >= 10.0) {
                //     break;
                // }
            }
    
            else if (msg[0] == "VEHICLE") {
                vehicle_number = std::stoi(msg[1]);
                // Resize the outer list to hold the number of vehicles
                wifiNodes.Create (vehicle_number+1);

                // Configuration de la mobilité des nœuds
                mobility.SetMobilityModel ("ns3::WaypointMobilityModel");
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

                for (int j = 0; j < vehicle_number; j++) {
                    Ptr<Socket> recvSink = Socket::CreateSocket (wifiNodes.Get (j), tid);
                    InetSocketAddress local = InetSocketAddress (i.GetAddress(j), 80);
                    recvSink->Bind (local);
                    recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));
                }
                
                for (int j = 0; j < vehicle_number; j++) {
                    Ptr<Socket> source = Socket::CreateSocket (wifiNodes.Get (j), tid);

                    for (int k = 0; k < vehicle_number; k++) {
                        if (j == k) {
                            continue;
                        }
                        InetSocketAddress remote = InetSocketAddress (i.GetAddress(k), 80);
                        source->SetAllowBroadcast (true);
                        source->Connect (remote);
                    }
                }

                // Tracing
                wifiPhy.EnablePcap ("wifi-simple-adhoc", devices);

                // Output what we are doing
                NS_LOG_UNCOND ("Testing " << numPackets  << " packets sent with receiver rss " << rss );
            }

        }

        close(sockfd);

        AnimationInterface anim ("adhoc-wifi-animation.xml");

        Simulator::Run ();
        Simulator::Stop();
        Simulator::Destroy();
    }

    uint16_t port;
    std::atomic<bool> running;
    std::thread serverThread;
};

int main() {
    AsyncUdpServer server(12345);
    server.start();

    // Simuler une attente pour démontrer le fonctionnement asynchrone
    std::this_thread::sleep_for(std::chrono::seconds(10));

    std::cout << "Stopping server..." << std::endl;
    server.stop();
    
    return 0;
}