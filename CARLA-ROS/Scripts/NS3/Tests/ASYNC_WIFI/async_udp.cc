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

class CustomMessageReceiver : public Application 
{
public:
    CustomMessageReceiver();
    virtual ~CustomMessageReceiver();
    void Setup(Address address, uint16_t port);

    virtual void StartApplication(void);
    virtual void StopApplication(void);

private:
    void HandleRead(Ptr<Socket> socket);

    Ptr<Socket> m_socket;
    Address m_local;
    bool m_running;
};

CustomMessageReceiver::CustomMessageReceiver()
  : m_socket(0), 
    m_local(), 
    m_running(false) 
{
}

CustomMessageReceiver::~CustomMessageReceiver()
{
    m_socket = 0;
}

void CustomMessageReceiver::Setup(Address address, uint16_t port)
{
    m_local = InetSocketAddress(Ipv4Address::ConvertFrom(address), port);
}

void CustomMessageReceiver::StartApplication(void)
{
    m_running = true;
    if (!m_socket)
    {
        m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_socket->Bind(m_local);
        m_socket->SetRecvCallback(MakeCallback(&CustomMessageReceiver::HandleRead, this));
    }
}

void CustomMessageReceiver::StopApplication(void)
{
    m_running = false;
    if (m_socket)
    {
        m_socket->Close();
        m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
    }
}

void CustomMessageReceiver::HandleRead(Ptr<Socket> socket)
{
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom(from)))
    {
        if (packet->GetSize() > 0)
        {
            // Traiter le paquet reçu
            uint8_t buffer[packet->GetSize()];
            packet->CopyData(buffer, packet->GetSize());
            std::string packetContent(reinterpret_cast<char*>(buffer), packet->GetSize());
            NS_LOG_UNCOND("Packet content: " << packetContent);
        }
    }
}

class CustomMessageSender : public Application 
{
public:
    CustomMessageSender();
    virtual ~CustomMessageSender();
    void Setup(Ptr<Socket> socket, Address address, uint32_t packetSize, DataRate dataRate, std::string message);

    virtual void StartApplication(void);
    virtual void StopApplication(void);

    void SendPacket(std::string message);

    Ptr<Socket> m_socket;
    Address m_peer;
    uint32_t m_packetSize;
    DataRate m_dataRate;
    bool m_running;
    uint32_t m_packetsSent;
    std::string m_message; // Ajouté pour stocker le message à envoyer
};

CustomMessageSender::CustomMessageSender()
  : m_socket(0), 
    m_peer(), 
    m_packetSize(0), 
    m_dataRate(0), 
    m_running(false), 
    m_packetsSent(0),
    m_message("") // Initialisé à une chaîne vide
{
}

CustomMessageSender::~CustomMessageSender()
{
    m_socket = 0;
}

void CustomMessageSender::Setup(Ptr<Socket> socket, Address address, uint32_t packetSize, DataRate dataRate, std::string message)
{
    m_socket = socket;
    m_peer = address;
    m_packetSize = packetSize;
    m_dataRate = dataRate;
    m_message = message; // Stocke le message à envoyer
}

void CustomMessageSender::StartApplication(void)
{   
    m_running = true;
    m_packetsSent = 0;
    if (!m_socket)
    {
        TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
        m_socket = Socket::CreateSocket(GetNode(), tid);
        m_socket->Bind();
        m_socket->Connect(m_peer);
    }
    SendPacket(m_message); // Envoie un seul message
}

void CustomMessageSender::StopApplication(void)
{
    m_running = false;
}

void CustomMessageSender::SendPacket(std::string message)
{
    if (!message.empty())
    {
        Ptr<Packet> packet = Create<Packet>((uint8_t*)message.c_str(), message.length());
        m_socket->Send(packet);
        NS_LOG_UNCOND("Sending packet with message: " << message);
    }
}

void positionCallback(
    NodeContainer nodes, int vehicle_number, std::vector<std::vector<std::vector<double>>> vehicleCoordinates, double map_width, double map_height)
{   
    // Create a mobility helper for the WaypointMobilityModel
    MobilityHelper mobility;
    mobility.SetMobilityModel ("ns3::WaypointMobilityModel");
    mobility.Install (nodes);

    // Find the minimum x, y, and z values across all lists in vehicleCoordinates
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double minZ = std::numeric_limits<double>::max();

    for (const auto& vehicle : vehicleCoordinates) {
        for (const auto& coordinate : vehicle) {
        minX = std::min(minX, coordinate[0]);
        minY = std::min(minY, coordinate[1]);
        minZ = std::min(minZ, coordinate[2]);
        }
    }

    for (int i = 0; i < vehicle_number; i++)
    {
        int n = i+1;
        Ptr<WaypointMobilityModel> waypointMobility = nodes.Get (n)->GetObject<WaypointMobilityModel> ();

        for (size_t j = 0; j < vehicleCoordinates[i].size(); j++)
        {
        // Subtract the minimum values from the coordinates to ensure they are all positive
        double x = vehicleCoordinates[i][j][0] - minX;
        double y = vehicleCoordinates[i][j][1] - minY;
        double z = vehicleCoordinates[i][j][2] - minZ;

        Waypoint waypoint (Seconds (j), Vector (x, y, z));
        waypointMobility->AddWaypoint (waypoint);
        }
    }
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

        //Map
        double map_width = 0.0;
        double map_height = 0.0;

        //Vehicle
        int vehicle_number = 0;

        //Coordinates
        std::vector<std::vector<std::vector<double>>> vehicleCoordinates;
        
        // Initialize the clock variable without starting it
        clock_t startTime = 0;

        NodeContainer wifiNodes;
        Ipv4InterfaceContainer apInterface, staNodeInterfaces;
        uint16_t port = 9;

        std::vector<Ptr<CustomMessageSender>> senderAppNodes;
        std::vector<Ptr<CustomMessageReceiver>> receiverAppNodes;

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
                            
                // // Configuration des émetteurs
                // for (uint32_t i = 1; i < wifiNodes.GetN(); ++i) {
                //     if (i == id+1) {
                //         std::cout << "sender" << i << std::endl;
                //         continue;
                //     }
                    
                //     Ptr<CustomMessageSender> senderAppNode = CreateObject<CustomMessageSender>();
                //     senderAppNode->Setup(wifiNodes.Get(i)->GetObject<Socket>(), InetSocketAddress(staNodeInterfaces.GetAddress(i-1), port), 1024, DataRate("1Mbps"), buffer);
                //     wifiNodes.Get(i)->AddApplication(senderAppNode);

                //     senderAppNode->StartApplication();
                //     // senderAppNode->StopApplication();
                //     senderAppNodes.push_back(senderAppNode);
                // }
                // Ptr<CustomMessageSender> senderAppNode1 = CreateObject<CustomMessageSender>();
                // senderAppNode1->Setup(wifiNodes.Get(1)->GetObject<Socket>(), InetSocketAddress(staNodeInterfaces.GetAddress(1), port), 1024, DataRate("1Mbps"), buffer);
                // wifiNodes.Get(1)->AddApplication(senderAppNode1);
                // senderAppNode1->StartApplication();
                // senderAppNodes.push_back(senderAppNode1);
                
                Ptr<CustomMessageSender> senderAppNode = CreateObject<CustomMessageSender>();
                senderAppNode->Setup(wifiNodes.Get(2)->GetObject<Socket>(), InetSocketAddress(staNodeInterfaces.GetAddress(1), port), 1024, DataRate("1Mbps"), buffer);
                wifiNodes.Get(2)->AddApplication(senderAppNode);
                senderAppNode->StartApplication();
                senderAppNodes.push_back(senderAppNode);

                // Respond to the client who sent the message
                std::string response = std::string(buffer);
                sendto(sockfd, response.c_str(), response.length(), MSG_CONFIRM, (const struct sockaddr *)&cliaddr, len);

                // clock_t currentTime = clock();
                // double elapsedTime = static_cast<double>(currentTime - startTime) / CLOCKS_PER_SEC; // Calculate the elapsed time in seconds
    
                // if (elapsedTime >= 10.0) {
                //     break;
                // }
            }
    
            else if (msg[0] == "VEHICLE") {
                vehicle_number = std::stoi(msg[1]);
                // Resize the outer list to hold the number of vehicles
                vehicleCoordinates.resize(vehicle_number);
                wifiNodes.Create (vehicle_number+1);

                // Configuration de la mobilité des nœuds
                MobilityHelper mobility;
                mobility.SetMobilityModel ("ns3::WaypointMobilityModel");
                mobility.Install (wifiNodes);

                // Configuration du module Wifi
                WifiHelper wifi;
                wifi.SetStandard (WIFI_PHY_STANDARD_80211g);
                wifi.SetRemoteStationManager ("ns3::AarfWifiManager");
                
                // Configuration du canal Wifi
                YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
                // Configuration du module PHY Wifi, qui permet de configurer les caractéristiques de la couche physique
                YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
                phy.SetChannel (channel.Create ());

                // Configuration du module MAC Wifi qui permet de configurer les caractéristiques de la couche MAC
                WifiMacHelper mac;
                Ssid ssid = Ssid ("ns-3-ssid");
                mac.SetType ("ns3::StaWifiMac",
                            "Ssid", SsidValue (ssid),
                            "ActiveProbing", BooleanValue (false));

                WifiMacHelper macAp, macSta;

                macAp.SetType("ns3::ApWifiMac",
                            "Ssid", SsidValue(ssid));

                macSta.SetType("ns3::StaWifiMac",
                            "Ssid", SsidValue(ssid),
                            "ActiveProbing", BooleanValue(false));

                // Configuration des périphériques réseau
                NetDeviceContainer apDevice, staDevices;
                apDevice = wifi.Install(phy, macAp, wifiNodes.Get(0));
    
                for (int i = 0; i < vehicle_number; i++)
                {
                    // Configuration des périphériques réseau
                    std::cout << "Installed Wifi for node " << i+1 << std::endl;
                    if (i == 0) {
                        staDevices = wifi.Install(phy, macSta, wifiNodes.Get(i+1));
                    }
                    else {
                        staDevices.Add(wifi.Install(phy, macSta, wifiNodes.Get(i+1)));
                    }
                }
                // Configuration de la pile IP qui permet de configurer les protocoles de la couche réseau
                InternetStackHelper stack;
                stack.Install(wifiNodes);

                // Configuration des adresses IP
                Ipv4AddressHelper address;
                address.SetBase("10.1.1.0", "255.255.255.0");
                apInterface = address.Assign(apDevice);
                staNodeInterfaces = address.Assign(staDevices);

                // Configuration de l'interface d'animation
                AnimationInterface anim("wifi-animation.xml");

                // Configuration du port
                Address localAddress(InetSocketAddress(Ipv4Address::GetAny(), port));

                // CRÉATION D'INSTANCES RECEIVER
                for (int i = 0; i < vehicle_number; i++){
                    // Configuration des récepteurs
                    Ptr<CustomMessageReceiver> receiverAppNode = CreateObject<CustomMessageReceiver>();
                    receiverAppNode->Setup(Ipv4Address::GetAny(), port);
                    wifiNodes.Get(i+1)->AddApplication(receiverAppNode);
                    receiverAppNode->StartApplication();
                    receiverAppNodes.push_back(receiverAppNode);
                }

                // Configuration du serveur et du client UDP
                UdpEchoClientHelper client(apInterface.GetAddress(0), port);
                client.SetAttribute("MaxPackets", UintegerValue(1));
                client.SetAttribute("Interval", TimeValue(Seconds(1.0)));
                client.SetAttribute("PacketSize", UintegerValue(1024)); 
                
                // Configuration du serveur qui écoute les paquets entrants sur le port 9
                for (uint32_t i = 2; i < wifiNodes.GetN(); ++i) {
                    ApplicationContainer clientApp = client.Install(wifiNodes.Get(i));
                    clientApp.Start(Seconds(1.0)); // Ajustez selon le besoin
                    clientApp.Stop(Seconds(10.0)); // Ajustez selon le besoin
                }
            }

            std::cout << "Message Received: " << buffer << std::endl;
        }

        close(sockfd);
        positionCallback(wifiNodes, vehicle_number, vehicleCoordinates, map_width, map_height);

        for (const auto& sender : senderAppNodes) {
            sender->StopApplication();
        }
        for (const auto& receiver : receiverAppNodes) {
            receiver->StopApplication();
        }
        AnimationInterface anim ("async-wifi-animation.xml");

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
    std::this_thread::sleep_for(std::chrono::seconds(60));

    std::cout << "Stopping server..." << std::endl;
    server.stop();
    
    return 0;
}