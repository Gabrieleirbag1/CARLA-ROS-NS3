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
    NodeContainer nodes, 
    std::tuple<double, double, double> coordinates, 
    int id, 
    double map_x, 
    double map_y, 
    double map_z,
    double elapsedTime
    )

{   
    int int_elapsedTime = static_cast<int>(elapsedTime);
    std::cout << "Position Callback" << std::get<0>(coordinates) - map_x << std::endl;
    std::cout << "Position Callback" << std::get<1>(coordinates) - map_y << std::endl;
    std::cout << "Position Callback" << std::get<2>(coordinates) - map_z << std::endl;

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

                std::cout << "Elapsed Time: " << elapsedTime << std::endl;
                std::tuple<double, double, double> coordinates = std::make_tuple(x, y, z);

                positionCallback(wifiNodes, coordinates, id, map_x, map_y, map_z, elapsedTime);

                // Configuration des émetteurs
                for (uint32_t i = 1; i < wifiNodes.GetN(); ++i) {
                    if (i == id+1) {
                        std::cout << "sender" << i << std::endl;
                        continue;
                    }
                    
                    Ptr<CustomMessageSender> senderAppNode = CreateObject<CustomMessageSender>();
                    senderAppNode->Setup(wifiNodes.Get(i)->GetObject<Socket>(), InetSocketAddress(staNodeInterfaces.GetAddress(i-1), port), 1024, DataRate("1Mbps"), buffer);
                    wifiNodes.Get(i)->AddApplication(senderAppNode);

                    senderAppNode->StartApplication();
                    // senderAppNode->StopApplication();
                    senderAppNodes.push_back(senderAppNode);
                }

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

        }

        close(sockfd);

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