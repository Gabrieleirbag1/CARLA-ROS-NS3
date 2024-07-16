#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/netanim-module.h"
#include "ns3/applications-module.h" // Include this header for UdpEchoServerHelper and UdpEchoClientHelper

using namespace ns3;

// Add logging functionality to see the message content
NS_LOG_COMPONENT_DEFINE("WifiSimpleExample");

class CustomMessageReceiver : public Application 
{
public:
    CustomMessageReceiver();
    virtual ~CustomMessageReceiver();
    void Setup(Address address, uint16_t port);

protected:
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
            NS_LOG_UNCOND("Received a packet");
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
    void Setup(Ptr<Socket> socket, Address address, uint32_t packetSize, DataRate dataRate);

private:
    virtual void StartApplication(void);
    virtual void StopApplication(void);

    void ScheduleTx(void);
    void SendPacket(void);

    Ptr<Socket> m_socket;
    Address m_peer;
    uint32_t m_packetSize;
    DataRate m_dataRate;
    EventId m_sendEvent;
    bool m_running;
    uint32_t m_packetsSent;
};

CustomMessageSender::CustomMessageSender()
  : m_socket(0), 
    m_peer(), 
    m_packetSize(0), 
    m_dataRate(0), 
    m_sendEvent(), 
    m_running(false), 
    m_packetsSent(0) 
{
}

CustomMessageSender::~CustomMessageSender()
{
    m_socket = 0;
}

void CustomMessageSender::Setup(Ptr<Socket> socket, Address address, uint32_t packetSize, DataRate dataRate)
{
    m_socket = socket;
    m_peer = address;
    m_packetSize = packetSize;
    m_dataRate = dataRate;
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
    SendPacket();
}

void CustomMessageSender::StopApplication(void)
{
    m_running = false;
    if (m_sendEvent.IsRunning())
    {
        Simulator::Cancel(m_sendEvent);
    }
    if (m_socket)
    {
        m_socket->Close();
    }
}

void CustomMessageSender::SendPacket(void)
{
    std::ostringstream msgStream;
    // Example: Sending the first waypoint coordinates
    msgStream << "Coordinates: X=" << 12.0 << ", Y=" << 12.0;
    std::string message = msgStream.str();
    Ptr<Packet> packet = Create<Packet>((uint8_t*)message.c_str(), message.length());
    m_socket->Send(packet);

    NS_LOG_UNCOND("Sending packet with message: " << message);

    if (m_running)
    {
        Time tNext(Seconds(m_packetSize * 8 / static_cast<double>(m_dataRate.GetBitRate())));
        m_sendEvent = Simulator::Schedule(tNext, &CustomMessageSender::SendPacket, this);
    }
}

void CustomMessageSender::ScheduleTx(void)
{
    if (m_running)
    {
        Time tNext(Seconds(m_packetSize * 8 / static_cast<double>(m_dataRate.GetBitRate())));
        m_sendEvent = Simulator::Schedule(tNext, &CustomMessageSender::SendPacket, this);
    }
}

int main (int argc, char *argv[])
{
    double xCoordinates[31] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0, 231.0};
    double yCoordinates[31] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0, 231.0};

    // Création de deux nœuds
    NodeContainer wifiNodes;
    wifiNodes.Create (3);

    // Configuration de la mobilité des nœuds
    MobilityHelper mobility;
    mobility.SetMobilityModel ("ns3::WaypointMobilityModel");
    mobility.Install (wifiNodes);


    Ptr<WaypointMobilityModel> waypointMobility = wifiNodes.Get (1)->GetObject<WaypointMobilityModel> ();

    for (int i = 0; i < 31; i++)
    {
        Waypoint waypoint (Seconds (i), Vector (xCoordinates[i], yCoordinates[i], 0));
        waypointMobility->AddWaypoint (waypoint);
    }

    Ptr<WaypointMobilityModel> waypointMobility2 = wifiNodes.Get (2)->GetObject<WaypointMobilityModel> ();

    for (int i = 0; i < 31; i++)
    {
        Waypoint waypoint (Seconds (i), Vector (xCoordinates[i]+1, yCoordinates[i]+1, 0));
        waypointMobility2->AddWaypoint (waypoint);
    }

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
    staDevices = wifi.Install(phy, macSta, wifiNodes.Get(1));
    staDevices.Add(wifi.Install(phy, macSta, wifiNodes.Get(2)));

    // Configuration de la pile IP qui permet de configurer les protocoles de la couche réseau
    InternetStackHelper stack;
    stack.Install(wifiNodes);

    // Configuration des adresses IP
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterface, staNodeInterfaces;
    apInterface = address.Assign(apDevice);
    staNodeInterfaces = address.Assign(staDevices);

    // Configuration de l'interface d'animation
    AnimationInterface anim("wifi-animation.xml");

    // Configuration du port
    uint16_t port = 9;
    Address localAddress(InetSocketAddress(Ipv4Address::GetAny(), port));

    // CRÉATION D'INSTANCE RECEIVER ET SENDER
    // Configuration des récepteurs
    Ptr<CustomMessageReceiver> receiverAppNode1 = CreateObject<CustomMessageReceiver>();
    receiverAppNode1->Setup(Ipv4Address::GetAny(), port);
    wifiNodes.Get(1)->AddApplication(receiverAppNode1);
    receiverAppNode1->SetStartTime(Seconds(1.0));
    receiverAppNode1->SetStopTime(Seconds(20.0));

    Ptr<CustomMessageReceiver> receiverAppNode2 = CreateObject<CustomMessageReceiver>();
    receiverAppNode2->Setup(Ipv4Address::GetAny(), port);
    wifiNodes.Get(2)->AddApplication(receiverAppNode2);
    receiverAppNode2->SetStartTime(Seconds(1.0));
    receiverAppNode2->SetStopTime(Seconds(20.0));

    // Configuration des émetteurs
    Ptr<CustomMessageSender> senderAppNode1 = CreateObject<CustomMessageSender>();
    senderAppNode1->Setup(wifiNodes.Get(1)->GetObject<Socket>(), InetSocketAddress(staNodeInterfaces.GetAddress(0), port), 1024, DataRate("1Mbps"));
    wifiNodes.Get(1)->AddApplication(senderAppNode1);
    senderAppNode1->SetStartTime(Seconds(2.0));
    senderAppNode1->SetStopTime(Seconds(20.0));

    Ptr<CustomMessageSender> senderAppNode2 = CreateObject<CustomMessageSender>();
    senderAppNode2->Setup(wifiNodes.Get(2)->GetObject<Socket>(), InetSocketAddress(staNodeInterfaces.GetAddress(1), port), 1024, DataRate("1Mbps"));
    wifiNodes.Get(2)->AddApplication(senderAppNode2);
    senderAppNode2->SetStartTime(Seconds(2.0));
    senderAppNode2->SetStopTime(Seconds(20.0));

    // Configuration du serveur et du client UDP
    UdpEchoClientHelper client(apInterface.GetAddress(0), port);
    client.SetAttribute("MaxPackets", UintegerValue(1));
    client.SetAttribute("Interval", TimeValue(Seconds(1.0)));
    client.SetAttribute("PacketSize", UintegerValue(1024));
    
    // Configuration du serveur qui écoute les paquets entrants sur le port 9
    ApplicationContainer clientApps = client.Install(wifiNodes.Get(1));
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(20.0));

    Simulator::Stop(Seconds(20.0));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}