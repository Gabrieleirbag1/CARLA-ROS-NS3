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

NS_LOG_COMPONENT_DEFINE ("WifiSimpleExample");

int main (int argc, char *argv[])
{
    double xCoordinates[31] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0, 231.0};
    double yCoordinates[31] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0, 231.0};

    NodeContainer wifiNodes;
    wifiNodes.Create (2);

    MobilityHelper mobility;
    mobility.SetMobilityModel ("ns3::WaypointMobilityModel");
    mobility.Install (wifiNodes);

    Ptr<WaypointMobilityModel> waypointMobility = wifiNodes.Get (0)->GetObject<WaypointMobilityModel> ();

    for (int i = 0; i < 31; i++)
    {
        Waypoint waypoint (Seconds (i), Vector (xCoordinates[i], yCoordinates[i], 0));
        waypointMobility->AddWaypoint (waypoint);
    }

    WifiHelper wifi;
    wifi.SetStandard (WIFI_PHY_STANDARD_80211g);
    wifi.SetRemoteStationManager ("ns3::AarfWifiManager");

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
    YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
    phy.SetChannel (channel.Create ());

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

    NetDeviceContainer apDevice, staDevices;
    apDevice = wifi.Install(phy, macAp, wifiNodes.Get(0));
    staDevices = wifi.Install(phy, macSta, wifiNodes.Get(1));

    InternetStackHelper stack;
    stack.Install(wifiNodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterface, staNodeInterfaces;
    apInterface = address.Assign(apDevice);
    staNodeInterfaces = address.Assign(staDevices);

    AnimationInterface anim("wifi-animation.xml");

    uint16_t port = 9;
    UdpEchoServerHelper server(port);
    ApplicationContainer serverApps = server.Install(wifiNodes.Get(0));
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(20.0));

    UdpEchoClientHelper client(apInterface.GetAddress(0), port);
    client.SetAttribute("MaxPackets", UintegerValue(1));
    client.SetAttribute("Interval", TimeValue(Seconds(1.0)));
    client.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer clientApps = client.Install(wifiNodes.Get(1));
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(20.0));

    Simulator::Stop(Seconds(20.0));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}