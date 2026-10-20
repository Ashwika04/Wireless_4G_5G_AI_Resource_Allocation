#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"

// 5G NR
#include "ns3/nr-module.h"
#include "ns3/nr-point-to-point-epc-helper.h"

#include <fstream>
#include <map>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SmartCity5G");

int main(int argc, char *argv[])
{
    // ================= PARAMETERS =================
    uint16_t numberOfUes = 50;
    double simTime = 20.0;

    CommandLine cmd;
    cmd.AddValue("nUes", "Number of UEs", numberOfUes);
    cmd.Parse(argc, argv);

    uint32_t group1 = numberOfUes / 3;
    uint32_t group2 = 2 * numberOfUes / 3;

    // ================= NR + EPC =================
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();

    nrHelper->SetEpcHelper(epcHelper);

    // Scheduler (PF default)
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaPF"));

    // ================= BAND CONFIG =================
    CcBwpCreator ccBwpCreator;

    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 100e6, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    nrHelper->InitializeOperationBand(&band);

    auto allBwps = CcBwpCreator::GetAllBwps({band});

    // ================= NODES =================
    NodeContainer gnbNodes;
    gnbNodes.Create(2);

    NodeContainer ueNodes;
    ueNodes.Create(numberOfUes);

    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    // ================= MOBILITY =================
    MobilityHelper mobility;

    Ptr<ListPositionAllocator> gnbPos = CreateObject<ListPositionAllocator>();
    gnbPos->Add(Vector(250,250,0));
    gnbPos->Add(Vector(750,250,0));

    mobility.SetPositionAllocator(gnbPos);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbNodes);

    Ptr<RandomRectanglePositionAllocator> uePos = CreateObject<RandomRectanglePositionAllocator>();
    uePos->SetAttribute("X", StringValue("ns3::UniformRandomVariable[Min=0|Max=1000]"));
    uePos->SetAttribute("Y", StringValue("ns3::UniformRandomVariable[Min=0|Max=500]"));

    mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
        "Speed", StringValue("ns3::UniformRandomVariable[Min=5|Max=20]"),
        "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0]"),
        "PositionAllocator", PointerValue(uePos));

    mobility.Install(ueNodes);

    // ================= INTERNET =================
    InternetStackHelper internet;
    internet.Install(ueNodes);
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", StringValue("100Gbps"));
    p2ph.SetChannelAttribute("Delay", StringValue("10ms"));

    NetDeviceContainer internetDevices =
        p2ph.Install(epcHelper->GetPgwNode(), remoteHost);

    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0","255.0.0.0");

    Ipv4InterfaceContainer internetIpIfaces =
        ipv4h.Assign(internetDevices);

    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);

    // ================= DEVICES =================
    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs  = nrHelper->InstallUeDevice(ueNodes, allBwps);

    Ipv4InterfaceContainer ueIpIface =
        epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    std::map<Ipv4Address, uint32_t> ipToUeMap;

    for (uint32_t u = 0; u < numberOfUes; ++u)
    {
        ipToUeMap[ueIpIface.GetAddress(u)] = u;
    }

    // Attach
    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

    // ================= ROUTING =================
    Ipv4StaticRoutingHelper ipv4RoutingHelper;

    for (uint32_t u = 0; u < numberOfUes; ++u)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(u)->GetObject<Ipv4>());

        ueStaticRouting->SetDefaultRoute(
            epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());

    remoteHostStaticRouting->AddNetworkRouteTo(
        Ipv4Address("7.0.0.0"),
        Ipv4Mask("255.0.0.0"),
        1);

    // ================= TRAFFIC =================
    uint16_t dlPort = 1000;
    uint16_t ulPort = 2000;

    PacketSinkHelper ulSink("ns3::UdpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), ulPort));
    ulSink.Install(remoteHost).Start(Seconds(1.0));

    for(uint32_t u = 0; u < numberOfUes; ++u)
    {
        int interval = (u < group1) ? 100 : (u < group2 ? 10 : 20);
        int size     = (u < group1) ? 200 : (u < group2 ? 1024 : 512);

        // DL
        UdpClientHelper dlClient(ueIpIface.GetAddress(u), dlPort);
        dlClient.SetAttribute("MaxPackets", UintegerValue(1000000));
        dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(interval)));
        dlClient.SetAttribute("PacketSize", UintegerValue(size));
        dlClient.Install(remoteHost).Start(Seconds(1.0));

        PacketSinkHelper dlSink("ns3::UdpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        dlSink.Install(ueNodes.Get(u)).Start(Seconds(1.0));

        // UL
        UdpClientHelper ulClient(remoteHostAddr, ulPort);
        ulClient.SetAttribute("MaxPackets", UintegerValue(1000000));
        ulClient.SetAttribute("Interval", TimeValue(MilliSeconds(interval)));
        ulClient.SetAttribute("PacketSize", UintegerValue(size));
        ulClient.Install(ueNodes.Get(u)).Start(Seconds(1.0));
    }

    // ================= FLOW MONITOR =================
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    monitor->CheckForLostPackets();

    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());

    auto stats = monitor->GetFlowStats();

    std::ofstream outFile("5G_Results.csv");

    outFile << "FlowID,UE_ID,TrafficClass,Throughput(Mbps),Delay(ms),Jitter(ms)\n";

    for (auto &flow : stats)
    {
        auto t = classifier->FindFlow(flow.first);

        int ueId = -1;

        if (ipToUeMap.count(t.sourceAddress))
            ueId = ipToUeMap[t.sourceAddress];
        else if (ipToUeMap.count(t.destinationAddress))
            ueId = ipToUeMap[t.destinationAddress];

        int trafficClass = (ueId < group1) ? 0 :
                           (ueId < group2 ? 1 : 2);

        double throughput = flow.second.rxBytes * 8.0 /
            (flow.second.timeLastRxPacket.GetSeconds() -
             flow.second.timeFirstTxPacket.GetSeconds()) / 1e6;

        double delay = flow.second.delaySum.GetSeconds() /
                       flow.second.rxPackets * 1000;

        double jitter = flow.second.jitterSum.GetSeconds() /
                        flow.second.rxPackets * 1000;

        outFile << flow.first << ","
                << ueId << ","
                << trafficClass << ","
                << throughput << ","
                << delay << ","
                << jitter << "\n";
    }

    outFile.close();

    Simulator::Destroy();
    return 0;
}