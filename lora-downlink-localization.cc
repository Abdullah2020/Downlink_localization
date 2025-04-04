/*
 * Copyright (c) 2017 University of Padova
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Davide Magrin <magrinda@dei.unipd.it>
 */

/*
 * This script simulates a complex scenario with multiple gateways and end
 * devices. The metric of interest for this script is the throughput of the
 * network.
 */

 #include "ns3/building-allocator.h"
 #include "ns3/building-penetration-loss.h"
 #include "ns3/buildings-helper.h"
 #include "ns3/class-a-end-device-lorawan-mac.h"
 #include "ns3/command-line.h"
 #include "ns3/constant-position-mobility-model.h"
 #include "ns3/correlated-shadowing-propagation-loss-model.h"
 #include "ns3/double.h"
 #include "ns3/end-device-lora-phy.h"
 #include "ns3/forwarder-helper.h"
 #include "ns3/gateway-lora-phy.h"
 #include "ns3/gateway-lorawan-mac.h"
 #include "ns3/log.h"
 #include "ns3/lora-helper.h"
 #include "ns3/mobility-helper.h"
 #include "ns3/network-server-helper.h"
 #include "ns3/node-container.h"
 #include "ns3/periodic-sender-helper.h"
 #include "ns3/pointer.h"
 #include "ns3/position-allocator.h"
 #include "ns3/random-variable-stream.h"
 #include "ns3/propagation-loss-model.h"

 #include "ns3/simulator.h"
 #include "ns3/netanim-module.h"
 
 #include <algorithm>
 #include <ctime>

 #include <map>
 #include <set>

 #include <cmath>
 #include <fstream>

 #include <sys/stat.h>
 #include <sys/types.h>


 using namespace ns3;
 using namespace lorawan;


 // Unique 2D vector struct to avoid name conflict
struct MyVector2D {
    double x, y;
    MyVector2D() : x(0), y(0) {}
    MyVector2D(double _x, double _y) : x(_x), y(_y) {}
};

// Struct to store GW location and distance for trilateration
struct BeaconData {
    MyVector2D gwPos;
    double distance;
};

 // Maps each ED index to a list of gateway measurements
 std::map<uint32_t, std::vector<BeaconData>> edToBeaconData;

 // Maps each ED index to a set of gateway indices
 std::map<uint32_t, std::set<uint32_t>> edToGwsMap;

 
 NS_LOG_COMPONENT_DEFINE("ComplexLorawanNetworkExample");
 
 // Network settings
 int nDevices = 100;                 //!< Number of end device nodes to create
 int nGateways = 3;                  //!< Number of gateway nodes to create
 double radiusMeters = 1000;         //!< Radius (m) of the deployment
 double simulationTimeSeconds = 600; //!< Scenario duration (s) in simulated time
 
 // Channel model
 bool realisticChannelModel = true; //false; //!< Whether to use a more realistic channel model with
                                     //!< Buildings and correlated shadowing
 
 // int appPeriodSeconds = 600; //!< Duration (s) of the inter-transmission time of end devices
 int appPeriodSeconds = 600; //!< Duration (s) of the inter-transmission time of end devices
 
 // Output control
 bool printBuildingInfo = true; //!< Whether to print building information
 

 MyVector2D TrilateratePosition(const std::vector<MyVector2D>& gwPositions, const std::vector<double>& distances);   // Function prototype


void SendBeaconFromAllGateways(ns3::NodeContainer gateways,
    ns3::NodeContainer endDevices,
    ns3::Ptr<ns3::lorawan::LoraChannel> channel,
    ns3::Time interval)
{


    // Initialize the normal random variable for noise
    Ptr<NormalRandomVariable> noise = CreateObject<NormalRandomVariable>();
    noise->SetAttribute("Mean", DoubleValue(0.0));
    noise->SetAttribute("Variance", DoubleValue(4.0)); // 2 dB standard deviation
    

    double sensitivityThresholdDbm = -120.0;    // Example sensitivity threshold find here: https://lora.readthedocs.io/en/latest/

    ns3::Time now = ns3::Simulator::Now();
    NS_LOG_INFO("Sending downlink beacon at " << now.GetSeconds() << "s");

    // Open detailed log file
    std::ofstream detailLog("simulation_output/downlink-beacon-metrics-with-noise.txt", std::ios::app);
    detailLog << "===== Beacon Round @ " << now.GetSeconds() << "s =====" << std::endl;

    // Open PDR summary log file
    std::ofstream pdrLog("simulation_output/downlink-pdr.txt", std::ios::app);
    pdrLog << now.GetSeconds() << "s";

    for (uint32_t gwIndex = 0; gwIndex < gateways.GetN(); ++gwIndex)
    {
        ns3::Ptr<ns3::Node> gw = gateways.Get(gwIndex);
        ns3::Ptr<ns3::MobilityModel> gwMob = gw->GetObject<ns3::MobilityModel>();
        ns3::Vector gwPos = gwMob->GetPosition();

        ns3::Ptr<ns3::Packet> beaconPacket = ns3::Create<ns3::Packet>(16);
        int edsReceived = 0;

        for (uint32_t edIndex = 0; edIndex < endDevices.GetN(); ++edIndex)
        {
            ns3::Ptr<ns3::Node> ed = endDevices.Get(edIndex);
            ns3::Ptr<ns3::MobilityModel> edMob = ed->GetObject<ns3::MobilityModel>();

            double txPowerDbm = 14.0;
            double rxPowerDbm = channel->GetRxPower(txPowerDbm, gwMob, edMob);

            // Add Gaussian noise to the received power
            double noiseDb = noise->GetValue();
            rxPowerDbm += noiseDb;

            // Calculate distance
            double distance = ns3::CalculateDistance(gwMob->GetPosition(), edMob->GetPosition());

            if (rxPowerDbm > sensitivityThresholdDbm)
                {
                    edsReceived++;

                    edToGwsMap[edIndex].insert(gwIndex);  // Store the mapping of end device to gateway

                    edToBeaconData[edIndex].push_back({MyVector2D(gwPos.x, gwPos.y), distance});

                    detailLog << "GW " << gwIndex
                        << " [x=" << gwPos.x << ", y=" << gwPos.y << ", z=" << gwPos.z << "]"
                        << " → ED " << edIndex
                        << " | Distance: " << distance << " m"
                        << " | RSSI: " << rxPowerDbm << " dBm"
                        << " | Packet Size: " << beaconPacket->GetSize() << " bytes"
                        << " | Noise: " << noiseDb << " dBm"
                        << " | Tx power: " << txPowerDbm << " dBm"
                        << std::endl;
                }

        }

        double pdr = static_cast<double>(edsReceived) / endDevices.GetN();

        detailLog << "→ GW " << gwIndex << " Beacon PDR: " << edsReceived << "/" << endDevices.GetN()
            << " (" << (pdr * 100.0) << "%)" << std::endl << std::endl;

        // Log only summary to PDR file
        pdrLog << ", GW" << gwIndex << ": " << (pdr * 100.0) << "%";
    }

    pdrLog << std::endl;

    detailLog.close();
    pdrLog.close();

    // Schedule next beacon
    ns3::Simulator::Schedule(interval,
                             &SendBeaconFromAllGateways,
                             gateways,
                             endDevices,
                             channel,
                             interval);
}




// Let the user know how to compute the average downlink PDR

void ComputeAverageDownlinkPDR(std::string pdrFilename, uint32_t nGateways)
{
    std::ifstream file(pdrFilename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open " << pdrFilename << std::endl;
        return;
    }

    std::vector<double> totalPdr(nGateways, 0.0);
    std::vector<int> count(nGateways, 0);

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string token;
        while (std::getline(iss, token, ','))
        {
            for (uint32_t gwIndex = 0; gwIndex < nGateways; ++gwIndex)
            {
                std::string prefix = " GW" + std::to_string(gwIndex) + ": ";
                size_t pos = token.find(prefix);
                if (pos != std::string::npos)
                {
                    std::string percentStr = token.substr(pos + prefix.length());
                    percentStr.erase(std::remove(percentStr.begin(), percentStr.end(), '%'), percentStr.end());
                    double pdrValue = std::stod(percentStr);
                    totalPdr[gwIndex] += pdrValue;
                    count[gwIndex]++;
                }
            }
        }
    }

    file.close();

    // Print to console
    std::cout << "\n==== Average Downlink PDR per Gateway ====" << std::endl;

    // Write to file
    std::ofstream outFile("simulation_output/avg-pdr-with-noise.txt");
    if (!outFile.is_open())
    {
        std::cerr << "Failed to open avg-pdr-with-noise.txt for writing." << std::endl;
        return;
    }

    outFile << "Average Downlink PDR per Gateway\n";

    for (uint32_t gwIndex = 0; gwIndex < nGateways; ++gwIndex)
    {
        double avg = (count[gwIndex] > 0) ? totalPdr[gwIndex] / count[gwIndex] : 0.0;
        std::cout << "GW" << gwIndex << ": " << avg << " %" << std::endl;
        outFile << "GW" << gwIndex << ": " << avg << " %" << std::endl;
    }

    outFile.close();
}


 
 int
 main(int argc, char* argv[])
 {



     // Create the "simulation_output" folder if it doesn't exist
     const char* outputFolder = "simulation_output";
     mkdir(outputFolder, 0777); // 0777 gives full permissions


     // Set the input arguments parameters
     CommandLine cmd(__FILE__);
     cmd.AddValue("nDevices", "Number of end devices to include in the simulation", nDevices);
     cmd.AddValue("radius", "The radius (m) of the area to simulate", radiusMeters);
     cmd.AddValue("realisticChannel",
                  "Whether to use a more realistic channel model",
                  realisticChannelModel);
     cmd.AddValue("simulationTime", "The time (s) for which to simulate", simulationTimeSeconds);
     cmd.AddValue("appPeriod",
                  "The period in seconds to be used by periodically transmitting applications",
                  appPeriodSeconds);
     cmd.AddValue("print", "Whether or not to print building information", printBuildingInfo);
     cmd.Parse(argc, argv);
 
     std::cout << "Simulation Configuration Parameters:" << std::endl;
     std::cout << "------------------------------------" << std::endl;
     std::cout << "Number of End Devices: " << nDevices << std::endl;
     std::cout << "Number of Gateways: " << nGateways << std::endl;
     std::cout << "Deployment Radius (meters): " << radiusMeters << std::endl;
     std::cout << "Simulation Time (seconds): " << simulationTimeSeconds << std::endl;
     std::cout << "Application Period (seconds): " << appPeriodSeconds << std::endl;
     std::cout << "Realistic Channel Model: " << (realisticChannelModel ? "Enabled" : "Disabled") << std::endl;
     std::cout << "Print Building Information: " << (printBuildingInfo ? "Yes" : "No") << std::endl;
     std::cout << "------------------------------------" << std::endl;
 
     // Set up logging
     LogComponentEnable("ComplexLorawanNetworkExample", LOG_LEVEL_ALL);
     LogComponentEnable("LoraChannel", LOG_LEVEL_INFO);
     // LogComponentEnable("LoraPhy", LOG_LEVEL_ALL);
     // LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_ALL);
     // LogComponentEnable("GatewayLoraPhy", LOG_LEVEL_ALL);
     // LogComponentEnable("LoraInterferenceHelper", LOG_LEVEL_ALL);
     // LogComponentEnable("LorawanMac", LOG_LEVEL_ALL);
     // LogComponentEnable("EndDeviceLorawanMac", LOG_LEVEL_ALL);
    //  LogComponentEnable("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
     // LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
     // LogComponentEnable("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
     // LogComponentEnable("LogicalLoraChannel", LOG_LEVEL_ALL);
     // LogComponentEnable("LoraHelper", LOG_LEVEL_ALL);
     // LogComponentEnable("LoraPhyHelper", LOG_LEVEL_ALL);
     // LogComponentEnable("LorawanMacHelper", LOG_LEVEL_ALL);
     // LogComponentEnable("PeriodicSenderHelper", LOG_LEVEL_ALL);
     // LogComponentEnable("PeriodicSender", LOG_LEVEL_ALL);
     // LogComponentEnable("LorawanMacHeader", LOG_LEVEL_ALL);
     // LogComponentEnable("LoraFrameHeader", LOG_LEVEL_ALL);
     // LogComponentEnable("NetworkScheduler", LOG_LEVEL_ALL);
     // LogComponentEnable("NetworkServer", LOG_LEVEL_ALL);
     // LogComponentEnable("NetworkStatus", LOG_LEVEL_ALL);
     // LogComponentEnable("NetworkController", LOG_LEVEL_ALL);
 
     /***********
      *  Setup  *
      ***********/
 
     // Create the time value from the period
     Time appPeriod = Seconds(appPeriodSeconds);
 
     // Mobility
     MobilityHelper mobility;
     mobility.SetPositionAllocator("ns3::UniformDiscPositionAllocator",
                                   "rho", DoubleValue(radiusMeters),
                                   "X", DoubleValue(0.0),
                                   "Y", DoubleValue(0.0));
 
     // mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
     mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
         "Mode", StringValue("Time"),
         "Time", TimeValue(Seconds(1.0)),
         "Speed", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"),
         "Bounds", RectangleValue(Rectangle(-radiusMeters, radiusMeters, -radiusMeters, radiusMeters)));
 
 
     /************************
      *  Create the channel  *
      ************************/
 
     // Create the lora channel object
     Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
     loss->SetPathLossExponent(3.76);
     loss->SetReference(1, 7.7);
 
     if (realisticChannelModel)
     {
         // Create the correlated shadowing component
         Ptr<CorrelatedShadowingPropagationLossModel> shadowing =
             CreateObject<CorrelatedShadowingPropagationLossModel>();
 
         // Aggregate shadowing to the logdistance loss
         loss->SetNext(shadowing);
 
         // Add the effect to the channel propagation loss
         Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss>();
 
         shadowing->SetNext(buildingLoss);
     }
 
     Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();
 
     Ptr<LoraChannel> channel = CreateObject<LoraChannel>(loss, delay);
 
     /************************
      *  Create the helpers  *
      ************************/
 
     // Create the LoraPhyHelper
     LoraPhyHelper phyHelper = LoraPhyHelper();
     phyHelper.SetChannel(channel);
 
     // Create the LorawanMacHelper
     LorawanMacHelper macHelper = LorawanMacHelper();
 
     // Create the LoraHelper
     LoraHelper helper = LoraHelper();
     helper.EnablePacketTracking(); // Output filename
     helper.EnableSimulationTimePrinting (Seconds(1.0));   // Print simulation time in the output file
 
     // Create the NetworkServerHelper
     NetworkServerHelper nsHelper = NetworkServerHelper();
 
     // Create the ForwarderHelper
     ForwarderHelper forHelper = ForwarderHelper();
 
     /************************
      *  Create End Devices  *
      ************************/
 
     // Create a set of nodes
     NodeContainer endDevices;
     endDevices.Create(nDevices);
 
     // Assign a mobility model to each node
     mobility.Install(endDevices);
 
     // Make it so that nodes are at a certain height > 0
     for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
     {
         Ptr<MobilityModel> mobility = (*j)->GetObject<MobilityModel>();
         Vector position = mobility->GetPosition();
         position.z = 0.2; // Altitude of the end devices: by default 1.2 m
         mobility->SetPosition(position);
     }
 
     // Create the LoraNetDevices of the end devices
     uint8_t nwkId = 54;
     uint32_t nwkAddr = 1864;
     Ptr<LoraDeviceAddressGenerator> addrGen =
         CreateObject<LoraDeviceAddressGenerator>(nwkId, nwkAddr);
 
     // Create the LoraNetDevices of the end devices
     macHelper.SetAddressGenerator(addrGen);
     phyHelper.SetDeviceType(LoraPhyHelper::ED);
     macHelper.SetDeviceType(LorawanMacHelper::ED_A);
     helper.Install(phyHelper, macHelper, endDevices);
 
     // Now end devices are connected to the channel
 
 
     /*********************
      *  Create Gateways  *
      *********************/
 
     // Create the gateway nodes (allocate them uniformly on the disc)
     NodeContainer gateways;
     gateways.Create(nGateways);
 
     Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();
     // Make it so that nodes are at a certain height > 0
     // allocator->Add(Vector(0.0, 0.0, 15.0)); // Set the position of the gateway --> Add(Vector(x, y, z))
     
    //  allocator->Add(Vector(900.0, 100.0, 15.0)); // Gateway 1 at (0, 0, 15)
    //  allocator->Add(Vector(0.0, 500.0, 15.0)); // Gateway 2 at (50, 50, 15)
    //  allocator->Add(Vector(500.0, 1000.0, 15.0)); // Gateway 3 at (-50, -50, 15)
     

    allocator->Add(Vector(0.0, 577.0, 10.0)); // Gateway 1 
    allocator->Add(Vector(-500.0, -289.0, 10.0)); // Gateway 2 
    allocator->Add(Vector(500.0, -289.0, 10.0)); // Gateway 3 at

     mobility.SetPositionAllocator(allocator);
     mobility.Install(gateways);
 
     // Create a netdevice for each gateway
     phyHelper.SetDeviceType(LoraPhyHelper::GW);
     macHelper.SetDeviceType(LorawanMacHelper::GW);
     helper.Install(phyHelper, macHelper, gateways);
 
 
     /**********************
      *  Handle buildings  *
      **********************/
 
     double xLength = 130;
     double deltaX = 32;
     double yLength = 64;
     double deltaY = 17;
     int gridWidth = 2 * radiusMeters / (xLength + deltaX);
     int gridHeight = 2 * radiusMeters / (yLength + deltaY);
     if (!realisticChannelModel)
     {
         gridWidth = 0;
         gridHeight = 0;
     }
     Ptr<GridBuildingAllocator> gridBuildingAllocator;
     gridBuildingAllocator = CreateObject<GridBuildingAllocator>();
     gridBuildingAllocator->SetAttribute("GridWidth", UintegerValue(gridWidth));
     gridBuildingAllocator->SetAttribute("LengthX", DoubleValue(xLength));
     gridBuildingAllocator->SetAttribute("LengthY", DoubleValue(yLength));
     gridBuildingAllocator->SetAttribute("DeltaX", DoubleValue(deltaX));
     gridBuildingAllocator->SetAttribute("DeltaY", DoubleValue(deltaY));
     gridBuildingAllocator->SetAttribute("Height", DoubleValue(6));
     gridBuildingAllocator->SetBuildingAttribute("NRoomsX", UintegerValue(2));
     gridBuildingAllocator->SetBuildingAttribute("NRoomsY", UintegerValue(4));
     gridBuildingAllocator->SetBuildingAttribute("NFloors", UintegerValue(2));
     gridBuildingAllocator->SetAttribute(
         "MinX",
         DoubleValue(-gridWidth * (xLength + deltaX) / 2 + deltaX / 2));
     gridBuildingAllocator->SetAttribute(
         "MinY",
         DoubleValue(-gridHeight * (yLength + deltaY) / 2 + deltaY / 2));
     BuildingContainer bContainer = gridBuildingAllocator->Create(gridWidth * gridHeight);
 
     BuildingsHelper::Install(endDevices);
     BuildingsHelper::Install(gateways);
 
     // Print the buildings
     if (printBuildingInfo)
     {
         std::ofstream myfile;
         myfile.open("simulation_output/buildings-with-noise.txt");
         std::vector<Ptr<Building>>::const_iterator it;
         int j = 1;
         for (it = bContainer.Begin(); it != bContainer.End(); ++it, ++j)
         {
             Box boundaries = (*it)->GetBoundaries();
             myfile << "set object " << j << " rect from " << boundaries.xMin << ","
                    << boundaries.yMin << " to " << boundaries.xMax << "," << boundaries.yMax
                    << std::endl;
         }
         myfile.close();
     }
 
     /**********************************************
      *  Set up the end device's spreading factor  *
      **********************************************/
 
     LorawanMacHelper::SetSpreadingFactorsUp(endDevices, gateways, channel);
 
     NS_LOG_DEBUG("Completed configuration");
 
     /*********************************************
      *  Install applications on the end devices  *
      *********************************************/
 
     Time appStopTime = Seconds(simulationTimeSeconds);
     PeriodicSenderHelper appHelper = PeriodicSenderHelper();
 
     // appHelper.SetEnableBroadcast(true); // Treat as broadcast
 
     appHelper.SetPeriod(Seconds(appPeriodSeconds));
     appHelper.SetPacketSize(23); // <--- This sets the packet size to 23 bytes
     Ptr<RandomVariableStream> rv =
         CreateObjectWithAttributes<UniformRandomVariable>("Min",
                                                           DoubleValue(0),
                                                           "Max",
                                                           DoubleValue(10));
     ApplicationContainer appContainer = appHelper.Install(endDevices);
 
     appContainer.Start(Seconds(0));
     appContainer.Stop(appStopTime);
 
     /**************************
      *  Create network server  *
      ***************************/
 
     // Create the network server node
     Ptr<Node> networkServer = CreateObject<Node>();
 
     // Assign a ConstantPositionMobilityModel to the network server
     Ptr<MobilityModel> nsMobility = CreateObject<ConstantPositionMobilityModel>();
     Ptr<Object> obj = nsMobility;
     networkServer->AggregateObject(obj);
 
 
     // PointToPoint links between gateways and server
     PointToPointHelper p2p;
     p2p.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
     p2p.SetChannelAttribute("Delay", StringValue("2ms"));
     // Store network server app registration details for later
     P2PGwRegistration_t gwRegistration;
     for (auto gw = gateways.Begin(); gw != gateways.End(); ++gw)
     {
         auto container = p2p.Install(networkServer, *gw);
         auto serverP2PNetDev = DynamicCast<PointToPointNetDevice>(container.Get(0));
         gwRegistration.emplace_back(serverP2PNetDev, *gw);
     }
 
     // Create a network server for the network
     nsHelper.SetGatewaysP2P(gwRegistration);
     nsHelper.SetEndDevices(endDevices);
     nsHelper.Install(networkServer);
 
     // Create a forwarder for each gateway
     forHelper.Install(gateways);
 
     ////////////////
     // Simulation //
     ////////////////
 
     // === NetAnim Setup ===
     AnimationInterface anim("simulation_output/lorawan-downlink-3GW-100EDs-with-noise.xml");
 
     for (uint32_t i = 0; i < endDevices.GetN(); ++i)
     {
         anim.UpdateNodeDescription(endDevices.Get(i), "ED");
         anim.UpdateNodeColor(endDevices.Get(i), 0, 255, 0); // Green
     }
 
     for (uint32_t i = 0; i < gateways.GetN(); ++i)
     {
         anim.UpdateNodeDescription(gateways.Get(i), "GW");
         anim.UpdateNodeColor(gateways.Get(i), 0, 0, 255); // Blue
     }
 
     anim.UpdateNodeDescription(networkServer, "NS");
     anim.UpdateNodeColor(networkServer, 255, 0, 0); // Red
     // === End NetAnim Setup ===


    ns3::Time beaconInterval = ns3::Seconds(60); // Beacon every 60 seconds
    ns3::Simulator::Schedule(ns3::Seconds(0),
                         &SendBeaconFromAllGateways,
                         gateways,
                         endDevices,
                         channel,
                         beaconInterval);
        // Schedule the first beacon


 
     Simulator::Stop(appStopTime + Hours(1));
 
     NS_LOG_INFO("Running simulation...");
     Simulator::Run();


     std::ofstream locFile("simulation_output/lora-localization-results-with-noise.csv");
     locFile << "ED,ActualX,ActualY,EstimatedX,EstimatedY,Error\n";

     for (const auto& [edIndex, beacons] : edToBeaconData)
        {
            if (beacons.size() < 3) continue;

            std::vector<MyVector2D> gwPositions;
            std::vector<double> distances;

            for (const auto& b : beacons)
            {
                gwPositions.push_back(b.gwPos);
                distances.push_back(b.distance);
            }

            MyVector2D estimated = TrilateratePosition(gwPositions, distances);

            Ptr<Node> ed = endDevices.Get(edIndex);
            Ptr<MobilityModel> mob = ed->GetObject<MobilityModel>();
            Vector actual = mob->GetPosition();

            double error = std::sqrt(std::pow(estimated.x - actual.x, 2) + std::pow(estimated.y - actual.y, 2));

            locFile << edIndex << ","
                    << actual.x << "," << actual.y << ","
                    << estimated.x << "," << estimated.y << ","
                    << error << "\n";
        }
        locFile.close();

     Simulator::Destroy();

 
 
     ///////////////////////////
     // Print results to file //
     ///////////////////////////
     NS_LOG_INFO("Computing performance metrics...");


     std::cout << "\n==============================================" << std::endl;
     std::cout << "🔼 Uplink Packet Statistics (MAC-layer only)" << std::endl;
     std::cout << "This value represents the number of uplink packets" << std::endl;
     std::cout << "successfully transmitted by all end devices." << std::endl;
     std::cout << "==============================================" << std::endl;

 
     LoraPacketTracker& tracker = helper.GetPacketTracker();
 
     // Count and print the total number of MAC-layer packets globally
     std::cout << "Total MAC-layer packets transmitted globally: "
             << tracker.CountMacPacketsGlobally(Seconds(0), appStopTime + Hours(1)) << std::endl;
 
     
     // Compute average downlink PDR
     ComputeAverageDownlinkPDR("downlink-pdr.txt", nGateways);


     std::cout << "\n==== End Devices Receiving Beacons from ≥ 3 Gateways ====" << std::endl;
     int coveredCount = 0;
     for (const auto& [edIndex, gwSet] : edToGwsMap)
        {
            if (gwSet.size() >= 3)
            {
                std::cout << "ED" << edIndex << " received beacons from " << gwSet.size() << " GWs" << std::endl;
                coveredCount++;
            }
        }
        std::cout << "Total EDs covered by ≥ 3 GWs: " << coveredCount << " out of " << edToGwsMap.size() << std::endl;

    
     std::cout << "------------------------------------" << std::endl;
     std::cout << "Simulation completed successfully." << std::endl;
     std::cout << "------------------------------------" << std::endl;

     return 0;
 }




// Function to trilaterate the position of an end device based on distances from GWs
 MyVector2D TrilateratePosition(
    const std::vector<MyVector2D>& gwPositions,
    const std::vector<double>& distances)
{
    uint32_t n = gwPositions.size();
    if (n < 3)
    {
        return MyVector2D(NAN, NAN); // Not enough GWs
    }

    // double A[n - 1][2], b[n - 1];
    std::vector<std::array<double, 2>> A(n - 1);
    std::vector<double> b(n - 1);


    for (uint32_t i = 1; i < n; ++i)
    {
        A[i - 1][0] = 2 * (gwPositions[i].x - gwPositions[0].x);
        A[i - 1][1] = 2 * (gwPositions[i].y - gwPositions[0].y);
        b[i - 1] = distances[0] * distances[0] - distances[i] * distances[i]
                 - gwPositions[0].x * gwPositions[0].x + gwPositions[i].x * gwPositions[i].x
                 - gwPositions[0].y * gwPositions[0].y + gwPositions[i].y * gwPositions[i].y;
    }

    double ATA[2][2] = {{0, 0}, {0, 0}};
    double ATb[2] = {0, 0};

    for (uint32_t i = 0; i < n - 1; ++i)
    {
        ATA[0][0] += A[i][0] * A[i][0];
        ATA[0][1] += A[i][0] * A[i][1];
        ATA[1][0] += A[i][1] * A[i][0];
        ATA[1][1] += A[i][1] * A[i][1];
        ATb[0] += A[i][0] * b[i];
        ATb[1] += A[i][1] * b[i];
    }

    double det = ATA[0][0] * ATA[1][1] - ATA[0][1] * ATA[1][0];
    if (fabs(det) < 1e-6)
    {
        return MyVector2D(NAN, NAN);
    }

    double invATA[2][2];
    invATA[0][0] = ATA[1][1] / det;
    invATA[0][1] = -ATA[0][1] / det;
    invATA[1][0] = -ATA[1][0] / det;
    invATA[1][1] = ATA[0][0] / det;

    MyVector2D result;
    result.x = invATA[0][0] * ATb[0] + invATA[0][1] * ATb[1];
    result.y = invATA[1][0] * ATb[0] + invATA[1][1] * ATb[1];

    return result;
}
