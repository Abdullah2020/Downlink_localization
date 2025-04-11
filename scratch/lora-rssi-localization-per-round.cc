/*
 * Copyright (c) 2025 Aiman 
 *
 * Author(s): Abdullahi Isa Ahmed, Dossa Amavi, ....
 */

/*
 * This script simulates a complex LoRaWAN scenario with multiple gateways and end
 * devices. It includes features such as downlink beaconing, localization using
 * trilateration, and performance metrics like Packet Delivery Ratio (PDR), etc....
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

 #include <filesystem> // C++17


 namespace fs = std::filesystem;

 using namespace ns3;
 using namespace lorawan;


 // Unique 2D vector struct
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

 // ns log component definition
 NS_LOG_COMPONENT_DEFINE("ComplexLorawanNetwork");
 
 // Default LoRaWAN Network settings
 int nDevices = 100;                 //!< Number of end device nodes to create
 int nGateways = 4;                  //!< Number of gateway nodes to create
 double radiusMeters = 1000;         //!< Radius (m) of the deployment
 double simulationTimeSeconds = 600; //!< Scenario duration (s) in simulated time
 
 // Channel model
 bool realisticChannelModel = false; //false; //!< Whether to use a more realistic channel model with
                                     //!< Buildings and correlated shadowing
 
 // int appPeriodSeconds = 600; //!< Duration (s) of the inter-transmission time of end devices
 int appPeriodSeconds = 600; //!< Duration (s) of the inter-transmission time of end devices
 
 // Output control
 bool printBuildingInfo = false; //!< Whether to print building information
 
 // Simulation control 
 MyVector2D TrilateratePosition(const std::vector<MyVector2D>& gwPositions, const std::vector<double>& distances);   // Function prototype

 // localization function for all devices at a specific time
 void LocalizeAllDevicesAt(ns3::Time currentTime, ns3::NodeContainer endDevices, std::string filePath);


// declare output folder as a global variables:
std::string outputFolder;
 

//  // function to estimate distance from RSSI
//  double EstimateDistanceFromRSSI(double txPowerDbm, double rxPowerDbmWithNoise)
//   {
//       // Parameters
//       double d0 = 1.0;                        // reference distance in meters
//       double pathLossAtD0 = 40; //40.0;            // PL(d0) in dB (you can tune this)
//       double pathLossExponent = 2.7;         // environment dependent
  
//       // RSSI = txPower - PL(d)
//       // => PL(d) = txPower - rxPower
//       double pathLossDb = txPowerDbm - rxPowerDbmWithNoise;
  
//       // Estimate distance
//     //   double estimatedDistance = d0 * std::pow(10.0, (pathLossDb - pathLossAtD0) / (10.0 * pathLossExponent));
//      double estimatedDistance = d0 * std::pow(10.0, (pathLossDb - pathLossAtD0) / (10.0 * pathLossExponent));
  
//       return estimatedDistance;
//   }


double EstimateDistanceFromRSSI(double txPowerDbm, double rxPowerDbmWithNoise)
{
    double d0 = 1.0;                  // Reference distance in meters
    double pathLossExponent = 2.3;    // η, environment-dependent


    // Ptr<NormalRandomVariable> shadow = CreateObject<NormalRandomVariable>();
    // shadow->SetAttribute("Mean", DoubleValue(0.0));
    // shadow->SetAttribute("Variance", DoubleValue(4.0)); // σ² = 4 → σ ≈ 2 dB

    // double X_sigma = shadow->GetValue();

    double X_sigma = 0.0;             // Shadowing 

    double exponent = (txPowerDbm + X_sigma - rxPowerDbmWithNoise) / (10.0 * pathLossExponent);
    double distance = d0 * std::pow(10.0, exponent);

    return distance;
}

  
 
  

// send beacon from a specific gateway
// This function is called by the SendBeaconFromAllGateways function
// It simulates the sending of a beacon from a specific gateway to all end devices
// It calculates the received signal strength at each end device and logs the results
// It also calculates the distance from the gateway to each end device
// The function takes the following parameters:
    // - gwIndex: The index of the gateway sending the beacon
    // - gateways: The container of gateway nodes
    // - endDevices: The container of end device nodes
    // - channel: The channel model used for transmission
    // - noise: The normal random variable used to simulate noise
    // - detailLogPath: The path to the detailed log file
    // - pdrLogPath: The path to the PDR log file
    // - now: The current simulation time
 
 void SendBeaconFromGateway(uint32_t gwIndex,
        ns3::NodeContainer gateways,
        ns3::NodeContainer endDevices,
        ns3::Ptr<ns3::lorawan::LoraChannel> channel,
        ns3::Ptr<ns3::NormalRandomVariable> noise,
        std::string detailLogPath,
        std::string pdrLogPath,
        ns3::Time now)
    {
    std::ofstream detailLog(detailLogPath, std::ios::app);
    std::ofstream pdrLog(pdrLogPath, std::ios::app);

    Ptr<Node> gw = gateways.Get(gwIndex);
    Ptr<MobilityModel> gwMob = gw->GetObject<MobilityModel>();
    Vector gwPos = gwMob->GetPosition();

    Ptr<Packet> beaconPacket = Create<Packet>(16);
    int edsReceived = 0;

    double txPowerDbm = 14.0;

    // double tx_power = 14.0;
    // double txPowerDbm = std::pow(10.0, tx_power / 10.0) / 1000.0; // Result: ~0.0251 W

    double sensitivityThresholdDbm = -137.0;

    for (uint32_t edIndex = 0; edIndex < endDevices.GetN(); ++edIndex)
    {
    Ptr<Node> ed = endDevices.Get(edIndex);
    Ptr<MobilityModel> edMob = ed->GetObject<MobilityModel>();

    double rxPowerDbm = channel->GetRxPower(txPowerDbm, gwMob, edMob);
    double noiseDb = noise->GetValue();
    rxPowerDbm += noiseDb;

    double distance = EstimateDistanceFromRSSI(txPowerDbm, rxPowerDbm);

    if (rxPowerDbm > sensitivityThresholdDbm)
    {
    edsReceived++;
    edToGwsMap[edIndex].insert(gwIndex);
    edToBeaconData[edIndex].push_back({MyVector2D(gwPos.x, gwPos.y), distance});

    detailLog << "Time: " << now.GetSeconds()
    << "s | GW " << gwIndex
    << " [x=" << gwPos.x << ", y=" << gwPos.y << "]"
    << " → ED " << edIndex
    << " | Dist: " << distance << " m"
    << " | RSSI: " << rxPowerDbm << " dBm"
    << " | Tx Power: " << txPowerDbm << " dBm"
    << std::endl;
    }
    }

    double pdr = static_cast<double>(edsReceived) / endDevices.GetN();
    detailLog << "→ GW " << gwIndex << " Beacon PDR: " << edsReceived << "/" << endDevices.GetN()
    << " (" << (pdr * 100.0) << "%)\n" << std::endl;

    pdrLog << now.GetSeconds() << "s, GW" << gwIndex << ", " << (pdr * 100.0) << "%\n";

    // Optional: Timeline log for visualization
    std::ofstream visualLog(outputFolder + "/beacon-timeline.csv", std::ios::app);
    visualLog << now.GetSeconds() << "," << gwIndex << "\n";
    visualLog.close();

    detailLog.close();
    pdrLog.close();
    }



 void SendBeaconFromAllGateways(ns3::NodeContainer gateways,
    ns3::NodeContainer endDevices,
    ns3::Ptr<ns3::lorawan::LoraChannel> channel,
    ns3::Time interval)
    {
    Ptr<NormalRandomVariable> noise = CreateObject<NormalRandomVariable>();
    noise->SetAttribute("Mean", DoubleValue(0.0));
    noise->SetAttribute("Variance", DoubleValue(4.0));

    Time now = Simulator::Now();
    NS_LOG_INFO("Scheduling beacon round at " << now.GetSeconds() << "s");

    std::ostringstream detailLogFileNameStream;
    detailLogFileNameStream << outputFolder << "/downlink-beacon-metrics-" 
    << nDevices << "EDs_" 
    << nGateways << "GWs_" 
    << radiusMeters << "m.txt";

    std::ostringstream pdrFileNameStream;
    pdrFileNameStream << outputFolder << "/downlink-pdr-" 
    << nDevices << "EDs_" 
    << nGateways << "GWs_" 
    << radiusMeters << "m.txt";

    // ⏱ Add guard time between each GW beacon transmission
    Time guardTime = MilliSeconds(100); // e.g. 100ms between gateways

    for (uint32_t gwIndex = 0; gwIndex < gateways.GetN(); ++gwIndex)
        {
            Time delay = guardTime * gwIndex;

            Simulator::Schedule(delay,
            &SendBeaconFromGateway,
            gwIndex,
            gateways,
            endDevices,
            channel,
            noise,
            detailLogFileNameStream.str(),
            pdrFileNameStream.str(),
            now + delay);
        }


    // === Schedule localization after all GWs finish beaconing ===
    Time totalDelay = guardTime * gateways.GetN(); // Total beaconing round delay
    Simulator::Schedule(now + totalDelay + MilliSeconds(50), // 50ms buffer
        &LocalizeAllDevicesAt,
        now + totalDelay,
        endDevices,
        outputFolder + "/localization-per-round.csv");


    // Schedule the next beacon round after full interval
    Simulator::Schedule(interval,
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
        return; // Indicate an error
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
    // std::ofstream outFile("simulation_output/avg-pdr-with-noise.txt");
    std::ostringstream avgPdrFileNameStream;
    avgPdrFileNameStream << outputFolder << "/avg-pdr-" 
                        << nDevices << "EDs_" 
                        << nGateways << "GWs_" 
                        << radiusMeters << "m.txt";
    std::ofstream outFile(avgPdrFileNameStream.str());


    if (!outFile.is_open())
    {
        std::cerr << "Failed to open avg-pdr-with-noise.txt for writing." << std::endl;
        return; // Indicate an error
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


 
// Function to check for expected output files
void VerifyOutputFiles(const std::string& folder, int nDevices, int nGateways, double radiusMeters)
{
    std::vector<std::string> expectedFiles = {
        "downlink-beacon-metrics-" + std::to_string(nDevices) + "EDs_" + std::to_string(nGateways) + "GWs_" + std::to_string((int)radiusMeters) + "m.txt",
        "downlink-pdr-" + std::to_string(nDevices) + "EDs_" + std::to_string(nGateways) + "GWs_" + std::to_string((int)radiusMeters) + "m.txt",
        "lora-localization-results-" + std::to_string(nDevices) + "EDs_" + std::to_string(nGateways) + "GWs_" + std::to_string((int)radiusMeters) + "m.csv",
        "Statistical-error-summary.txt",  // Fixed typo
        "localization-error-summary.txt",
        "buildings-info.txt",
        "lorawan-downlink-" + std::to_string(nGateways) + "GWs_" + std::to_string(nDevices) + "EDs_" + std::to_string((int)radiusMeters) + "m.xml"
    };

    std::cout << "\n==== Verifying Output Files ====" << std::endl;
    for (const auto& filename : expectedFiles)
    {
        std::string path = folder + "/" + filename;

        if (fs::exists(path))
        {
            auto size = fs::file_size(path);
            if (size > 0)
            {
                std::cout << "✅ File OK: " << filename << " (" << size << " bytes)" << std::endl;
            }
            else
            {
                std::cout << "⚠️  File is empty: " << filename << std::endl;
            }
        }
        else
        {
            std::cout << "❌ Missing file: " << filename << std::endl;
        }
    }
}





 int
 main(int argc, char* argv[])
 {

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
        

     std::ostringstream folderNameStream;
     std::time_t now = std::time(nullptr);
     char timeBuffer[20];
     std::strftime(timeBuffer, sizeof(timeBuffer), "%Y%m%d_%H%M%S", std::localtime(&now));
     folderNameStream << "simulation_output_rssi_distance" << nDevices << "EDs_" << nGateways << "GWs_" << radiusMeters << "m_" << timeBuffer;
    //  std::string outputFolder = folderNameStream.str();
    outputFolder = folderNameStream.str();  // GOOD: sets the global variable
    

     // Check if the folder exists
     struct stat info;
     if (stat(outputFolder.c_str(), &info) != 0) {
         if (mkdir(outputFolder.c_str(), 0777) == 0) {
             std::cout << "Created folder: " << outputFolder << std::endl;
         } else {
             std::cerr << "Error: Failed to create folder: " << outputFolder << std::endl;
         }
     } else if (info.st_mode & S_IFDIR) {
         std::cout << "Folder already exists: " << outputFolder << std::endl;
     } else {
         std::cerr << "Error: A file with the same name as the folder already exists: " << outputFolder << std::endl;
     }

     // === Create localization log file header ===
    std::ofstream roundLocLog(outputFolder + "/localization-per-round.csv");
    roundLocLog << "Time(s),ED_ID,ActualX,ActualY,EstimatedX,EstimatedY,Error(m)\n";
    roundLocLog.close();


     // Set up logging
     LogComponentEnable("ComplexLorawanNetwork", LOG_LEVEL_ALL);
    //  LogComponentEnable("LoraChannel", LOG_LEVEL_INFO);
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
    // loss->SetReference(1, 40.0); // Reference distance and loss at reference distance
 
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
        //  position.z = 1.2; // Altitude of the end devices: by default 1.2 m
        position.z = 0.0; // Altitude of the end devices
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


        // ✅ Manually assign communication parameters (SF, BW, CR, CF and TP) to all EDs, although some of the parameters are assinged by defaults.
        for (uint32_t i = 0; i < endDevices.GetN(); ++i)
        {
            Ptr<LoraNetDevice> dev = endDevices.Get(i)->GetDevice(0)->GetObject<LoraNetDevice>();
            Ptr<EndDeviceLoraPhy> phy = dev->GetPhy()->GetObject<EndDeviceLoraPhy>();
        
            // Set Spreading Factor
            phy->SetSpreadingFactor(7); // Example: SF7

            // Set Carrier Frequency
            phy->SetFrequency(868100000); // 868.1 MHz


        }
        
        // === Store SF and TxPower to CSV for visualization ===
        std::ostringstream sfTpFileNameStream;
        sfTpFileNameStream << outputFolder << "/ed-sf-txpower-" 
                        << nDevices << "EDs_" 
                        << nGateways << "GWs_" 
                        << radiusMeters << "m.csv";
        std::ofstream sfTpFile(sfTpFileNameStream.str());
        sfTpFile << "ED_ID,SF,TxPower_dBm\n";

        for (uint32_t i = 0; i < endDevices.GetN(); ++i)
        {
            Ptr<LoraNetDevice> dev = endDevices.Get(i)->GetDevice(0)->GetObject<LoraNetDevice>();
            Ptr<EndDeviceLoraPhy> phy = dev->GetPhy()->GetObject<EndDeviceLoraPhy>();

            uint8_t sf = phy->GetSpreadingFactor();
            // double txPower = phy->GetTxPower();
            double txPower = 14.0; 

            sfTpFile << i << "," << static_cast<int>(sf) << "," << txPower << "\n";
        }
        sfTpFile.close();


 
     // Now end devices are connected to the channel
 
 
     /*********************
      *  Create Gateways  *
      *********************/
 
     // Create the gateway nodes (allocate them uniformly on the disc)
     NodeContainer gateways;
     gateways.Create(nGateways);
 
     Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();

     // allocator->Add(Vector(0.0, 0.0, 15.0)); // Set the position of the gateway --> Add(Vector(x, y, z))
     
    //  allocator->Add(Vector(900.0, 100.0, 15.0)); // Gateway 1 at (0, 0, 15)
    //  allocator->Add(Vector(0.0, 500.0, 15.0)); // Gateway 2 at (50, 50, 15)
    //  allocator->Add(Vector(500.0, 1000.0, 15.0)); // Gateway 3 at (-50, -50, 15)
     

    // allocator->Add(Vector(-500.0, 289.0, 10.0)); // Gateway 1 
    // allocator->Add(Vector(500.0, 289.0, 10.0)); // Gateway 2
    // allocator->Add(Vector(-500.0, -289.0, 10.0)); // Gateway 3 
    // allocator->Add(Vector(500.0, -289.0, 10.0)); // Gateway 4

    allocator->Add(Vector(-500.0, 350.0, 10.0)); // Gateway 1 
    allocator->Add(Vector(500.0, 350.0, 10.0)); // Gateway 2
    allocator->Add(Vector(-500.0, -350.0, 10.0)); // Gateway 3 
    allocator->Add(Vector(500.0, -350.0, 10.0)); // Gateway 4

     mobility.SetPositionAllocator(allocator);
     mobility.Install(gateways);
 
     // Create a netdevice for each gateway
     phyHelper.SetDeviceType(LoraPhyHelper::GW);
     macHelper.SetDeviceType(LorawanMacHelper::GW);
     helper.Install(phyHelper, macHelper, gateways);

    //  // ✅ Explicitly configure GW PHYs for downlink beaconing
    //     for (uint32_t i = 0; i < gateways.GetN(); ++i)
    //     {
    //         Ptr<LoraNetDevice> dev = gateways.Get(i)->GetDevice(0)->GetObject<LoraNetDevice>();
    //         Ptr<GatewayLoraPhy> phy = dev->GetPhy()->GetObject<GatewayLoraPhy>();

    //         phy->SetSpreadingFactor(7);    // SF7
    //         phy->SetBandwidth(125000);     // 125 kHz in Hz
    //         phy->SetCodingRate(1);         // CR = 4/5 (1 means 4/5 in NS-3 enum)
    //     }

 
 
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
            std::ostringstream buildingsFileNameStream;
            buildingsFileNameStream << outputFolder << "/buildings-info.txt";
            myfile.open(buildingsFileNameStream.str());

            if (!myfile.is_open())
            {
                std::cerr << "Failed to open buildings-info.txt for writing." << std::endl;
                return 1; // Indicate an error
            }

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
 
    //  LorawanMacHelper::SetSpreadingFactorsUp(endDevices, gateways, channel);

    //  // Log the spreading factor (SF) assigned to each end device
    // std::ostringstream sfLogFileNameStream;
    // sfLogFileNameStream << outputFolder << "/spreading-factors-" 
    //                     << nDevices << "EDs_" 
    //                     << nGateways << "GWs_" 
    //                     << radiusMeters << "m.csv";
    // std::ofstream sfLog(sfLogFileNameStream.str());
    // sfLog << "ED_ID,SF\n";
    //     for (uint32_t i = 0; i < endDevices.GetN(); ++i)
    //     {
    //         Ptr<LoraNetDevice> dev = endDevices.Get(i)->GetDevice(0)->GetObject<LoraNetDevice>();
    //         Ptr<EndDeviceLoraPhy> phy = dev->GetPhy()->GetObject<EndDeviceLoraPhy>();
    //         uint8_t sf = phy->GetSpreadingFactor();
    //         sfLog << i << "," << static_cast<int>(sf) << "\n";
    //     }

    //     sfLog.close();



    // std::cout << "Confirming manual SF assignment:\n";
    // for (uint32_t i = 0; i < 10; ++i)
    // {
    //     Ptr<LoraNetDevice> dev = endDevices.Get(i)->GetDevice(0)->GetObject<LoraNetDevice>();
    //     Ptr<EndDeviceLoraPhy> phy = dev->GetPhy()->GetObject<EndDeviceLoraPhy>();
    //     std::cout << "ED" << i << ": SF = " << static_cast<int>(phy->GetSpreadingFactor()) << "\n";
    // }


    for (uint32_t i = 0; i < endDevices.GetN(); ++i)
        {
            Ptr<LoraNetDevice> dev = endDevices.Get(i)->GetDevice(0)->GetObject<LoraNetDevice>();
            Ptr<EndDeviceLoraPhy> phy = dev->GetPhy()->GetObject<EndDeviceLoraPhy>();

            // Force SF7
            phy->SetSpreadingFactor(7);
        }

 
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
    //  AnimationInterface anim("simulation_output/lorawan-downlink-3GW-100EDs-with-noise.xml");

    // === NetAnim Setup ===
    std::ostringstream animFileNameStream;
    animFileNameStream << outputFolder << "/lorawan-downlink-" 
                    << nGateways << "GWs_" 
                    << nDevices << "EDs_" 
                    << radiusMeters << "m.xml";
    AnimationInterface anim(animFileNameStream.str());
    
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

     ///////////////////////////
     // Print results to file //
     ///////////////////////////

     std::cout << "\n==============================================" << std::endl;
     NS_LOG_INFO("Simulation completed. Writing results to file...");
     NS_LOG_INFO("Computing performance metrics...");
     std::cout << "==============================================" << std::endl;

    //  std::ofstream locFile("simulation_output/lora-localization-results-with-noise.csv");
    //  locFile << "ED,ActualX,ActualY,EstimatedX,EstimatedY,Error\n";

     std::ostringstream locFileNameStream;
     locFileNameStream << outputFolder << "/lora-localization-results-" 
                       << nDevices << "EDs_" 
                       << nGateways << "GWs_" 
                       << radiusMeters << "m.csv";
     std::ofstream locFile(locFileNameStream.str());
     locFile << "ED,ActualX,ActualY,EstimatedX,EstimatedY,Error\n";
     
        // Loop through each end device's beacon data
        // and perform trilateration to estimate the position

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



        // Collect all errors for statistical analysis
        std::vector<double> allErrors;

        double totalError = 0.0;
        int localizableDevices = 0;

        // Loop through each end device's beacon data
        for (const auto& [edIndex, beacons] : edToBeaconData)
        {
            // Skip devices with fewer than 3 beacons (not localizable)
            if (beacons.size() < 3) continue;

            // Prepare data for trilateration
            std::vector<MyVector2D> gwPositions;
            std::vector<double> distances;

            for (const auto& b : beacons)
            {
                gwPositions.push_back(b.gwPos);
                distances.push_back(b.distance);
            }

            // Perform trilateration to estimate the position
            MyVector2D estimated = TrilateratePosition(gwPositions, distances);

            // Retrieve the actual position of the end device
            Ptr<Node> ed = endDevices.Get(edIndex);
            Ptr<MobilityModel> mob = ed->GetObject<MobilityModel>();
            Vector actual = mob->GetPosition();

            // Calculate the localization error
            double error = std::sqrt(std::pow(estimated.x - actual.x, 2) + std::pow(estimated.y - actual.y, 2));

            // Accumulate error statistics
            totalError += error;
            localizableDevices++;
            allErrors.push_back(error); // Store all errors
        }

        // Calculate statistics
        if (!allErrors.empty())
        {
            double meanError = totalError / allErrors.size();

            // Calculate standard deviation
            double sumSquaredDiff = 0.0;
            for (double err : allErrors)
            {
                sumSquaredDiff += std::pow(err - meanError, 2);
            }
            double stdDev = std::sqrt(sumSquaredDiff / allErrors.size());

            // Find min and max
            double minError = *std::min_element(allErrors.begin(), allErrors.end());
            double maxError = *std::max_element(allErrors.begin(), allErrors.end());

            // Calculate median (requires sorting)
            std::sort(allErrors.begin(), allErrors.end());
            double median;
            if (allErrors.size() % 2 == 0)
            {
                median = (allErrors[allErrors.size() / 2 - 1] + allErrors[allErrors.size() / 2]) / 2.0;
            }
            else
            {
                median = allErrors[allErrors.size() / 2];
            }

            // Output results to console
            std::cout << "\n==== Localization Error Statistics ====" << std::endl;
            std::cout << "Total localizable devices: " << allErrors.size() << std::endl;
            std::cout << "Mean error: " << meanError << " meters" << std::endl;
            std::cout << "Median error: " << median << " meters" << std::endl;
            std::cout << "Standard deviation: " << stdDev << " meters" << std::endl;
            std::cout << "Min error: " << minError << " meters" << std::endl;
            std::cout << "Max error: " << maxError << " meters" << std::endl;


             // Write results to a file
            std::ostringstream errorFileNameStream;
            errorFileNameStream << outputFolder << "/Statistical-error-summary.txt";
            std::ofstream errorFile(errorFileNameStream.str());
            if (!errorFile.is_open())
            {
                std::cerr << "Failed to open Statistical-error-summary.txt for writing." << std::endl;
                return 1; // Indicate an error
            }


            errorFile << "Total localizable devices: " << allErrors.size() << std::endl;
            errorFile << "Mean error: " << meanError << " meters" << std::endl;
            errorFile << "Median error: " << median << " meters" << std::endl;
            errorFile << "Standard deviation: " << stdDev << " meters" << std::endl;
            errorFile << "Min error: " << minError << " meters" << std::endl;
            errorFile << "Max error: " << maxError << " meters" << std::endl;
            errorFile.close();
        }
        else
        {
            std::cout << "No localizable devices found (fewer than 3 beacons per device)." << std::endl;
        }


     Simulator::Destroy();



     std::cout << "\n==============================================" << std::endl;
     std::cout << "🔼 Uplink Packet Statistics (MAC-layer only)" << std::endl;
     std::cout << "This value represents the number of uplink packets" << std::endl;
     std::cout << "successfully transmitted by all end devices." << std::endl;
     std::cout << "==============================================" << std::endl;

 
     LoraPacketTracker& tracker = helper.GetPacketTracker();
 
     // Count and print the total number of MAC-layer packets globally
     std::cout << "Total MAC-layer packets transmitted globally: "
             << tracker.CountMacPacketsGlobally(Seconds(0), appStopTime + Hours(1)) << std::endl;
 
     
    //  // Compute average downlink PDR
    //  ComputeAverageDownlinkPDR("downlink-pdr.txt", nGateways);

     std::ostringstream pdrFile;
     pdrFile << outputFolder << "/downlink-pdr-" 
            << nDevices << "EDs_" 
            << nGateways << "GWs_" 
            << radiusMeters << "m.txt";
     ComputeAverageDownlinkPDR(pdrFile.str(), nGateways);


     // Write results to a file
     std::ostringstream errorFileNameStream;
     errorFileNameStream << outputFolder << "/localization-error-summary.txt";
     std::ofstream errorFile(errorFileNameStream.str());
     if (!errorFile.is_open())
        {
            std::cerr << "Failed to open localization-error-summary.txt for writing." << std::endl;
            return 1; // Indicate an error
        }


     std::cout << "\n==== End Devices Receiving Beacons from ≥ 3 Gateways ====" << std::endl;
     int coveredCount = 0;
     for (const auto& [edIndex, gwSet] : edToGwsMap)
        {
            if (gwSet.size() >= 3)
            {
                std::cout << "ED" << edIndex << " received beacons from " << gwSet.size() << " GWs" << std::endl;
                errorFile << "ED" << edIndex << " received beacons from " << gwSet.size() << " GWs" << std::endl;
                coveredCount++;
            }
        }
        std::cout << "Total EDs covered by ≥ 3 GWs: " << coveredCount << " out of " << edToGwsMap.size() << std::endl;
        errorFile << "Total EDs covered by ≥ 3 GWs: " << coveredCount << " out of " << edToGwsMap.size() << std::endl;
        errorFile.close();
    

     std::cout << "------------------------------------" << std::endl;
     std::cout << "Simulation completed successfully." << std::endl;
     std::cout << "------------------------------------" << std::endl;

     // Verify output files
     std::cout << "\nVerifying output files..." << std::endl;
     std::cout << "------------------------------------" << std::endl;
     VerifyOutputFiles(outputFolder, nDevices, nGateways, radiusMeters);

     return 0; // Indicate successful execution
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


// function to localize all devices at a given time 
void LocalizeAllDevicesAt(Time currentTime, NodeContainer endDevices, std::string filePath)
{
    std::ofstream locLog(filePath, std::ios::app);
    if (!locLog.is_open())
    {
        NS_LOG_ERROR("Failed to open localization log file.");
        return;
    }

    for (const auto& [edIndex, beacons] : edToBeaconData)
    {
        if (beacons.size() < 3)
            continue;

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

        locLog << currentTime.GetSeconds() << ","
               << edIndex << ","
               << actual.x << "," << actual.y << ","
               << estimated.x << "," << estimated.y << ","
               << error << "\n";
    }

    locLog.close();

    // 🔄 Clear the beacon data if you want localization to be per-round (sliding window of 1)
    edToBeaconData.clear();
    edToGwsMap.clear();
}
