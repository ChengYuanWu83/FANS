#include "planner_ns3.h"

#include "ns3/packet-sink.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/bulk-send-helper.h"

int start_lawn = 0;
int start_left = 0;
int Pkt[] = {1, 1, 1, 1, 1, 1, 1, 1, 1};


void TraceSink (std::size_t index, ns3::Ptr<const ns3::Packet> p, const ns3::Address& a)
{
  std::cerr << "At " << ns3::Simulator::Now ().GetSeconds ()
            << " sec, node" << index << " received " << p->GetSize () << "bytes"
            << " from "<< ns3::InetSocketAddress::ConvertFrom (a).GetIpv4() << std::endl;

  std::ofstream out;
  std::string fileName = "pkt_rec_time.txt";
  out.open(fileName.c_str(), std::ios::app);
  for(int i=0; i<=5; i++){
    if( ns3::Ipv4Address( (rnl::IP_BASE + std::to_string(i+1)).c_str() ) == ns3::InetSocketAddress::ConvertFrom(a).GetIpv4() )
    {
      out << "At " << ns3::Simulator::Now().GetSeconds() << "\t" << "received " << "Pkt No.:" << Pkt[i]
          << " from "<< "node" << i << std::endl;

      Pkt[i]++;

      if(Pkt[i] == 21)
      {
        Pkt[i] = 1;
        out << std::endl;

        if(i==5)
        {
          out << "-------------------------------------------" << std::endl << std::endl;
        }
      }
      break;
    }
  }
  out.close();
}

/*---------------------------------------------------------------------------*/
/*-------------------------------Properties---------------------------------*/
/*---------------------------------------------------------------------------*/
rnl::Properties::Properties (
  std::string _phyMode = "DsssRate1Mbps", 
  double _rss = -80,  // dBm // Deprecated
  int _num_nodes = 3
):
  phy_mode{_phyMode}, 
  rss{_rss}, 
  num_nodes{_num_nodes}
{
}

void rnl::Properties::initialize(bool rt, bool chsum ) 
{    
    if (rt)
    {
        ns3::GlobalValue::Bind ("SimulatorImplementationType", ns3::StringValue ("ns3::RealtimeSimulatorImpl"));
        ns3::GlobalValue::Bind ("ChecksumEnabled", ns3::BooleanValue (chsum));
    }

    // ns3::Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", ns3::UintegerValue(70));

    ns3::Config::SetDefault ("ns3::PcapFileWrapper::NanosecMode", ns3::BooleanValue (true));
    // ns3::Config::SetDefault("ns3::UdpSocket::SndBufSize", ns3::UintegerValue(65536));
    // ns3::Config::SetDefault("ns3::UdpSocket::RcvBufSize", ns3::UintegerValue(4294967295));


    c.Create(num_nodes + 1);
    tid = ns3::TypeId::LookupByName ("ns3::UdpSocketFactory");
    std::cerr<<"Initialization of Properties Completed..."<<std::endl;
}

void rnl::Properties::setWifi(bool verbose, bool pcap_enable, bool band_5GHz_enable)
{
    // The below set of helpers will help us to put together the wifi NICs we want
    if (verbose)
    {
        wifi.EnableLogComponents ();  // Turn on all Wifi logging
    }

    if(band_5GHz_enable){
        wifi.SetStandard (ns3::WIFI_STANDARD_80211n_5GHZ);
    }
    else{
        wifi.SetStandard (ns3::WIFI_STANDARD_80211n_2_4GHZ);
    }
    
    wifi.SetRemoteStationManager("ns3::MinstrelHtWifiManager");
    

    // This is one parameter that matters when using FixedRssLossModel
    // set it to zero; otherwise, gain will be added

    // wifiPhy.Set ("TxGain", ns3::DoubleValue(0));
    // wifiPhy.Set ("RxGain", ns3::DoubleValue (0));
    // wifiPhy.Set ("RxSensitivity", ns3::DoubleValue (-77.5));
    // wifiPhy.Set ("TxPowerStart", ns3::DoubleValue (20.0));
    // wifiPhy.Set ("TxPowerEnd", ns3::DoubleValue (20.0));

    // wifiPhy.Set ("ShortPlcpPreambleSupported", ns3::BooleanValue (true) );
    
    // wifiPhy.Set("Frequency", UintegerValue(frequency)); // Set to frequency MHz frequency band


    wifiPhy.SetPcapDataLinkType (ns3::YansWifiPhyHelper::DLT_IEEE802_11);

    wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    
    wifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel","Exponent",ns3::DoubleValue(3), 
        "ReferenceDistance", ns3::DoubleValue(1), "ReferenceLoss", ns3::DoubleValue(40.02));
    
    wifiPhy.SetChannel (wifiChannel.Create ());

    // Add a mac and disable rate control
    // wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
    //                                 "DataMode",ns3::StringValue (phy_mode),
    //                                 "ControlMode",ns3::StringValue (phy_mode));

    if (c.GetN() > 0) {
        // Install AP's MAC layer configuration for the 
        wifiMac.SetType("ns3::ApWifiMac",
                    "Ssid", ns3::StringValue("DroneNetwork"));
        
        apDevice = wifi.Install(wifiPhy, wifiMac, c.Get(0));
        devices.Add(apDevice);
        
        // Install STA's MAC layer configuration for the drone node
        wifiMac.SetType("ns3::StaWifiMac",
                    "Ssid", ns3::StringValue("DroneNetwork"),
                    "ActiveProbing", ns3::BooleanValue(false));
        
        ns3::NodeContainer droneNodes;
        for (uint32_t i = 1; i < c.GetN(); ++i) {
            droneNodes.Add(c.Get(i));
        }

        staDevices = wifi.Install(wifiPhy, wifiMac, droneNodes);
        devices.Add(staDevices);
    }

    if (pcap_enable)
    {
        wifiPhy.EnableAsciiAll (ascii.CreateFileStream ("planner_ns3_trace.tr"));
        wifiPhy.EnablePcap ("planner_ns3", devices);
    }
    std::cerr<<"Wifi Properties Set"<<std::endl;
}

void rnl::Properties::setInternet()
{
    internet.Install(c);
    std::cerr<<"Assigning IP address"<<std::endl;
    std::string bid = rnl::IP_BASE + "0"; 

    // assign GCS ip 10.1.1.50
    if (c.GetN() > 0) {
        ipv4.SetBase(bid.c_str(), "255.255.255.0", "0.0.0.50");
        ns3::Ipv4InterfaceContainer apInterface = ipv4.Assign(apDevice);
        i.Add(apInterface);

        ns3::Ptr<ns3::Ipv4> ipv4j = c.Get(0)->GetObject<ns3::Ipv4>();
        ns3::Ipv4Address addr = ipv4j->GetAddress(1, 0).GetLocal();
        std::cerr << "GCS (AP) assigned IP: " << addr << std::endl;
    }
    
    // assign Drone 1~n ip 10.1.1.1~n
    if (c.GetN() > 1) {
        ipv4.SetBase(bid.c_str(), "255.255.255.0", "0.0.0.1");
        
        ns3::Ipv4InterfaceContainer staInterfaces = ipv4.Assign(staDevices);
        i.Add(staInterfaces);

        for (uint32_t j = 1; j < c.GetN(); ++j) {
            ns3::Ptr<ns3::Ipv4> ipv4j = c.Get(j)->GetObject<ns3::Ipv4>();
            ns3::Ipv4Address addr = ipv4j->GetAddress(1, 0).GetLocal();
            std::cerr << "Drone (Node " << j << ") assigned IP: " << addr << std::endl;
        }
    }

    std::string fName = "pkt_rec_time.txt";
    std::ofstream fout (fName.c_str());
    fout.close();

    std::cerr << "IPs assigned with centralized architecture" << std::endl;
}


ns3::TypeId& rnl::Properties::tid_val ()
{
  return tid;
}

ns3::TypeId rnl::Properties::tid_val () const
{
  return tid;
}

/*---------------------------------------------------------------------------*/
/*---------------------------------Soc---------------------------------------*/
/*---------------------------------------------------------------------------*/
rnl::Soc::Soc()
{
  source = nullptr;
  recv_sink = nullptr;
}
rnl::Soc::~Soc() {}

void rnl::Soc::closeSender ()
{
  this->source -> Close();
}


void rnl::Soc::setBcSender (ns3::Ptr<ns3::Node> node, ns3::TypeId tid)
{
    this->source_bc = ns3::Socket::CreateSocket (node, tid);
    std::string bcAddr = IP_BASE + "255";
    ns3::InetSocketAddress remote = ns3::InetSocketAddress (ns3::Ipv4Address(bcAddr.c_str()), 9);
    this->source_bc->SetAllowBroadcast (true);
    this->source_bc->Connect (remote);
    std::cerr << "Node " << this->id << " successfully connected socket for broadcasting" << std::endl;
}

static void StaticRecvCallback(ns3::Ptr<ns3::Socket> socket)
{
    std::cerr << "StaticRecvCallback called!" << std::endl;
}
void rnl::Soc::setRecv (ns3::Ptr<ns3::Node> node, ns3::TypeId tid)
{
    this -> recv_sink = ns3::Socket::CreateSocket (node, tid);
    ns3::InetSocketAddress local1 = ns3::InetSocketAddress (ns3::Ipv4Address::GetAny (), 9);
    if (this->recv_sink->Bind(local1) == -1) {
        std::cerr << "Failed to bind socket for node " << this->id << std::endl;
    } else {
        std::cerr << "Successfully bound socket " << this->recv_sink <<" for node "<< this->id << std::endl;
    }
    std::cerr << "this = " << this << ", type = " << typeid(*this).name() << std::endl;

    this -> recv_sink->SetRecvCallback (ns3::MakeCallback (&rnl::Soc::receivePacket, this));
}
void rnl::Soc::setRecv_test (ns3::Ptr<ns3::Node> node, ns3::TypeId tid)
{
    this -> recv_sink = ns3::Socket::CreateSocket (node, tid);
    ns3::InetSocketAddress local1 = ns3::InetSocketAddress (ns3::Ipv4Address::GetAny (), 9);
    if (this->recv_sink->Bind(local1) == -1) {
        std::cerr << "Failed to bind socket for node " << this->id << std::endl;
    } else {
        std::cerr << "Successfully bound socket " << this->recv_sink <<" for node "<< this->id << std::endl;
    }
    std::cerr << "this = " << this << ", type = " << typeid(*this).name() << std::endl;

    this -> recv_sink->SetRecvCallback (ns3::MakeCallback (&StaticRecvCallback));
}


void rnl::Soc::receivePacket(ns3::Ptr<ns3::Socket> soc)
{ 
    std::cerr <<"StaticRecvCallback called!"<<std::endl;
    
}

void rnl::Soc::sendOdomPacket(geometry_msgs::Pose _pos){
    int64_t sendTimestamp = ns3::Simulator::Now().GetMicroSeconds();
    float x = _pos.position.x;
    float y = _pos.position.y;
    float z = _pos.position.z;
    float qx = _pos.orientation.x;
    float qy = _pos.orientation.y;
    float qz = _pos.orientation.z;
    float qw = _pos.orientation.w;
    float q[4] = {qw, qx,qy,qz};
    float null[21];
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_odometry_pack(
        this->id, 200, &msg,
        sendTimestamp,
        MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_NED,
        x, y, z, q,
        0, 0, 0,  // velocities field
        0, 0, 0,  // anlgular velocities field
        null, null,
        0, 0, 0        
    );

    int16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    ns3::Ptr<ns3::Packet> packet = ns3::Create<ns3::Packet> (buf, len);
    this->source_bc->Send (packet);
    std::cerr <<"Camera pose x=" << x << ", y=" << y << ", z=" << z << 
                " send to node 0" << std::endl;
}

void rnl::Soc::sendArrivedPacket(uint32_t targetId, uint32_t cmdId){
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // mavlink_msg_mission_ack_pack(this->id, 200, &msg, targetId, 200, MAV_RESULT_ACCEPTED, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
    // uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // ns3::Ptr<ns3::Packet> packet = ns3::Create<ns3::Packet> (buf, len);
    // this->source_bc->Send (packet);
    // std::cerr <<"Arrived target. Send ACK to GCS" << std::endl;
    switch (cmdId)
    {
        case MAV_CMD_NAV_WAYPOINT:
        {
            mavlink_msg_command_ack_pack(
                this->id,            // Sender system ID
                200,                 // Sender component ID
                &msg,                // Message to pack into
                MAV_CMD_NAV_WAYPOINT,// Command being acknowledged
                MAV_RESULT_ACCEPTED, // Result code
                100,                 // Progress (100 = complete)
                0,                   // result_param2 (unused)
                targetId,            // Target system ID
                200                  // Target component ID
            );
            std::cerr << "Arrived target. Send COMMAND_ACK to GCS" << std::endl;
            break;
        }
        
        case MAV_CMD_IMAGE_START_CAPTURE:
        {
            std::pair<uint8_t, uint8_t> sender(msg.sysid, msg.compid);
            
            mavlink_msg_command_ack_pack(
                this->id,            // Sender system ID
                200,                 // Sender component ID
                &msg,                // Message to pack into
                MAV_CMD_IMAGE_START_CAPTURE,// Command being acknowledged
                MAV_RESULT_ACCEPTED, // Result code
                this->image_buffers_[sender].batch_idx,                 // Progress (100 = complete)
                0,                     // result_param2 (unused)
                targetId,            // Target system ID
                200                  // Target component ID
            );
            break;
        }

        default:
            break;
    }
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    ns3::Ptr<ns3::Packet> packet = ns3::Create<ns3::Packet>(buf, len);
    this->source_bc->Send(packet);
    
}

void rnl::Soc::sendGoalPacket (const geometry_msgs::PoseStamped::ConstPtr& _pos) { // send position  and orientation
    
    int64_t sendTimestamp = ns3::Simulator::Now().GetMicroSeconds();
    uint8_t targetId = std::stoi(_pos->header.frame_id);
    float x = _pos->pose.position.x;
    float y = _pos->pose.position.y;
    float z = _pos->pose.position.z;
    float qx = _pos->pose.orientation.x;
    float qy = _pos->pose.orientation.y;
    float qz = _pos->pose.orientation.z;
    float qw = _pos->pose.orientation.w;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint16_t type_mask = 
        POSITION_TARGET_TYPEMASK_VX_IGNORE |
        POSITION_TARGET_TYPEMASK_VY_IGNORE |
        POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        POSITION_TARGET_TYPEMASK_AX_IGNORE |
        POSITION_TARGET_TYPEMASK_AY_IGNORE |
        POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        POSITION_TARGET_TYPEMASK_FORCE_SET |
        POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; 

    mavlink_msg_set_position_target_local_ned_pack(
        this->id, 200, &msg,
        sendTimestamp,
        targetId, 200, MAV_FRAME_LOCAL_NED,
        type_mask,
        x, y, z,
        0, 0, 0,  // velocities field
        0, 0, 0,  // accelerations
        0, 0    // yaw, yaw_rate
    );
    int16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    ns3::Ptr<ns3::Packet> packet = ns3::Create<ns3::Packet> (buf, len);
    this->source_bc->Send (packet);
    std::cerr <<"Target pose x=" << x << ", y=" << y << ", z=" << z << 
                " send to node" << unsigned(targetId) << std::endl;
}

/*---------------------------------------------------------------------------*/
/*-------------------------------Planner-------------------------------------*/
/*---------------------------------------------------------------------------*/
rnl::Planner::Planner(ros::NodeHandle& _nh, ros::NodeHandle& _nh_private, rnl::Properties& p, int n, float  _pki,
              float _pos_int, float _stopTime):
wifi_prop{p}, num_nodes{n}, pkt_interval{ns3::Seconds(_pki)},
pos_interval{ns3::Seconds(_pos_int)}, stopTime{ns3::Seconds(_stopTime)},
nh{_nh}, nh_private{_nh_private}
{

}

void rnl::Planner::initializeBuilding()
{
    // Building
    ns3::Ptr<ns3::Building> building1 = ns3::CreateObject<ns3::Building>();
    building1->SetBoundaries(ns3::Box(-3.0, 3.0, -3.0, 3.0, 0.0, 3.0));  //x_min, x_max, y_min, y_max, z_min, z_max
    building1->SetBuildingType(ns3::Building::Office);
    building1->SetExtWallsType(ns3::Building::ConcreteWithWindows);
    building1->SetNFloors(12);
    building1->SetNRoomsX(3);
    building1->SetNRoomsY(4);

    ns3::BuildingContainer buildings;
    buildings.Add(building1);
    ns3::BuildingsHelper::Install(wifi_prop.c);

}

void rnl::Planner::initializeMobility ()
{
    ns3::Ptr<ns3::ListPositionAllocator> positionAlloc = ns3::CreateObject<ns3::ListPositionAllocator> ();
    positionAlloc -> Add (gsocs.pos);
    std::cerr << "Node" << gsocs.id << " mobility set to " << gsocs.pos << std::endl;

    for (int i = 0; i < nsocs.size(); ++i){
        positionAlloc -> Add (nsocs[i]->pos);
        std::cerr << "Node" << nsocs[i]->id << " mobility set to " << nsocs[i]->pos << std::endl;
    }
    mobility.SetPositionAllocator (positionAlloc);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (wifi_prop.c);
    
}

void rnl::Planner::initializeRosParams()
{  
    gsocs.arrive_response_pub= nh.advertise<std_msgs::String>("/gcs/arrived", 10);
    gsocs.drone_image_pub    = nh.advertise<sensor_msgs::Image>("/gcs/image", 10);
    gsocs.drone_camera_pub   = nh.advertise<geometry_msgs::Pose>("/gcs/cam_pose", 10);

    for (int i = 0; i < nsocs.size(); ++i){
        nsocs[i]->drone_camera_sub   = nh.subscribe("/uav" + std::to_string(nsocs[i]->id) + "/cam_pose",
                                    1, &rnl::DroneSoc::camPosSubCb, &*nsocs[i]);
        nsocs[i]->target_pos_sub     = nh.subscribe("/gcs/target_pose_uav" + std::to_string(nsocs[i]->id),
                                    10, &rnl::GcsSoc::sendGoalPacket, &gsocs);
        nsocs[i]->drone_lk_ahead_pub = nh.advertise<geometry_msgs::Pose>( "/uav" + std::to_string(nsocs[i]->id) + "/sp_pos", 1);
        nsocs[i]->drone_pos_sub      = nh.subscribe("/uav" + std::to_string(nsocs[i]->id) + "/global_pose",
                                    1, &rnl::DroneSoc::posSubCb, &*nsocs[i]);
        nsocs[i]->drone_image_sub    = nh.subscribe("/uav" + std::to_string(nsocs[i]->id) + "/image",
                                    1, &rnl::DroneSoc::imageSubCb, &*nsocs[i]);
                                    
        std::cerr << nsocs[i]->id << " Initialized ros params" << std::endl;
        std::cerr <<"/uav" + std::to_string(nsocs[i]->id) + "/global_pose"  << " Subscriber" << std::endl;
    
    }
}


void rnl::Planner::initializeSockets (double dist_gcs2building, double _jpeg_quality, bool _imagePublish)
{
    nsocs.clear();
    
    // gsocs = std::make_shared<rnl::GcsSoc>();
    gsocs.id = 0;
    gsocs.system = GCS;
    gsocs.setBcSender(wifi_prop.c.Get(0), wifi_prop.tid_val());
    gsocs.setRecv(wifi_prop.c.Get(0), wifi_prop.tid_val());
    gsocs.imagePublish = _imagePublish;
    gsocs.pos = ns3::Vector3D(dist_gcs2building, 0.0 , 0.0);
    

    for (int i = 0 ; i < num_nodes; ++i)
    {
        auto _dsoc = std::make_shared<rnl::DroneSoc>();
        _dsoc->id       = i + 1; 
        _dsoc->system   = DRONE;
        _dsoc->setBcSender (wifi_prop.c.Get(_dsoc->id), wifi_prop.tid_val());
        _dsoc->setRecv (wifi_prop.c.Get(_dsoc->id), wifi_prop.tid_val());
        _dsoc->toggle_bc = 1;       
        _dsoc->jpeg_quality = _jpeg_quality;

        std::string filename    = std::string(std::getenv("HOME")) + "/fans_ws/src/pci/config/uav" + std::to_string(_dsoc->id) + ".yaml";
        YAML::Node config       = YAML::LoadFile(filename);
        _dsoc->pos               = ns3::Vector3D(config["initial_pos"][0].as<double>(),
                                                config["initial_pos"][1].as<double>(),
                                                config["initial_pos"][2].as<double>());

        // rnl::posHold(&_dsoc->wpts,_dsoc->pos);
        _dsoc->lookaheadindex = 0;
        _dsoc->state    = SMOVE;
        nsocs.push_back(_dsoc);
    }
}

bool rnl::Planner::siteReached (ns3::Vector3D pos, ns3::Vector3D goal, int ID)
{
    bool res;
    res = (ns3::CalculateDistance(pos, goal) < 0.5) ? 1 : 0;
	return res;
}


bool rnl::Planner::withinThreshold (const rnl::DroneSoc* _soc)
{
  ns3::Vector3D _lka = _soc->wpts[_soc->lookaheadindex];
  ns3::Vector3D _cpos = _soc -> pos;
  return ns3::CalculateDistance (_lka, _cpos) < 0.5;
}

void rnl::Planner::incLookAhead ()
{
  for (int i = 0; i< nsocs.size(); ++i)
  {
    if (nsocs[i]->lookaheadindex + 1 < nsocs[i]->wpts.size() && withinThreshold(&*nsocs[i]))
    {
      nsocs[i]->lookaheadindex ++;
    }
  }
}

void rnl::Planner::updatePosSocs ()
{
  for (int i = 0; i < nsocs.size(); ++i)
  {
    rnl::setPosition(wifi_prop.c.Get(i), nsocs[i]->pos);
  }
}

void rnl::Planner::updateSocsfromRec ()
{
  for (int i = 0; i < nsocs.size(); ++i)
  {
    // updateWpts(i);
    std::cerr << "update waypoints" << std::endl;
  }
}



void rnl::Planner::updateSocs ()
{
  for (int i = 0; i < num_nodes; ++i)
  {
        if (rnl::Planner::siteReached (nsocs[i]->pos, nsocs[i]->goal, i) && nsocs[i]->state != SSITEREACHED){
            ns3::Simulator::Schedule (ns3::MilliSeconds(2500), &rnl::DroneSoc::sendImagePacket, &*nsocs[i]); // Wait 2.5 seconds for the drone to stabilize
            // ns3::Simulator::Schedule (ns3::MilliSeconds(4000), &rnl::DroneSoc::sendArrivedPacket, &*nsocs[i], 0, MAV_CMD_NAV_WAYPOINT); // 0 is the id of gcs
            nsocs[i]->state = SSITEREACHED;
            rnl::posHold (&nsocs[i]->wpts, nsocs[i]->pos);
            nsocs[i]->lookaheadindex = 0;
        }
  }
}
void rnl::Planner::advancePos (ns3::Time interval)
{

    ros::spinOnce();
    updatePosSocs ();   //update ns3 mobility
    incLookAhead ();    //increase index to the next waypoints
    updateSocs ();

    for (int i = 0; i < nsocs.size(); ++i)
    {
        if (nsocs[i]->wpts.size())
        {
            nsocs[i]->publishLookAhead();
        }
    }
    ns3::Simulator::Schedule(interval, &rnl::Planner::advancePos, this, interval);
}

void rnl::Planner::takeOff (double _t)
{
    if ((ns3::Simulator::Now ().GetSeconds() - _t) < 1)
    {
        ros::spinOnce();
        ns3::Simulator::Schedule (ns3::Seconds(0.1), &rnl::Planner::takeOff, this, _t);
    }
  
    else{
        // setLeaderExplorePath ();
    }
}

void rnl::Planner::startSimul()
{
    for (int i = 0; i < nsocs.size(); ++i)
    {
        // nsocs[i]->setRecv (wifi_prop.c.Get(i), wifi_prop.tid_val());
    }
    initializeRosParams();
    initializeMobility();
    initializeBuilding();

    ns3::Simulator::ScheduleNow (&rnl::Planner::takeOff, this, ns3::Simulator::Now ().GetSeconds());
    ns3::Simulator::ScheduleNow (&rnl::Planner::advancePos, this, pos_interval);
    ns3::Simulator::Stop(stopTime);
    ns3::AnimationInterface anim ("planner_ns3_anim.xml");
    anim.SetMaxPktsPerTraceFile(9999999);
    ns3::Simulator::Run();
    ns3::Simulator::Destroy();
}
