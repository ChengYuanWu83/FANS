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

void rnl::Properties::initialize(bool rt , bool chsum ) 
{
  if (rt)
  {
    ns3::GlobalValue::Bind ("SimulatorImplementationType", ns3::StringValue ("ns3::RealtimeSimulatorImpl"));
    ns3::GlobalValue::Bind ("ChecksumEnabled", ns3::BooleanValue (chsum));
  }

  ns3::Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", ns3::UintegerValue(70));

  ns3::Config::SetDefault ("ns3::PcapFileWrapper::NanosecMode", ns3::BooleanValue (true));

  c.Create(num_nodes + 1);
  tid = ns3::TypeId::LookupByName ("ns3::UdpSocketFactory");
  std::cerr<<"Initialization of Properties Completed..."<<std::endl;
}

void rnl::Properties::setWifi(bool verbose, bool pcap_enable)
{
    // The below set of helpers will help us to put together the wifi NICs we want
    if (verbose)
    {
        wifi.EnableLogComponents ();  // Turn on all Wifi logging
    }

    wifi.SetStandard (ns3::WIFI_STANDARD_80211b);

    // This is one parameter that matters when using FixedRssLossModel
    // set it to zero; otherwise, gain will be added
    wifiPhy.Set ("TxGain", ns3::DoubleValue(0));
    wifiPhy.Set ("RxGain", ns3::DoubleValue (0));
    wifiPhy.Set ("RxSensitivity", ns3::DoubleValue (-77.5));
    wifiPhy.Set ("TxPowerStart", ns3::DoubleValue (20.0));
    wifiPhy.Set ("TxPowerEnd", ns3::DoubleValue (20.0));

    wifiPhy.Set ("ShortPlcpPreambleSupported", ns3::BooleanValue (true) );

    wifiPhy.SetPcapDataLinkType (ns3::YansWifiPhyHelper::DLT_IEEE802_11);

    wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    
    wifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel","Exponent",ns3::DoubleValue(3), 
        "ReferenceDistance", ns3::DoubleValue(1), "ReferenceLoss", ns3::DoubleValue(40.02));
    
    wifiPhy.SetChannel (wifiChannel.Create ());

    // Add a mac and disable rate control
    wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                    "DataMode",ns3::StringValue (phy_mode),
                                    "ControlMode",ns3::StringValue (phy_mode));

    if (c.GetN() > 0) {
        wifiMac.SetType("ns3::ApWifiMac",
                    "Ssid", ns3::StringValue("DroneNetwork"));
        
        // 為基站（節點0）安裝AP的MAC層配置
        // ns3::NetDeviceContainer apDevice;
        apDevice = wifi.Install(wifiPhy, wifiMac, c.Get(0));
        devices.Add(apDevice);
        
        // 為其餘節點（無人機）設置STA模式
        wifiMac.SetType("ns3::StaWifiMac",
                    "Ssid", ns3::StringValue("DroneNetwork"),
                    "ActiveProbing", ns3::BooleanValue(false));
        
        // 為無人機節點安裝STA的MAC層配置
        ns3::NodeContainer droneNodes;
        for (uint32_t i = 1; i < c.GetN(); ++i) {
            droneNodes.Add(c.Get(i));
        }
        
        // ns3::NetDeviceContainer staDevices;
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
        std::cerr << "Node 0 assigned IP: " << addr << std::endl;
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

// void rnl::Properties::SetStaticRoute(ns3::Ptr<ns3::Node> n, const char* destination, const char* nextHop, uint32_t interface)
// {
//   ns3::Ipv4StaticRoutingHelper staticRouting;
//   ns3::Ptr<ns3::Ipv4> ipv4 = n->GetObject<ns3::Ipv4> ();
//   ns3::Ptr<ns3::Ipv4StaticRouting> a = staticRouting.GetStaticRouting (ipv4);
//   a->AddHostRouteTo (ns3::Ipv4Address (destination), ns3::Ipv4Address (nextHop), interface);
// }

ns3::TypeId& rnl::Properties::tid_val ()
{
  return tid;
}

ns3::TypeId rnl::Properties::tid_val () const
{
  return tid;
}

/*---------------------------------------------------------------------------*/
/*-------------------------------DroneSoc---------------------------------*/
/*---------------------------------------------------------------------------*/
rnl::DroneSoc::DroneSoc()
{
  source = nullptr;
  recv_sink = nullptr;
}

void rnl::DroneSoc::closeSender ()
{
  this->source -> Close();
}

// void rnl::DroneSoc::setSender (ns3::Ptr<ns3::Node> node, ns3::TypeId tid, const std::string& ip)
// {
//     this->source = ns3::Socket::CreateSocket (node, tid);
//     ns3::InetSocketAddress remote1 = ns3::InetSocketAddress (ns3::Ipv4Address (ip.c_str()), 9);
//     std::cerr << "setSender IP to IP: " << (rnl::IP_BASE).c_str() << this->id + 1 << ", "<< ip.c_str() <<std::endl;
//     this->source->Connect (remote1);
// }

// void rnl::DroneSoc::setSenderTCP (ns3::Ptr<ns3::Node> node, const std::string& self_ip, const std::string& remote_ip, ns3::Time startTime)
// {
//   ns3::BulkSendHelper source ("ns3::TcpSocketFactory", ns3::InetSocketAddress (self_ip.c_str(), 8080));
//   source.SetAttribute ("MaxBytes", ns3::UintegerValue (536*20));
//   ns3::InetSocketAddress remote = ns3::InetSocketAddress (ns3::Ipv4Address ( remote_ip.c_str() ), 8080);
//   source.SetAttribute ("Remote", ns3::AddressValue (remote));
//   source.SetAttribute ("SendSize", ns3::UintegerValue (536));
//   ns3::ApplicationContainer sourceApps = source.Install (node);
//   sourceApps.Start (startTime);
//   sourceApps.Stop (startTime + ns3::Seconds (1));
// }

void rnl::DroneSoc::setBcSender (ns3::Ptr<ns3::Node> node, ns3::TypeId tid)
{
    this->source_bc = ns3::Socket::CreateSocket (node, tid);
    ns3::InetSocketAddress remote1 = ns3::InetSocketAddress (ns3::Ipv4Address ("255.255.255.255"), 9);
    this->source_bc->SetAllowBroadcast (true);
    this->source_bc->Connect (remote1);
    std::cerr << "Node " << this->id << " successfully connected socket for broadcasting" << std::endl;
}

void rnl::DroneSoc::setRecv (ns3::Ptr<ns3::Node> node, ns3::TypeId tid)
{
  this -> recv_sink = ns3::Socket::CreateSocket (node, tid);
  ns3::InetSocketAddress local1 = ns3::InetSocketAddress (ns3::Ipv4Address::GetAny (), 9);
  if (this->recv_sink->Bind(local1) == -1) {
    std::cerr << "Failed to bind socket for node " << this->id << std::endl;
  } else {
    std::cerr << "Successfully bound socket " << this->recv_sink <<" for node "<< this->id << std::endl;
  }
  this -> recv_sink->SetRecvCallback (ns3::MakeCallback (&rnl::DroneSoc::receivePacket, this));
}

void rnl::DroneSoc::setRecvTCP (ns3::Ptr<ns3::Node> node, const std::string& ip, int num_nodes, ns3::Time stopTime)
{
  ns3::Address sinkAddress (ns3::InetSocketAddress (ip.c_str(), 8080));
  ns3::PacketSinkHelper packetSinkHelper ("ns3::TcpSocketFactory", ns3::InetSocketAddress (ns3::Ipv4Address::GetAny (), 8080));
  ns3::ApplicationContainer sinkApps = packetSinkHelper.Install (node);
  ns3::Ptr<ns3::PacketSink> packetSink = sinkApps.Get (0)->GetObject<ns3::PacketSink> ();
  packetSink->TraceConnectWithoutContext ("Rx", ns3::MakeBoundCallback (&TraceSink, num_nodes-1));
  sinkApps.Start (ns3::Seconds (80));
  sinkApps.Stop (stopTime);
}

void rnl::DroneSoc::receivePacket(ns3::Ptr<ns3::Socket> soc)
{    
    ns3::Ptr<ns3::Packet> packet;
    ns3::Address senderAddress;

    while ((packet = soc->RecvFrom(senderAddress))) {

        if (packet->GetSize() == 0) { 
            NS_LOG_WARN("Received empty packet");
            break;
        }
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        packet->CopyData(buf, packet->GetSize());

        mavlink_message_t msg;
        mavlink_status_t status;

        for (uint32_t i = 0; i < packet->GetSize(); ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                // Handle the received MAVLink message here
                ns3::InetSocketAddress inetSenderAddr = ns3::InetSocketAddress::ConvertFrom(senderAddress);
                switch(msg.msgid) {
                    case MAVLINK_MSG_ID_ODOMETRY:
                    {
                        mavlink_odometry_t odom;
                        mavlink_msg_odometry_decode(&msg, &odom);

                        // handle received position
                        ns3::Vector3D receivedPosition(odom.x, odom.y, odom.z);
                        bool res = rnl::getTrajectory (&this->wpts, this->pos, receivedPosition, rnl::STEP);
                        this->goal = receivedPosition;
                        this->lookaheadindex = 0;
                        
                        std::cerr << "Received target position from " << 
                                    inetSenderAddr.GetIpv4() << " (node " << unsigned(msg.sysid) << ")" <<
                                    " x=" << receivedPosition.x << 
                                    " y=" << receivedPosition.y << 
                                    " z=" << receivedPosition.z << std::endl;
                        break;
                    }
                    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
                    {
                        mavlink_set_position_target_local_ned_t pt;
                        mavlink_msg_set_position_target_local_ned_decode(&msg, &pt);

                        if(pt.target_system != this->id){
                            std::cerr << "Received packet not belong to it. Discard" << std::endl; 
                            return;
                        }

                        // handle received position
                        ns3::Vector3D receivedPosition(pt.x, pt.y, pt.z);
                        bool res = rnl::getTrajectory (&this->wpts, this->pos, receivedPosition, rnl::STEP);
                        this->goal = receivedPosition;
                        this->lookaheadindex = 0;

                        this->state = SMOVE;
                        
                        std::cerr << "Node: " << this->id << " received target position from " << 
                                    inetSenderAddr.GetIpv4() << " (node " << unsigned(msg.sysid) << ")" <<
                                    " x=" << receivedPosition.x << 
                                    " y=" << receivedPosition.y << 
                                    " z=" << receivedPosition.z << std::endl;
                        break;
                    }
                    case MAVLINK_MSG_ID_MISSION_ACK:
                    {
                        mavlink_mission_ack_t ack;
                        mavlink_msg_mission_ack_decode(&msg, &ack);

                        if(ack.target_system != this->id){
                            std::cerr << "Received packet not belong to it. Discard" << std::endl; 
                            return;
                        }

                        //hanlde ack
                        std::stringstream ss;
                        ss << unsigned(msg.sysid) << ":";
                        if(ack.type == MAV_RESULT_ACCEPTED){
                            ss << " arrive taregt";
                        }
                        else{
                            ss << " failure";
                        }   
                        std_msgs::String response;
                        response.data = ss.str(); 
                        
                        this->arrive_response_pub.publish(response);

                        std::cerr << "Node: " << this->id <<" received response from " << 
                                    inetSenderAddr.GetIpv4() << " (node " << unsigned(msg.sysid) << ")" << std::endl;
                        break;
                    }
                    default:
                        break;
                }
            }
            NS_LOG_INFO("Received MAVLink message with ID: " << msg.msgid);
        }
    }

} 

// void rnl::DroneSoc::updateSendMsg ()
// {
//   if (msg_send.p_id == this->id)
//   {
//     msg_send.p_loc = this->pos;
//   }
// }

// void rnl::DroneSoc::sendBcPacket (ns3::Time pktInterval, int n)
// {
//   std::string msg;
//   msg_send.serializeBC(&msg, this->id, this->pos);
// 	ns3::Ptr<ns3::Packet> packet = ns3::Create<ns3::Packet> ((uint8_t*) msg.c_str(), msg.length());
	
//   this->source_bc->Send (packet);  
// }

// void rnl::DroneSoc::sendPacket (ns3::Time pktInterval, int n)
// {
//   updateSendMsg ();
//   std::string msg;
//   msg_send.serialize(&msg);
// 	ns3::Ptr<ns3::Packet> packet = ns3::Create<ns3::Packet> ((uint8_t*) msg.c_str(), msg.length());
	
//   this->source->Send (packet);
//   if (toggle_bc ==1)
//   {
//     ns3::Simulator::Schedule ((n - 1/2)*pktInterval, &rnl::DroneSoc::sendBcPacket, this,
//     pktInterval, n);
//   }
// 	ns3::Simulator::Schedule (n*pktInterval, &rnl::DroneSoc::sendPacket, this,
// 	pktInterval, n);

//   std::cerr << this->id << " sendPacket with state and control: "<< this->msg_send.state << ", "<< this->msg_send.control << std::endl;
// }
void rnl::DroneSoc::sendArrivedPacket(uint32_t targetId){
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_mission_ack_pack(this->id, 200, &msg, targetId, 200, MAV_RESULT_ACCEPTED, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    ns3::Ptr<ns3::Packet> packet = ns3::Create<ns3::Packet> (buf, len);
    this->source_bc->Send (packet);
    std::cerr <<"Arrived target. Send ACK to GCS" << std::endl;
}

void rnl::DroneSoc::sendMAVLinkPacket(uint32_t msgId, uint32_t targetId){
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Create a sample MAVLink message (heartbeat in this case)
    switch (msgId)
    {
    case MAVLINK_MSG_ID_ODOMETRY:
        float pose_covariance[21]; 
        float velocity_covariance[21];
        // mavlink_msg_odometry_pack(targetId, 200, &msg, ns3::Simulator::Now().GetMicroSeconds(), MAV_FRAME_GLOBAL, MAV_FRAME_GLOBAL, x, y, z, q, 0, 0, 0, 0, 0, 0, 0, pose_covariance, velocity_covariance, 0, 0 ,0);
        break;
    case MAVLINK_MSG_ID_MISSION_ACK:  // Your custom message ID:17000
        // Pack your custom message here. Adjust according to your message structure.
        mavlink_msg_mission_ack_pack(this->id, 200, &msg, MAV_RESULT_ACCEPTED ,targetId, 200, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
        break;
    default:
        NS_LOG_ERROR("Unknown message ID: " << msgId);
        return;
    }
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    ns3::Ptr<ns3::Packet> packet = ns3::Create<ns3::Packet> (buf, len);
    this->source->Send(packet);
}
void rnl::DroneSoc::sendRosPacket (const geometry_msgs::PoseStamped::ConstPtr& _pos) { //cyw
    std::cerr <<"Calling sendRosPacket" << std::endl;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint8_t targetId = std::stoi(_pos->header.frame_id);
    float x = _pos->pose.position.x;
    float y = _pos->pose.position.y;
    float z = _pos->pose.position.z;
    uint16_t type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE |
    	POSITION_TARGET_TYPEMASK_VY_IGNORE |
    	POSITION_TARGET_TYPEMASK_VZ_IGNORE |
    	POSITION_TARGET_TYPEMASK_AX_IGNORE |
    	POSITION_TARGET_TYPEMASK_AY_IGNORE |
    	POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    	POSITION_TARGET_TYPEMASK_FORCE_SET |
    	POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    mavlink_msg_set_position_target_local_ned_pack(this->id, 200, &msg, ns3::Simulator::Now().GetMicroSeconds(), targetId, 200, MAV_FRAME_GLOBAL, type_mask, x, y, z, 0, 0, 0, 0, 0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    ns3::Ptr<ns3::Packet> packet = ns3::Create<ns3::Packet> (buf, len);
    this->source_bc->Send (packet);
    std::cerr <<"Target pose x=" << x << ", y=" << y << ", z=" << z << " send to node" << unsigned(targetId) << std::endl;
}

// void rnl::DroneSoc::initializeRosParams (ros::NodeHandle& nh) {  
//     if(this->id == 0){
//         arrive_response_pub = nh.advertise<std_msgs::String>("/gcs/arrived", 10);
//         // target_pos_sub1      = nh.subscribe("/gcs/target_pose1",
//         //                             10, &rnl::DroneSoc::sendRosPacket, this);
//         // target_pos_sub2      = nh.subscribe("/gcs/target_pose2",
//         //                                 10, &rnl::DroneSoc::sendRosPacket, this);
//         target_pos_sub      = nh.subscribe("/gcs/target_pose",
//                                             10, &rnl::DroneSoc::sendRosPacket, this);
//     }
//     else{
//         target_pos_sub     = nh.subscribe("/gcs/target_pose_uav" + this->id,
//                                     10, &rnl::DroneSoc::sendRosPacket, &nsocs[0]);
//         drone_lk_ahead_pub = nh.advertise<geometry_msgs::Pose>( "/uav" + std::to_string(this->id) + "/sp_pos", 1);
//         drone_pos_sub      = nh.subscribe("/uav" + std::to_string(this->id) + "/global_pose",
//                                     1, &rnl::DroneSoc::posSubCb, this);
//         std::cerr << this->id << " Initialized ros params" << std::endl;
//         std::cerr <<"/uav" + std::to_string(this->id) + "/global_pose"  << " Subscriber" << std::endl;
//     }    
// }

void rnl::DroneSoc::posSubCb (const geometry_msgs::PoseStamped& _pos)
{
    this->pos.x = _pos.pose.position.x;
    this->pos.y = _pos.pose.position.y;
    this->pos.z = _pos.pose.position.z;

}

void rnl::DroneSoc::imageSubCb (const sensor_msgs::ImageConstPtr& _msg)
{
    this->image = *_msg;
}


void rnl::DroneSoc::publishLookAhead ()
{
    geometry_msgs::Pose _lka;
    _lka.position.x = this->wpts[this->lookaheadindex].x;
    _lka.position.y = this->wpts[this->lookaheadindex].y;
    _lka.position.z = this->wpts[this->lookaheadindex].z;
    // std::cerr<<"Node "<< this->id << " fly to " << _lka << std::endl;
    drone_lk_ahead_pub.publish (_lka);
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
  leader_id = 0;
  ldirec_flag = 1;
  lchild_id = 1;
  tail_id = 6;
}

void rnl::Planner::initializeMobility ()
{
    ns3::Ptr<ns3::ListPositionAllocator> positionAlloc = ns3::CreateObject<ns3::ListPositionAllocator> ();

    for (int i = 0; i < nsocs.size(); ++i){
        positionAlloc -> Add (nsocs[i].pos);
        std::cerr << "Node" << i << " mobility set to " << nsocs[i].pos << std::endl;
    }
    mobility.SetPositionAllocator (positionAlloc);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (wifi_prop.c);

    
}

void rnl::Planner::initializeRosParams()
{  
    for (int i = 0; i < nsocs.size(); ++i){
        if(i == 0){
            nsocs[i].arrive_response_pub = nh.advertise<std_msgs::String>("/gcs/arrived", 10);

        }
        else{
            nsocs[i].target_pos_sub     = nh.subscribe("/gcs/target_pose_uav" + std::to_string(i),
                                        10, &rnl::DroneSoc::sendRosPacket, &nsocs[0]);
            nsocs[i].drone_lk_ahead_pub = nh.advertise<geometry_msgs::Pose>( "/uav" + std::to_string(i) + "/sp_pos", 1);
            nsocs[i].drone_pos_sub      = nh.subscribe("/uav" + std::to_string(i) + "/global_pose",
                                        1, &rnl::DroneSoc::posSubCb, &nsocs[i]);
            nsocs[i].drone_image_sub      = nh.subscribe("/uav" + std::to_string(i) + "/image",
                                        1, &rnl::DroneSoc::imageSubCb, &nsocs[i]);
            std::cerr << i << " Initialized ros params" << std::endl;
            std::cerr <<"/uav" + std::to_string(i) + "/global_pose"  << " Subscriber" << std::endl;
        }   
    }
     
}

rnl::Nbt rnl::setinitialNbt (int id , int n)
{
  rnl::Nbt nbt;
  // if (id - 1 >= 0 )
  //   nbt.one_hop.push_back (std::pair <int, ns3::Vector3D> (id - 1 , ns3::Vector3D (0,0,0)));
  
  // if (id - 2 >= 0 )
  //   nbt.two_hop.push_back (std::pair <int, ns3::Vector3D> (id - 2 , ns3::Vector3D (0,0,0)));
  
  // if (id + 1 < n)
  //   nbt.one_hop.push_back (std::pair <int, ns3::Vector3D> (id + 1 , ns3::Vector3D (0,0,0)));
  
  // if (id + 2 < n)
  //   nbt.two_hop.push_back (std::pair <int, ns3::Vector3D> (id + 2 , ns3::Vector3D (0,0,0)));

  return nbt;
}

// rnl::USMsg rnl::setinitialSMsg (rnl::Nbt nbt, int id, int n)
// {
//   rnl::USMsg  msg;
//   msg.source_id = id;

//   if (id + 1 < n)
//     msg.dst_id = id + 1;
//   else
//     msg.dst_id = rnl::BASEID;

//   nbt.serialize (&msg.nbs);
  
//   msg.control = CHOLDRC;
//   msg.state   = SONLINE | SGDRONEREQ;
  
//   msg.p_id  = id;
//   msg.p_loc = ns3::Vector3D (-id,0.0,rnl::Planner::disas_centre.z);

//   return msg;
// }

void rnl::Planner::initializeSockets ()
{
  nsocs.clear();
  for (int i = 0 ; i < num_nodes + 1; ++i)
  {
    rnl::DroneSoc  _dsoc;
    _dsoc.id       = i; 
    // rnl::Nbt       _nbt     = rnl::setinitialNbt  (i, num_nodes);
    // rnl::USMsg     _smsg    = rnl::setinitialSMsg (_nbt, i, num_nodes); 
    // rnl::URMsg     _rmsg;
    // if (i+1 < num_nodes)
    // {
    //   _dsoc.setSender (wifi_prop.c.Get(i), wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(i+2));
    // }
    // else
    // {
    //   _dsoc.setSender (wifi_prop.c.Get(i), wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(rnl::BASEID));
    // }
    _dsoc.setBcSender (wifi_prop.c.Get(i), wifi_prop.tid_val());
    _dsoc.toggle_bc = 1;
    if(i == 0){
        _dsoc.pos      = ns3::Vector3D(0.0, 0.0 , 0.0);
    }
    else {
        _dsoc.pos      = ns3::Vector3D(-i , 0.0 , rnl::Planner::disas_centre.z);
    }
    rnl::posHold(&_dsoc.wpts,_dsoc.pos);
    _dsoc.lookaheadindex = 0;
    // _dsoc.msg_send = _smsg;
    // _dsoc.msg_rec  = _rmsg;
    // _dsoc.nbt      = _nbt;
    _dsoc.state    = SMOVE;
    nsocs.push_back(_dsoc);
  }
}

bool rnl::Planner::siteReached (ns3::Vector3D pos, ns3::Vector3D goal, int ID)
{
    bool res;
    res = (ns3::CalculateDistance(pos, goal) < 1) ? 1 : 0;
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
    if (nsocs[i].lookaheadindex + 1 < nsocs[i].wpts.size() && withinThreshold(&nsocs[i]))
    {
      nsocs[i].lookaheadindex ++;
    }
  }
}

void rnl::Planner::updatePosSocs ()
{
  for (int i = 0; i < nsocs.size(); ++i)
  {
    rnl::setPosition(wifi_prop.c.Get(i), nsocs[i].pos);
  }
}

void rnl::Planner::updateSocsfromRec ()
{
  for (int i = 1; i < nsocs.size(); ++i)
  {
    // updateWpts(i);
    std::cerr << "update waypoints" << std::endl;
  }
}



void rnl::Planner::updateSocs ()
{
  for (int i = 1; i < num_nodes + 1; ++i)
  {
    if (rnl::Planner::siteReached (nsocs[i].pos, nsocs[i].goal, i) && !(nsocs[i].state & SSITEREACHED)){
        ns3::Simulator::ScheduleNow (&rnl::DroneSoc::sendArrivedPacket, &nsocs[i], 0); // 0 is the id of gcs
        nsocs[i].state = SSITEREACHED;
        rnl::posHold (&nsocs[i].wpts, nsocs[i].pos);
        nsocs[i].lookaheadindex = 0;
    }
  }
}
void rnl::Planner::advancePos (ns3::Time interval)
{
    ros::spinOnce();
    updatePosSocs ();   //update ns3 mobility
    incLookAhead ();    //increase index to the next waypoints
    updateSocs ();

    for (int i = 1; i < nsocs.size(); ++i)
    {
        if (nsocs[i].wpts.size())
        {
            nsocs[i].publishLookAhead();
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
    for (int i =0; i< nsocs.size(); ++i)
    {
        // ns3::Simulator::Schedule (ns3::Seconds (2.0) + i*pkt_interval, &rnl::DroneSoc::sendPacket, &nsocs[i], pkt_interval, num_nodes);
        nsocs[i].setRecv (wifi_prop.c.Get(i), wifi_prop.tid_val());
        // nsocs[i].initializeRosParams (nh);

    }
    initializeRosParams();
    initializeMobility();


    ns3::Simulator::ScheduleNow (&rnl::Planner::takeOff, this, ns3::Simulator::Now ().GetSeconds());
    ns3::Simulator::Schedule (ns3::Seconds (2.0) + 5 * (num_nodes+1) * pkt_interval, &rnl::Planner::advancePos, this, pos_interval);
    ns3::Simulator::Stop(stopTime);
    ns3::AnimationInterface anim ("planner_ns3_anim.xml");
    anim.SetMaxPktsPerTraceFile(9999999);
    ns3::Simulator::Run();
    ns3::Simulator::Destroy();
}
