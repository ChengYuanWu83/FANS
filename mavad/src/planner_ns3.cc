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
            << " sec node" << index << " received " << p->GetSize () << "bytes"
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
  double _rss = -80,  // -dBm
  int _num_nodes = 3
):
  phy_mode{_phyMode}, 
  rss{_rss}, 
  num_nodes{_num_nodes}
{
}

void rnl::Properties::initialize(bool rt , bool chsum ) 
{
    // inter_packet_interval = ns3::Seconds(interval);
    // broadcast_interval = ns3::Seconds(interval);
    
    if (rt){
        ns3::GlobalValue::Bind ("SimulatorImplementationType",
            ns3::StringValue ("ns3::RealtimeSimulatorImpl"));
        ns3::GlobalValue::Bind ("ChecksumEnabled", ns3::BooleanValue (chsum));
    }

    // Fix non-unicast data rate to be the same as that of unicast
    // ns3::Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
    //                   ns3::StringValue (phy_mode));

    ns3::Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", ns3::UintegerValue(70));

    ns3::Config::SetDefault ("ns3::PcapFileWrapper::NanosecMode", ns3::BooleanValue (true));

    c.Create(num_nodes);
    tid = ns3::TypeId::LookupByName ("ns3::UdpSocketFactory");
    std::cerr<<"TEST1:Initialization of Properties Complete..."<<std::endl;
}

void rnl::Properties::setWifi(bool verbose, bool pcap_enable)
{

    // std::cerr<<"TEST2:setWifi start"<<std::endl;
        // The below set of helpers will help us to put together the wifi NICs we want
    if (verbose)
        {
        wifi.EnableLogComponents ();  // Turn on all Wifi logging
        }
    //wifi.SetStandard (ns3::WIFI_PHY_STANDARD_80211b);
    wifi.SetStandard (ns3::WIFI_STANDARD_80211b);
    // std::cerr<<"TEST3:setStandard"<<std::endl;

    // This is one parameter that matters when using FixedRssLossModel
    // set it to zero; otherwise, gain will be added
    wifiPhy.Set ("TxGain", ns3::DoubleValue(0));
    wifiPhy.Set ("RxGain", ns3::DoubleValue (0));
    wifiPhy.Set ("RxSensitivity", ns3::DoubleValue (-77.5));
    wifiPhy.Set ("TxPowerStart", ns3::DoubleValue (20.0));
    wifiPhy.Set ("TxPowerEnd", ns3::DoubleValue (20.0));

    wifiPhy.Set ("ShortPlcpPreambleSupported", ns3::BooleanValue (true) );

    // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
    // wifiPhy.SetPcapDataLinkType (ns3::WifiPhyHelper::DLT_IEEE802_11_RADIO);
    wifiPhy.SetPcapDataLinkType (ns3::YansWifiPhyHelper::DLT_IEEE802_11);
    // std::cerr<<"TEST4:setPhy"<<std::endl;

    wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    // The below FixedRssLossModel will cause the rss to be fixed regardless
    // of the distance between the two stations, and the transmit power
    // wifiChannel.AddPropagationLoss ("ns3::FixedRssLossModel","Rss",ns3::DoubleValue (rss));
    // wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange",
		// 	ns3::DoubleValue(rnl::MAXTHRESHRC));
    wifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel","Exponent",ns3::DoubleValue(3), 
      "ReferenceDistance", ns3::DoubleValue(1), "ReferenceLoss", ns3::DoubleValue(40.02));
    
    wifiPhy.SetChannel (wifiChannel.Create ());
    // std::cerr<<"TEST5:setChannel"<<std::endl;

    // Add a mac and disable rate control
    wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                    "DataMode",ns3::StringValue (phy_mode),
                                    "ControlMode",ns3::StringValue (phy_mode));
    // Set it to adhoc mode
    wifiMac.SetType ("ns3::AdhocWifiMac");
    // std::cerr<<"TEST6:setMac"<<std::endl;
    devices = wifi.Install (wifiPhy, wifiMac, this->c);
    // std::cerr<<"TEST7:setDevice"<<std::endl;
  
    if (pcap_enable)
    {
      wifiPhy.EnableAsciiAll (ascii.CreateFileStream ("planner_ns3_trace.tr"));
      wifiPhy.EnablePcap ("planner_ns3", devices);
    }

    std::cerr<<"TEST2:Wifi Properties Set"<<std::endl;
}

void rnl::Properties::setInternet()
{
  internet.SetRoutingHelper (staticRouting); 
  internet.Install (c);
  std::cerr<<"Assigning IP"<<std::endl;
  std::string bid = rnl::IP_BASE + "0"; 
  ipv4.SetBase (bid.c_str(), "255.255.255.0");
  i = ipv4.Assign (devices);

  // nodes:
    // n2  n5
    // n0  n3  n6  n7
    // n1  n4

  // [n2 to n7], [n0 to n7], [n3 to n7]
  SetStaticRoute(c.Get(2),  (rnl::IP_BASE + "8").c_str(), (rnl::IP_BASE + "1").c_str(), 1);
  SetStaticRoute(c.Get(0),  (rnl::IP_BASE + "8").c_str(), (rnl::IP_BASE + "4").c_str(), 1);
  SetStaticRoute(c.Get(3),  (rnl::IP_BASE + "8").c_str(), (rnl::IP_BASE + "7").c_str(), 1);
  SetStaticRoute(c.Get(6),  (rnl::IP_BASE + "8").c_str(), (rnl::IP_BASE + "8").c_str(), 1);

  // [n7 to n2]
  SetStaticRoute(c.Get(7),  (rnl::IP_BASE + "3").c_str(), (rnl::IP_BASE + "7").c_str(), 1);
  SetStaticRoute(c.Get(6),  (rnl::IP_BASE + "3").c_str(), (rnl::IP_BASE + "4").c_str(), 1);
  SetStaticRoute(c.Get(3),  (rnl::IP_BASE + "3").c_str(), (rnl::IP_BASE + "1").c_str(), 1);
  SetStaticRoute(c.Get(0),  (rnl::IP_BASE + "3").c_str(), (rnl::IP_BASE + "3").c_str(), 1);

  // [n7 to n0]
  SetStaticRoute(c.Get(7),  (rnl::IP_BASE + "1").c_str(), (rnl::IP_BASE + "7").c_str(), 1);
  SetStaticRoute(c.Get(6),  (rnl::IP_BASE + "1").c_str(), (rnl::IP_BASE + "4").c_str(), 1);
  SetStaticRoute(c.Get(3),  (rnl::IP_BASE + "1").c_str(), (rnl::IP_BASE + "1").c_str(), 1);

  // [n7 to n3]
  SetStaticRoute(c.Get(7),  (rnl::IP_BASE + "4").c_str(), (rnl::IP_BASE + "7").c_str(), 1);
  SetStaticRoute(c.Get(6),  (rnl::IP_BASE + "4").c_str(), (rnl::IP_BASE + "4").c_str(), 1);

  // [n1 to n7]
  SetStaticRoute(c.Get(1), (rnl::IP_BASE + "8").c_str(), (rnl::IP_BASE + "1").c_str(), 1);

  // [n7 to n1]
  SetStaticRoute(c.Get(7),  (rnl::IP_BASE + "2").c_str(), (rnl::IP_BASE + "7").c_str(), 1);
  SetStaticRoute(c.Get(6),  (rnl::IP_BASE + "2").c_str(), (rnl::IP_BASE + "4").c_str(), 1);
  SetStaticRoute(c.Get(3),  (rnl::IP_BASE + "2").c_str(), (rnl::IP_BASE + "1").c_str(), 1);
  SetStaticRoute(c.Get(0),  (rnl::IP_BASE + "2").c_str(), (rnl::IP_BASE + "2").c_str(), 1);

  // [n5 to n7]
  SetStaticRoute(c.Get(5), (rnl::IP_BASE + "8").c_str(), (rnl::IP_BASE + "4").c_str(), 1);

  // [n7 to n5]
  SetStaticRoute(c.Get(7),  (rnl::IP_BASE + "6").c_str(), (rnl::IP_BASE + "7").c_str(), 1);
  SetStaticRoute(c.Get(6),  (rnl::IP_BASE + "6").c_str(), (rnl::IP_BASE + "4").c_str(), 1);
  SetStaticRoute(c.Get(3),  (rnl::IP_BASE + "6").c_str(), (rnl::IP_BASE + "6").c_str(), 1);

  // [n4 to n7]
  SetStaticRoute(c.Get(4), (rnl::IP_BASE + "8").c_str(), (rnl::IP_BASE + "4").c_str(), 1);

  // [n7 to n4]
  SetStaticRoute(c.Get(7),  (rnl::IP_BASE + "5").c_str(), (rnl::IP_BASE + "7").c_str(), 1);
  SetStaticRoute(c.Get(6),  (rnl::IP_BASE + "5").c_str(), (rnl::IP_BASE + "4").c_str(), 1);
  SetStaticRoute(c.Get(3),  (rnl::IP_BASE + "5").c_str(), (rnl::IP_BASE + "5").c_str(), 1);

  std::string fName = "pkt_rec_time.txt";
  std::ofstream fout (fName.c_str());
  fout.close();

  std::cerr<<"TEST4: IPs Assigned"<<std::endl;
}

void rnl::Properties::SetStaticRoute(ns3::Ptr<ns3::Node> n, const char* destination, const char* nextHop, uint32_t interface)
{
  ns3::Ipv4StaticRoutingHelper staticRouting;
  ns3::Ptr<ns3::Ipv4> ipv4 = n->GetObject<ns3::Ipv4> ();
  ns3::Ptr<ns3::Ipv4StaticRouting> a = staticRouting.GetStaticRouting (ipv4);
  a->AddHostRouteTo (ns3::Ipv4Address (destination), ns3::Ipv4Address (nextHop), interface);
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

void rnl::DroneSoc::setSender (ns3::Ptr<ns3::Node> node, ns3::TypeId tid, const std::string& ip)
{
    this->source = ns3::Socket::CreateSocket (node, tid);
    ns3::InetSocketAddress remote1 = ns3::InetSocketAddress (ns3::Ipv4Address (ip.c_str()), 9);
    std::cerr << "IP to IP: " << this->id + 1 << ", "<< ip.c_str() <<std::endl;
    // this->source->SetAllowBroadcast (false);
    this->source->Connect (remote1);
}

void rnl::DroneSoc::setSenderTCP (ns3::Ptr<ns3::Node> node, const std::string& self_ip, const std::string& remote_ip, ns3::Time startTime)
{
  ns3::BulkSendHelper source ("ns3::TcpSocketFactory", ns3::InetSocketAddress (self_ip.c_str(), 8080));
  source.SetAttribute ("MaxBytes", ns3::UintegerValue (536*20));
  ns3::InetSocketAddress remote = ns3::InetSocketAddress (ns3::Ipv4Address ( remote_ip.c_str() ), 8080);
  source.SetAttribute ("Remote", ns3::AddressValue (remote));
  source.SetAttribute ("SendSize", ns3::UintegerValue (536));
  ns3::ApplicationContainer sourceApps = source.Install (node);
  sourceApps.Start (startTime);
  sourceApps.Stop (startTime + ns3::Seconds (1));
}

void rnl::DroneSoc::setBcSender (ns3::Ptr<ns3::Node> node, ns3::TypeId tid)
{
    this->source_bc = ns3::Socket::CreateSocket (node, tid);
    ns3::InetSocketAddress remote1 = ns3::InetSocketAddress (ns3::Ipv4Address ("255.255.255.255"), 9);
    this->source_bc->SetAllowBroadcast (true);
    this->source_bc->Connect (remote1);
}

void rnl::DroneSoc::setRecv (ns3::Ptr<ns3::Node> node, ns3::TypeId tid)
{
    this -> recv_sink = ns3::Socket::CreateSocket (node, tid);
    ns3::InetSocketAddress local1 = ns3::InetSocketAddress (ns3::Ipv4Address::GetAny (), 9);
    this -> recv_sink->Bind (local1);
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
  std::string receivedData;
  
  while (ns3::Ptr<ns3::Packet> msg = soc->Recv ())
    {
      uint8_t *buffer = new uint8_t[msg->GetSize ()];
      msg->CopyData (buffer, msg->GetSize ());
      receivedData = std::string ((char *) buffer);
      // std::cerr<< this->id << " received Packet at " << ns3::Simulator::Now ().GetSeconds()<<std::endl;
      // std::cerr << receivedData <<std::endl;
    }
  msg_rec.parse(receivedData);
  nbt.parseSingleNb (this->msg_rec.bc_nbs);
} 

void rnl::DroneSoc::updateSendMsg ()
{
  if (msg_send.p_id == this->id)
  {
    msg_send.p_loc = this->pos;
  }
}

void rnl::DroneSoc::sendBcPacket (ns3::Time pktInterval, int n)
{
  std::string msg;
  msg_send.serializeBC(&msg, this->id, this->pos);
  // std::cerr << "Msg in BC: " << msg<<std::endl;
	ns3::Ptr<ns3::Packet> packet = ns3::Create<ns3::Packet> ((uint8_t*) msg.c_str(), msg.length());
	
  // std::cerr<<"Broadcasting packet from node " << this->id << " at " << ns3::Simulator::Now().GetSeconds()<<std::endl;
  this->source_bc->Send (packet);  
}

void rnl::DroneSoc::sendPacket (ns3::Time pktInterval, int n)
{
  updateSendMsg ();
  // std::cerr << msg_send.state << ", " << msg_send.control <<std::endl;
  std::string msg;
  msg_send.serialize(&msg);
	ns3::Ptr<ns3::Packet> packet = ns3::Create<ns3::Packet> ((uint8_t*) msg.c_str(), msg.length());
	
  // std::cerr<<"Sending packet from node " << this->id << " at " << ns3::Simulator::Now().GetSeconds()<<std::endl;
  // std::cerr<<"Sending packet from node " << this->id << " State: " << msg_send.state<<std::endl;
  this->source->Send (packet);
  if (toggle_bc ==1)
  {
    ns3::Simulator::Schedule ((n - 1/2)*pktInterval, &rnl::DroneSoc::sendBcPacket, this,
    pktInterval, n);
  }
	ns3::Simulator::Schedule (n*pktInterval, &rnl::DroneSoc::sendPacket, this,
	pktInterval, n);
  // this->msg_send.state &= (~SGDRONEREQ);
  std::cerr << this->id << " sp "<< this->msg_send.state << ", "<< this->msg_send.control << std::endl;
}

// void rnl::DroneSoc::setReqDrone()
// {
//   std::cerr << "One Hops of " << this->id << std::endl;
//   for (auto x:nbt.one_hop)
//   {
//     std::cerr << x.first << ", " << x.second << std::endl;
//   }
//   this->msg_send.state |=SGDRONEREQ;
// }

void rnl::DroneSoc::initializeRosParams (ros::NodeHandle& nh)
{
  drone_lk_ahead_pub = nh.advertise<geometry_msgs::Pose>( "/uav" + std::to_string(this->id) + "/sp_pos", 1);
  drone_pos_sub      = nh.subscribe("/uav" + std::to_string(this->id) + "/global_pose",
                              1, &rnl::DroneSoc::posSubCb, this);
  std::cerr << this->id << " Initialized ros params" << std::endl;
  std::cerr <<"/uav" + std::to_string(this->id) + "/global_pose"  << " Subscriber" << std::endl;
}

void rnl::DroneSoc::posSubCb (const geometry_msgs::PoseStamped& _pos)
{
  this->pos.x = _pos.pose.position.x;
  this->pos.y = _pos.pose.position.y;
  this->pos.z = _pos.pose.position.z;

}

void rnl::DroneSoc::publishLookAhead ()
{
  geometry_msgs::Pose _lka;
  _lka.position.x = this->wpts[this->lookaheadindex].x;
  _lka.position.y = this->wpts[this->lookaheadindex].y;
  _lka.position.z = this->wpts[this->lookaheadindex].z;
  
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

  // Note that with FixedRssLossModel, the positions below are not
  // used for received signal strength.
  ns3::Ptr<ns3::ListPositionAllocator> positionAlloc = ns3::CreateObject<ns3::ListPositionAllocator> ();
  for (int i = 0; i< num_nodes; ++i){
    positionAlloc -> Add (nsocs[i].pos);
  }
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifi_prop.c);

  std::cerr<<"TEST3:Planner Mobility Set"<<std::endl;
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

rnl::USMsg rnl::setinitialSMsg (rnl::Nbt nbt, int id, int n)
{
  rnl::USMsg  msg;
  msg.source_id = id;

  if (id + 1 < n)
    msg.dst_id = id + 1;
  else
    msg.dst_id = rnl::BASEID;

  nbt.serialize (&msg.nbs);
  
  msg.control = CHOLDRC;
  msg.state   = SONLINE | SGDRONEREQ;
  
  msg.p_id  = id;
  // msg.p_loc = ns3::Vector3D (5 * id,0.0,rnl::Planner::fire_centre.z);
  // msg.p_loc = ns3::Vector3D (5 * id,0.0,rnl::Planner::disas_centre.z);
  msg.p_loc = ns3::Vector3D (-id,0.0,rnl::Planner::disas_centre.z);

  return msg;
}

void rnl::Planner::initializeSockets ()
{
  nsocs.clear();
  for (int i = 0 ; i < num_nodes; ++i)
  {
      rnl::DroneSoc  _dsoc;
      _dsoc.id       = i; 
      rnl::Nbt       _nbt     = rnl::setinitialNbt  (i, num_nodes);
      rnl::USMsg     _smsg    = rnl::setinitialSMsg (_nbt, i, num_nodes); 
      rnl::URMsg     _rmsg;
      if (i+1 < num_nodes)
      {
        _dsoc.setSender (wifi_prop.c.Get(i), wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(i+2));
      }
      else
      {
        _dsoc.setSender (wifi_prop.c.Get(i), wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(rnl::BASEID));
      }
      _dsoc.setBcSender (wifi_prop.c.Get(i), wifi_prop.tid_val());
      _dsoc.toggle_bc = 0;
      // _dsoc.pos      = ns3::Vector3D(5*i , 0.0 , rnl::Planner::fire_centre.z);
      _dsoc.pos      = ns3::Vector3D(-i , 0.0 , rnl::Planner::disas_centre.z);
      rnl::posHold(&_dsoc.wpts,_dsoc.pos);
      _dsoc.lookaheadindex = 0;
      _dsoc.msg_send = _smsg;
      _dsoc.msg_rec  = _rmsg;
      _dsoc.nbt      = _nbt;
      nsocs.push_back(_dsoc);
  }
}

// bool rnl::Planner::fireDetected (ns3::Vector3D pos)
// {
//   bool res = (sqrt(pow (pos.x - fire_centre.x, 2) + pow (pos.y - fire_centre.y, 2) + pow (pos.z - fire_centre.z, 2)) <= 1.5 * fire_rad);
//   return res;

// }

bool rnl::Planner::siteReached (ns3::Vector3D pos, int ID)
{
  // bool res = (sqrt(pow (pos.x - disas_centre.x, 2) + pow (pos.y - disas_centre.y, 2) + pow (pos.z - disas_centre.z, 2)) <= 1.5 * disas_rad);
  // return res;

  // TODO: make it based on all wpts traversed
	bool res;

  // ns3::Vector3D pos0(disas_centre.x + RC/sqrt(2), disas_centre.y + RC/sqrt(2), disas_centre.z);
  // ns3::Vector3D pos1(disas_centre.x,              disas_centre.y + RC*sqrt(2), disas_centre.z);
  // ns3::Vector3D pos2(disas_centre.x + RC*sqrt(2), disas_centre.y,              disas_centre.z);
  // ns3::Vector3D pos3(disas_centre.x,              disas_centre.y,              disas_centre.z);
  // ns3::Vector3D pos4(disas_centre.x - RC/sqrt(2), disas_centre.y + RC/sqrt(2), disas_centre.z);
  // ns3::Vector3D pos5(disas_centre.x + RC/sqrt(2), disas_centre.y - RC/sqrt(2), disas_centre.z);
  // ns3::Vector3D pos6(disas_centre.x - RC/sqrt(2), disas_centre.y - RC/sqrt(2), disas_centre.z);
  // ns3::Vector3D pos7(disas_centre.x - RC*sqrt(2), disas_centre.y,              disas_centre.z);
  // ns3::Vector3D pos8(disas_centre.x,              disas_centre.y - RC*sqrt(2), disas_centre.z);

  ns3::Vector3D pos0(disas_centre.x + rnl::RC, disas_centre.y,              disas_centre.z);
  ns3::Vector3D pos1(disas_centre.x + rnl::RC, disas_centre.y + rnl::RC,    disas_centre.z);
  ns3::Vector3D pos2(disas_centre.x + rnl::RC, disas_centre.y - rnl::RC,    disas_centre.z);
  ns3::Vector3D pos3(disas_centre.x,           disas_centre.y,              disas_centre.z);
  ns3::Vector3D pos4(disas_centre.x,           disas_centre.y + rnl::RC,    disas_centre.z);
  ns3::Vector3D pos5(disas_centre.x,           disas_centre.y - rnl::RC,    disas_centre.z);
  // ns3::Vector3D pos6(disas_centre.x - rnl::RC, disas_centre.y,              disas_centre.z);
  // ns3::Vector3D pos7(disas_centre.x - rnl::RC, disas_centre.y + rnl::RC,    disas_centre.z);
  // ns3::Vector3D pos8(disas_centre.x - rnl::RC, disas_centre.y - rnl::RC,    disas_centre.z);

	switch(ID)
	{
		case 0:
			res = (ns3::CalculateDistance(pos, pos0) < 0.4) ? 1 : 0;
			break;
		case 1:
			res = (ns3::CalculateDistance(pos, pos1) < 0.8) ? 1 : 0;
			break;
		case 2:
			res = (ns3::CalculateDistance(pos, pos2) < 0.6) ? 1 : 0;
			break;
		case 3:
			res = (ns3::CalculateDistance(pos, pos3) < 0.5) ? 1 : 0;
			break;
		case 4:
			res = (ns3::CalculateDistance(pos, pos4) < 1) ? 1 : 0;
			break;
		case 5:
			res = (ns3::CalculateDistance(pos, pos5) < 0.6) ? 1 : 0;
			break;
		// case 6:
		// 	res = (ns3::CalculateDistance(pos, pos6) < 1) ? 1 : 0;
		// 	break;
		// case 7:
		// 	res = (ns3::CalculateDistance(pos, pos7) < 1) ? 1 : 0;
		// 	break;
		// case 8:
		// 	res = (ns3::CalculateDistance(pos, pos8) < 1) ? 1 : 0;
		// 	break;
    default:
      res = 0;
      break;
	}

	return res;
}

void rnl::Planner::setLeaderExplorePath ()
{
  // bool res = rnl::getTrajectory (&nsocs[0].wpts, rnl::getPosition(wifi_prop.c.Get(0)), fire_centre, rnl::STEP);
  // ns3::Vector3D pos0(disas_centre.x + RC, disas_centre.y, disas_centre.z);
  // ns3::Vector3D pos0(disas_centre.x + RC/sqrt(2), disas_centre.y + RC/sqrt(2), disas_centre.z);
  ns3::Vector3D pos0(disas_centre.x + rnl::RC, disas_centre.y, disas_centre.z);
  bool res = rnl::getTrajectory (&nsocs[0].wpts, nsocs[0].pos, pos0, rnl::STEP);
  nsocs[0].lookaheadindex = 0;
  std::cerr << nsocs[0].pos << " is init pos at "<<ns3::Simulator::Now ().GetSeconds() << std::endl;

  /*To do: Checking if result was generated and variable leader dec
  *        Explorative planners
  */
}

// void rnl::Planner::incLeaderChildId ()
// {
//   if (lchild_id < num_nodes)
//   {
//     lchild_id++;
//     ldirec_flag*=-1;
//     rnl::DroneSoc* unode = &nsocs[leader_id];
//     ns3::Simulator::ScheduleNow (&rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
//                                     wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(lchild_id+1));
//     unode->msg_send.control = CHOLDRC;
//   }
// }

// void rnl::Planner::updateStateofLeader () // need to substitute with updateStateofCentre, based on no. of neighbour
// {
//   rnl::DroneSoc* unode = &nsocs[leader_id];
//   if (fireDetected (unode->pos))
//     {
//       if (!(unode->msg_send.state & SFIREDET))
//       {
//         std::cerr << "Leader Detected Fire First time" <<std::endl;
//         rnl::posHold (&unode->wpts, unode->pos);
//         unode->lookaheadindex = 0;
//         if (ldirec_flag == 1)
//           {
//             unode->msg_send.control = CRTOP;
//           }
//           else
//           {
//             unode->msg_send.control = CLTOP;
//           }
//         unode->msg_send.state = SFIREDET | SGFIREDET;
//         // unode->toggle_bc = 1;
//         ns3::Simulator::Schedule ((num_nodes) * pkt_interval, &rnl::Planner::incLeaderChildId, this);
//         std::cerr << unode->msg_send.state << ", "<<unode->msg_send.control << std::endl;
//       }

//       else
//       {
//         if (unode->msg_rec.state & SGDRONEREQ)
//         {
//           std::cerr << "Received Drone Req\n";
//           if (ldirec_flag == 1)
//           {
//             unode->msg_send.control = CRTOP;
//           }
//           else
//           {
//             unode->msg_send.control = CLTOP;
//           }
//           unode->msg_rec.state &= (~SGDRONEREQ);
//           ns3::Simulator::Schedule ((num_nodes) * pkt_interval, &rnl::Planner::incLeaderChildId, this);
//         }
//       }
      
//     }
// }

void rnl::Planner::updateStateofCentre ()
{
  for(int i=0; i < tail_id; i = i+3)
  {
    rnl::DroneSoc* unode = &nsocs[i];
    if (siteReached (unode->pos, unode->id))
    {
      if (!(unode->msg_send.state & SSITEREACHED))
      {
        unode->msg_send.neigh_cnt = 1;
        std::cerr << "Centre Site Reached First time" <<std::endl;
        if(i==0)
        {
          unode->msg_rec.state = SGDRONEREQ;
        }

        start_left = 0;
      }
      
      if(unode->msg_send.neigh_cnt < 4 && siteReached (nsocs[i+unode->msg_send.neigh_cnt].pos, i+unode->msg_send.neigh_cnt)){
        
        // std::cerr << nsocs[0].pos <<std::endl;
        // std::cerr << nsocs[1].pos <<std::endl;
        if(i+unode->msg_send.neigh_cnt < tail_id){
          unode->msg_send.neigh_cnt++;
        }
      }
      if(unode->msg_send.neigh_cnt < 4)
      {
        unode->msg_send.state = (SCENTRE | SSITEREACHED) | SGSITEREACHED;
        if(i+unode->msg_send.neigh_cnt == tail_id){
          unode->msg_send.state |= !SGDRONEREQ;
        }
        else{
          unode->msg_send.state |= SGDRONEREQ;
        }
        // unode->msg_send.state = SCENTRE | SSITEREACHED | SGSITEREACHED;
        // std::cerr << "I'm here" <<std::endl;
        rnl::posHold (&unode->wpts, unode->pos);
        unode->lookaheadindex = 0;
        unode->toggle_bc = 1;
        if(unode->msg_send.neigh_cnt==1)
        {
          if(start_left > 20)
          {
            unode->msg_send.control = CLTOP;
          }
          start_left++;
          // std::cerr << nsocs[1].pos <<std::endl;
        }
        else if(unode->msg_send.neigh_cnt==2)
        {
          unode->msg_send.control = CRTOP;
          // std::cerr << nsocs[2].pos <<std::endl;
          // if(i+3 == tail_id)
          // {
          //   unode->msg_send.state |= !SGDRONEREQ;
          // }
        }
        else if(unode->msg_send.neigh_cnt==3)
        {
          // unode->msg_send.control = CHOLDRC;
          // unode->msg_send.control = 0;
          // std::cerr << nsocs[3].pos <<std::endl;
          unode->msg_send.control = CBTOP;
        }

        // ns3::Simulator::ScheduleNow (&rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
        //                                   wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(i+unode->msg_send.neigh_cnt+1));

        // std::cerr << i << " " << unode->msg_send.state << ", " << unode->msg_send.control << std::endl;
      
        // unode->msg_send.neigh_cnt++;
        if(!(unode->msg_send.state & SGDRONEREQ)){
          // rnl::posHold (&unode->wpts, unode->pos);
          // unode->lookaheadindex = 0;
          // unode->toggle_bc = 1;

          unode->msg_send.state = SCENTRE | SSITEREACHED | SGSITEREACHED;
          unode->msg_send.control = 0;

          unode->msg_rec.state &= ~SGDRONEREQ;

          ns3::Simulator::Schedule (pkt_interval, &rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
                                          wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(i-2));
          ns3::Simulator::Schedule (2*pkt_interval, &rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
                                          wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(i+2));
          ns3::Simulator::Schedule (3*pkt_interval, &rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
                                          wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(i+3));
          ns3::Simulator::Schedule (4*pkt_interval, &rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
                                          wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(i+4));

          std::cerr << "complete2 " << i << std::endl;

          if(start_lawn == 50)
          {
            for(int ii=1; ii<tail_id; ii++)
            {
              // std::cerr << "my_arr"<< ii << my_arr[ii] << std::endl;
              if(ii%3>0)// && my_arr[ii]==0)
              {
                // my_arr[ii] = 1;
                // ns3::Simulator::Schedule (2*pkt_interval, &rnl::Planner::doLawnMoverScanning, this, ns3::Seconds (40.0), ii, nsocs[ii].pos);
                rnl::DroneSoc* unode = &nsocs[ii];
                // ns3::Simulator::ScheduleNow (&rnl::Planner::doLawnMoverScanning, this, ns3::Seconds (80.0), ii, unode->pos);
                ns3::Simulator::Schedule (ns3::Seconds (2.0), &rnl::Planner::doLawnMoverScanning, this, ns3::Seconds (220.0), ii, unode->pos);
              }
            }
          }
          if(start_lawn < 80)
          {
            start_lawn++;
          }

          // std::cerr << i << " " << unode->msg_send.state << ", " << unode->msg_send.control << std::endl;
        }
        else
        {
          ns3::Simulator::ScheduleNow (&rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
                                          wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(i+unode->msg_send.neigh_cnt+1));
        }
        // std::cerr << unode->msg_send.state << ", "<<unode->msg_send.control << std::endl;
        // std::cerr << nsocs[2].pos <<std::endl;
        std::cerr << i << " " << unode->msg_send.state << ", " << unode->msg_send.control << std::endl;
      }
      else
      {
        if(!(unode->msg_rec.state & SGDRONEREQ))
        {
          rnl::posHold (&unode->wpts, unode->pos);
          unode->lookaheadindex = 0;
          unode->toggle_bc = 1;

          unode->msg_send.state = SCENTRE | SSITEREACHED | SGSITEREACHED;
          unode->msg_send.control = 0;

          unode->msg_rec.state &= ~SGDRONEREQ;

          if(i-2>0)
          {
            ns3::Simulator::ScheduleNow (&rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
                                            wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(i-2));
          }

          ns3::Simulator::Schedule (pkt_interval, &rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
                                        wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(i+2));
          ns3::Simulator::Schedule (2*pkt_interval, &rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
                                        wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(i+3));

          std::cerr << i << " " << unode->msg_send.state << ", " << unode->msg_send.control << std::endl;
          std::cerr << "complete " << i << std::endl;
        }
        else
        {
          rnl::posHold (&unode->wpts, unode->pos);
          unode->lookaheadindex = 0;
          unode->toggle_bc = 1;

          unode->msg_send.state = SCENTRE | SSITEREACHED | SGSITEREACHED | SGDRONEREQ;
          unode->msg_send.control = CHOLDRC;

          ns3::Simulator::Schedule (2*pkt_interval, &rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
                                        wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(i+unode->msg_send.neigh_cnt));
          // std::cerr << unode->msg_send.state << ", "<<unode->msg_send.control << std::endl;
          std::cerr << i << " " << unode->msg_send.state << ", " << unode->msg_send.control << std::endl;
        }
      }
    }
  }
}

void rnl::Planner::updateWpts (int id)
{
  rnl::DroneSoc* unode = &nsocs[id];
  
  // if (unode -> msg_rec.control & CLANDUNARM)
  // {
  //   try
  //   {
  //     unode->source->Close();
  //     unode->recv_sink->Close();
  //   }
  //   catch(const std::exception& e)
  //   {
  //     std::cerr << e.what() << '\n';
  //   }
  // }
  
  // if ((unode->msg_rec.control & CHOLDRC) && !(unode->msg_send.state & SFIREDET))
  if ((unode->msg_rec.control & CHOLDRC) && !(unode->msg_send.state & (SSITEREACHED | SANCHORING)))
  {
    try
    {
      // if ((unode->msg_rec.p_loc - unode->pos).GetLength() > rnl::RC  && !rnl::Planner::fireDetected (unode->pos))
      if ((unode->msg_rec.p_loc - unode->pos).GetLength() > rnl::RC  && !rnl::Planner::siteReached (unode->pos, id))
      {
        // rnl::getToCircleRange (&unode->wpts, unode->msg_rec.p_loc, unode->pos, rnl::RC, rnl::STEP );
        rnl::getToCircleRange (&unode->wpts, unode->msg_rec.p_loc, unode->wpts[unode->lookaheadindex], rnl::RC, rnl::STEP );
        unode->lookaheadindex = 0;
      }
      // if (id==6){
      //   std::cerr << "6 got HOLD RC" << '\n';
      // }
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
  }

  // if ((unode->msg_rec.control & CCHANGEPAR) && !(unode->msg_send.state & (SFIREDET | SANCHORING)))
  if ((unode->msg_rec.control & CCHANGEPAR) && !(unode->msg_send.state & (SSITEREACHED | SANCHORING | SCHANGEPAR)))
  {
    try
    {
      // std::cerr << unode->id << " has received change parent command" <<std::endl;
      // if ((unode->msg_rec.p_loc - unode->pos).GetLength() > rnl::RC & !rnl::Planner::fireDetected (unode->pos))
      if ((unode->msg_rec.p_loc - unode->pos).GetLength() > rnl::RC && !rnl::Planner::siteReached (unode->pos, id))
      {
        rnl::getToCircleRange (&unode->wpts, unode->msg_rec.p_loc, unode->pos, rnl::RC, rnl::STEP );
        // rnl::getToCircleRange (&unode->wpts, unode->msg_rec.p_loc, unode->wpts[unode->lookaheadindex], rnl::RC, rnl::STEP );
        unode->lookaheadindex = 0;
        unode->msg_send.state = (SCHANGEPAR | SONLINE | SGSITEREACHED | SGDRONEREQ);
        ns3::Simulator::Schedule ( 2*pkt_interval, &rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
                                        wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(unode->id+2));
      }
      // if(id==5){
      //   std::cerr << "5 got CHPAR" << '\n';
      // }
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
  }
  
  // if ((unode->msg_rec.control & CRTOP) && !(unode->msg_send.state & (SFIREDET | SANCHORING))) 
  if ((unode->msg_rec.control & CRTOP) && !(unode->msg_send.state & (SSITEREACHED | SANCHORING))) 
  {
    try
    {
      if ((unode->pos - unode->msg_rec.p_loc).GetLength() > rnl::RC)
      {
        std::cerr << (unode->pos - unode->msg_rec.p_loc).GetLength()<< " is greater than rc"<<std::endl;
      }
      // unode ->circle_dir = -1;
      unode ->circle_dir = 1;  // may be -1 (not sure)
      std::cerr << unode->id << " has received go right command" <<std::endl;
      // rnl::getCirclewpts(&unode->wpts, unode->msg_rec.p_loc, unode->pos, rnl::RC, rnl::DTHETA, -1, rnl::STEP);
      // rnl::getCirclewpts(&unode->wpts, unode->msg_rec.p_loc, unode->pos, rnl::RC, rnl::DTHETA, 1, rnl::STEP);
      // ns3::Vector3D posNew(unode->msg_rec.p_loc.x + RC/sqrt(2), unode->msg_rec.p_loc.y - RC/sqrt(2), unode->msg_rec.p_loc.z);
      ns3::Vector3D posNew(unode->msg_rec.p_loc.x, unode->msg_rec.p_loc.y - rnl::RC, unode->msg_rec.p_loc.z);
      rnl::getTrajectory (&unode->wpts, unode->pos, posNew, rnl::STEP);
      unode->lookaheadindex = 0;
      unode->msg_send.state = (SANCHORING | SRIGHT | SGSITEREACHED | SGDRONEREQ);
      // unode->anch_id = 0;
      unode->anch_id = unode->msg_rec.p_id;
      unode->anch_pos = unode->msg_rec.p_loc;
      unode->msg_send.control = (CCHANGEPAR);
      unode->msg_send.p_loc = unode->msg_rec.p_loc;
      // unode->msg_send.p_id = 0;
      unode->msg_send.p_id = unode->msg_rec.p_id;
      std::cerr << "RTOP Called"<<std::endl;
      // ns3::Simulator::Schedule ( 2 * pkt_interval, &rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
      //                                 wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(1));
      // ns3::Simulator::Schedule ( 2 * pkt_interval, &rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
      //                                 wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(unode->msg_rec.p_id+3+1));
      ns3::Simulator::Schedule ( 2*pkt_interval, &rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
                                      wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(unode->msg_rec.p_id+3+1));
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
  }

  // if ((unode->msg_rec.control & CLTOP) && !(unode->msg_send.state & (SFIREDET | SANCHORING)))
  if ((unode->msg_rec.control & CLTOP) && !(unode->msg_send.state & (SSITEREACHED | SANCHORING)))
  {
    try
    {
      std::cerr << unode->id << " has received go left command" <<std::endl;
      // rnl::getCirclewpts(&unode->wpts, unode->msg_rec.p_loc, unode->pos, rnl::RC, rnl::DTHETA, 1, rnl::STEP);
      // rnl::getCirclewpts(&unode->wpts, unode->msg_rec.p_loc, unode->pos, rnl::RC, rnl::DTHETA, -1, rnl::STEP);
      // ns3::Vector3D posNew(unode->msg_rec.p_loc.x - RC/sqrt(2), unode->msg_rec.p_loc.y + RC/sqrt(2), unode->msg_rec.p_loc.z);
      ns3::Vector3D posNew(unode->msg_rec.p_loc.x, unode->msg_rec.p_loc.y + rnl::RC, unode->msg_rec.p_loc.z);
      rnl::getTrajectory (&unode->wpts, unode->pos, posNew, rnl::STEP);
      // unode ->circle_dir = 1;
      unode ->circle_dir = -1; // may be +1, not sure
      unode->lookaheadindex = 0;
      unode->msg_send.state = (SANCHORING | SLEFT | SGSITEREACHED | SGDRONEREQ);
      unode->msg_send.control = (CCHANGEPAR);
      unode->anch_pos = unode->msg_rec.p_loc;
      // unode->anch_id = 0;
      unode->anch_id = unode->msg_rec.p_id;
      unode->msg_send.p_loc = unode->msg_rec.p_loc;
      unode->msg_send.p_id = unode->msg_rec.p_id;
      std::cerr << "LTOP Called"<<std::endl;
      // ns3::Simulator::Schedule ( 2 * pkt_interval, &rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
      //                               wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(1));
      // ns3::Simulator::Schedule ( 2 * pkt_interval, &rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
      //                               wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(unode->msg_rec.p_id+2+1));
      ns3::Simulator::Schedule ( 2*pkt_interval, &rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
                                    wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(unode->msg_rec.p_id+2+1));

      start_left = 0;
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
  }

  if ((unode->msg_rec.control & CBTOP) && !(unode->msg_send.state & (SSITEREACHED | SANCHORING)))
  {
    try
    {
      std::cerr << unode->id << " has received go behind command" <<std::endl;
      // rnl::getCirclewpts(&unode->wpts, unode->msg_rec.p_loc, unode->pos, rnl::RC, rnl::DTHETA, 1, rnl::STEP);
      // rnl::getCirclewpts(&unode->wpts, unode->msg_rec.p_loc, unode->pos, rnl::RC, rnl::DTHETA, -1, rnl::STEP);
      // ns3::Vector3D posNew(unode->msg_rec.p_loc.x - RC/sqrt(2), unode->msg_rec.p_loc.y + RC/sqrt(2), unode->msg_rec.p_loc.z);
      ns3::Vector3D posNew(unode->msg_rec.p_loc.x - rnl::RC, unode->msg_rec.p_loc.y, unode->msg_rec.p_loc.z);
      rnl::getTrajectory (&unode->wpts, unode->pos, posNew, rnl::STEP);
      // unode ->circle_dir = 1;
      unode ->circle_dir = 0;
      unode->lookaheadindex = 0;
      unode->msg_send.state = (SANCHORING | SCENTRE | SGSITEREACHED | SGDRONEREQ);
      unode->msg_send.control = (CHOLDRC);
      unode->anch_pos = unode->msg_rec.p_loc;
      // unode->anch_id = 0;
      unode->anch_id = unode->msg_rec.p_id;
      unode->msg_send.p_loc = unode->pos;
      unode->msg_send.p_id = unode->id;
      std::cerr << "BTOP Called"<<std::endl;
      // ns3::Simulator::Schedule ( 2 * pkt_interval, &rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
      //                                   wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(1));
      // ns3::Simulator::Schedule ( 2 * pkt_interval, &rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
      //                                   wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(unode->msg_rec.p_id+2+1));
      ns3::Simulator::Schedule ( 2*pkt_interval, &rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
                                        wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(unode->id+2));
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
  }
  
  // if ((unode->msg_rec.state & SGDRONEREQ))
  // {
  //   try
  //   {
  //     unode->msg_send.state |= SGDRONEREQ;
  //     unode->msg_rec.state &= (~SGDRONEREQ);
  //   }
  //   catch(const std::exception& e)
  //   {
  //     std::cerr << e.what() << '\n';
  //   }
  // }
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
    if (nsocs[i].lookaheadindex + 1 < nsocs[i].wpts.size() 
          && withinThreshold(&nsocs[i]))
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
    updateWpts(i);
  }
}

// bool rnl::Planner::isDirectionSame(int i1, int i2)
// {
//   return (i1%2 == i2%2);
// }

// std::pair<int, ns3::Vector3D> rnl::Planner::checkUpdateAnchor (int id)
// {
//   rnl::DroneSoc* unode = &nsocs[id];
//   std::pair<int,ns3::Vector3D> anchor (unode->anch_id, unode->anch_pos);
//   for (auto x: unode->nbt.one_hop)
//   {
//     if (isDirectionSame(x.first,id))
//     {
//       if (ns3::CalculateDistance(x.second, unode->pos) < ns3::CalculateDistance(anchor.second, unode->pos))
//       {
//         anchor = x;
//       }
//     }
//   }
//   return anchor;
  
// }

std::pair<int, ns3::Vector3D> rnl::Planner::getNearestDrone (int id)
{
  rnl::DroneSoc* unode = &nsocs[id];
  std::pair<int,ns3::Vector3D> anchor (unode->anch_id, unode->anch_pos);
  for (auto x: unode->nbt.one_hop)
  {
      if (ns3::CalculateDistance(x.second, unode->pos) < ns3::CalculateDistance(anchor.second, unode->pos))
      {
        anchor = x;
      }
  }
  return anchor;
}

// void rnl::Planner::checkDroneReq (int id)
// {
//   // std::cerr << "One Hops of " << this->id << std::endl;
//   for (auto x:nsocs[id].nbt.one_hop)
//   {
//     std::cerr << x.first << ", " << x.second << std::endl;
//   }
//   std::pair<int,ns3::Vector3D> temp = getNearestDrone (id);
//   if (isDirectionSame(id, temp.first) || temp.first <=0)
//   {
//     nsocs[id].msg_send.state |=SGDRONEREQ;
//   }
// }

void rnl::Planner::doLawnMoverScanning (ns3::Time interval, int id, ns3::Vector3D pos0)
{
  std::cerr << "----------lawn movering called----------"<< std::endl;

  // my_arr[id] = 1;

  int dir = 1;
  if(id%3 == 1)
  {
    dir = 1;
  }
  else if(id%3 == 2)
  {
    dir = -1;
  }

  rnl::DroneSoc* unode = &nsocs[id];

  ns3::Simulator::ScheduleNow (&rnl::DroneSoc::setSenderTCP, unode, unode->source->GetNode(), rnl::IP_BASE + std::to_string(id+1),
   rnl::IP_BASE + std::to_string(num_nodes), ns3::Seconds (150 + 3*id));

  if((id-1)%3 == 0)
  {
    int fake_id = id-1;

    rnl::DroneSoc* fake_unode = &nsocs[fake_id];

    ns3::Simulator::ScheduleNow (&rnl::DroneSoc::setSenderTCP, fake_unode, fake_unode->source->GetNode(), rnl::IP_BASE + std::to_string(fake_id+1),
      rnl::IP_BASE + std::to_string(num_nodes), ns3::Seconds (150 + 3*fake_id));
  }

  ns3::Vector3D pos1(pos0.x + rnl::RC/3.2, pos0.y + dir*rnl::RC/2, pos0.z);
  ns3::Vector3D pos2(pos0.x - rnl::RC/3.2, pos0.y + dir*rnl::RC/2, pos0.z);
  ns3::Vector3D pos3(pos0.x - rnl::RC/3.2, pos0.y                , pos0.z);
  ns3::Vector3D pos4(pos0.x + rnl::RC/3.2, pos0.y                , pos0.z);
  ns3::Vector3D pos5(pos0.x + rnl::RC/3.2, pos0.y - dir*rnl::RC/4, pos0.z);
  ns3::Vector3D pos6(pos0.x - rnl::RC/3.2, pos0.y - dir*rnl::RC/4, pos0.z);
  ns3::Vector3D pos7(pos0.x - rnl::RC/3.2, pos0.y - dir*rnl::RC/2, pos0.z);
  ns3::Vector3D pos8(pos0.x + rnl::RC/3.2, pos0.y - dir*rnl::RC/2, pos0.z);

  rnl::getTrajectory (&unode->wpts, pos0, pos1, rnl::STEP);
  rnl::getTrajectoryContinue (&unode->wpts, pos1, pos2, rnl::STEP);
  rnl::getTrajectoryContinue (&unode->wpts, pos2, pos3, rnl::STEP);
  rnl::getTrajectoryContinue (&unode->wpts, pos3, pos4, rnl::STEP);
  rnl::getTrajectoryContinue (&unode->wpts, pos4, pos5, rnl::STEP);
  rnl::getTrajectoryContinue (&unode->wpts, pos5, pos6, rnl::STEP);
  rnl::getTrajectoryContinue (&unode->wpts, pos6, pos7, rnl::STEP);
  rnl::getTrajectoryContinue (&unode->wpts, pos7, pos8, rnl::STEP);
  rnl::getTrajectoryContinue (&unode->wpts, pos8, pos0, rnl::STEP);

  unode->lookaheadindex = 0;

  std::cerr << id << " lawn movering"<< std::endl;

  nsocs[id].msg_rec.state &= ~SGDRONEREQ;
  nsocs[id].msg_send.state = SLAWNMOVERING | SGSITEREACHED | SSITEREACHED;

  if(id%3 == 1)
  {
    nsocs[id].msg_send.state |= SLEFT;
  }
  else if(id%3 == 2)
  {
    nsocs[id].msg_send.state |= SRIGHT;
  }

  ns3::Simulator::Schedule(interval, &rnl::Planner::doLawnMoverScanning, this, interval, id, pos0);
}

// void rnl::Planner::updateSocsSenders ()
void rnl::Planner::updateSocs ()
{
  for (int i = 1; i < tail_id; ++i)
  {
    // if (rnl::Planner::fireDetected (nsocs[i].pos) && !(nsocs[i].msg_send.state & SFIREDET) )
    // if (rnl::Planner::siteReached (nsocs[i].pos, i) && !(nsocs[i].msg_send.state & SSITEREACHED) )
    // {
    //   rnl::posHold (&nsocs[i].wpts, nsocs[i].pos);
    //   nsocs[i].lookaheadindex = 0;
    //   // nsocs[i].msg_send.state = SFIREDET;
    //   nsocs[i].msg_send.state = SSITEREACHED;
    //   // ns3::Simulator::Schedule ( num_nodes * pkt_interval, &rnl::Planner::checkDroneReq, this, i);
    //   nsocs[i].toggle_bc = 1;
    // }

    if (rnl::Planner::siteReached (nsocs[i].pos, i) && i%3 > 0)
    {
      if(nsocs[i].msg_rec.state & SGDRONEREQ)
      {
        rnl::posHold (&nsocs[i].wpts, nsocs[i].pos);
        nsocs[i].lookaheadindex = 0;
      }
      // nsocs[i].msg_send.state = SFIREDET;
      nsocs[i].msg_send.state &= SLAWNMOVERING;
      nsocs[i].msg_send.state = (SSITEREACHED | SGSITEREACHED) | (nsocs[i].msg_rec.state & SGDRONEREQ);
      nsocs[i].msg_send.control = 0;
      // ns3::Simulator::Schedule ( num_nodes * pkt_interval, &rnl::Planner::checkDroneReq, this, i);
      nsocs[i].toggle_bc = 1;

      if(i%3 == 1){
        nsocs[i].msg_send.state |= SLEFT;
      }
      if(i%3 == 2){
        nsocs[i].msg_send.state |= SRIGHT;
      }

      rnl::DroneSoc* unode = &nsocs[i];

      ns3::Simulator::Schedule (2*pkt_interval, &rnl::DroneSoc::setSender, unode, unode->source->GetNode(),
                                        wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(i-(i%3)+1));
      std::cerr << i << " " << unode->msg_send.state << ", " << unode->msg_send.control << std::endl;
    }

    // if ((nsocs[i].msg_send.state & SANCHORING) )
    // {
    //   std::pair <int, ns3::Vector3D> _anch = checkUpdateAnchor (i);
    //   if (_anch.first > nsocs[i].anch_id)
    //   {
    //     rnl::getCirclewpts(&nsocs[i].wpts, _anch.second, nsocs[i].pos, rnl::RC, rnl::DTHETA, nsocs[i].circle_dir, rnl::STEP);
    //     nsocs[i].lookaheadindex = 0;
    //     nsocs[i].anch_id = _anch.first;
    //     nsocs[i].anch_pos = _anch.second;
    //     nsocs[i].msg_send.p_loc = _anch.second;
    //     nsocs[i].msg_send.p_id = _anch.first;
    //     std::cerr << "SOCSEN Called "<< _anch.first << std::endl;
    //     // nsocs[i].setSender (nsocs[i].source->GetNode(), wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(_anch.first+1));
    //     ns3::Simulator::Schedule ((num_nodes+1) * pkt_interval, &rnl::DroneSoc::setSender, &nsocs[i], nsocs[i].source->GetNode(),
    //                                 wifi_prop.tid_val(), rnl::IP_BASE + std::to_string(_anch.first+1));
    //   }
    // }
  }
}

void rnl::Planner::advancePos (ns3::Time interval)
{
  ros::spinOnce();
  updatePosSocs ();
  incLookAhead ();
  //updateStateofLeader ();
  updateStateofCentre ();
  updateSocsfromRec ();
  // updateSocsSenders ();
  updateSocs ();
  for (int i = 0; i < nsocs.size(); ++i)
  {
    if (nsocs[i].wpts.size())
    {
      nsocs[i].publishLookAhead();
    }
  }
  ns3::Simulator::Schedule(interval, &rnl::Planner::advancePos, this, interval);
}

void rnl::Planner::takeOFF (double _t)
{
  if ((ns3::Simulator::Now ().GetSeconds() - _t) < 1)
  {
    ros::spinOnce();
    ns3::Simulator::Schedule (ns3::Seconds(0.1), &rnl::Planner::takeOFF, this, _t);
  }
  
  else{
    setLeaderExplorePath ();
  }
}

void rnl::Planner::startSimul()
{
  for (int i =0; i< nsocs.size(); ++i)
  {
    ns3::Simulator::Schedule (ns3::Seconds (2.0) + i*pkt_interval, &rnl::DroneSoc::sendPacket, &nsocs[i], pkt_interval, num_nodes);
    // ns3::Simulator::Schedule (ns3::Seconds (2.5) + i*pkt_interval, &rnl::DroneSoc::sendPacket, &nsocs[i],
    //                             pkt_interval, num_nodes);
    nsocs[i].setRecv (wifi_prop.c.Get(i), wifi_prop.tid_val());
    nsocs[i].initializeRosParams (nh);

  }
  initializeMobility();

  nsocs[num_nodes-1].setRecvTCP (wifi_prop.c.Get(num_nodes-1), rnl::IP_BASE + std::to_string(num_nodes-1), num_nodes, stopTime);

  ns3::Simulator::ScheduleNow (&rnl::Planner::takeOFF, this, ns3::Simulator::Now ().GetSeconds());
  ns3::Simulator::Schedule (ns3::Seconds (2.0) + 5 * (num_nodes+1) * pkt_interval, &rnl::Planner::advancePos, this, pos_interval);
  ns3::Simulator::Stop(stopTime);
  ns3::AnimationInterface anim ("planner_ns3_anim.xml");
  anim.SetMaxPktsPerTraceFile(9999999);
  ns3::Simulator::Run();
  ns3::Simulator::Destroy();
}
