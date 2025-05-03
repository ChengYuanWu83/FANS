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

    // wifi.SetStandard (ns3::WIFI_STANDARD_80211b);
    wifi.SetStandard (ns3::WIFI_STANDARD_80211g);
    wifi.SetRemoteStationManager("ns3::AarfWifiManager");
    

    // This is one parameter that matters when using FixedRssLossModel
    // set it to zero; otherwise, gain will be added

    // wifiPhy.Set ("TxGain", ns3::DoubleValue(0));
    // wifiPhy.Set ("RxGain", ns3::DoubleValue (0));
    // wifiPhy.Set ("RxSensitivity", ns3::DoubleValue (-77.5));
    // wifiPhy.Set ("TxPowerStart", ns3::DoubleValue (20.0));
    // wifiPhy.Set ("TxPowerEnd", ns3::DoubleValue (20.0));

    // wifiPhy.Set ("ShortPlcpPreambleSupported", ns3::BooleanValue (true) );
    
    if(band_5GHz_enable){
        // wifiPhy.Set("PhyBand", ns3::EnumValue(ns3::WifiPhyBand::WIFI_PHY_BAND_5GHZ));
        wifiPhy.Set("Frequency", ns3::UintegerValue(5180));
    }
    else{
        // wifiPhy.Set("PhyBand", ns3::EnumValue(ns3::WifiPhyBand::WIFI_PHY_BAND_2_4GHZ));
        wifiPhy.Set("Frequency", ns3::UintegerValue(2412));
    }
    
    
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


void rnl::DroneSoc::setBcSender (ns3::Ptr<ns3::Node> node, ns3::TypeId tid)
{
    this->source_bc = ns3::Socket::CreateSocket (node, tid);
    std::string bcAddr = IP_BASE + "255";
    ns3::InetSocketAddress remote = ns3::InetSocketAddress (ns3::Ipv4Address(bcAddr.c_str()), 9);
    this->source_bc->SetAllowBroadcast (true);
    this->source_bc->Connect (remote);
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

// void rnl::DroneSoc::setRecvTCP (ns3::Ptr<ns3::Node> node, const std::string& ip, int num_nodes, ns3::Time stopTime)
// {
//   ns3::Address sinkAddress (ns3::InetSocketAddress (ip.c_str(), 8080));
//   ns3::PacketSinkHelper packetSinkHelper ("ns3::TcpSocketFactory", ns3::InetSocketAddress (ns3::Ipv4Address::GetAny (), 8080));
//   ns3::ApplicationContainer sinkApps = packetSinkHelper.Install (node);
//   ns3::Ptr<ns3::PacketSink> packetSink = sinkApps.Get (0)->GetObject<ns3::PacketSink> ();
//   packetSink->TraceConnectWithoutContext ("Rx", ns3::MakeBoundCallback (&TraceSink, num_nodes-1));
//   sinkApps.Start (ns3::Seconds (80));
//   sinkApps.Stop (stopTime);
// }

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
                    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED: // goal packet
                    {
                        mavlink_set_position_target_local_ned_t pt;
                        mavlink_msg_set_position_target_local_ned_decode(&msg, &pt);

                        if(pt.target_system != this->id){
                            // std::cerr << "Received packet not belong to it. Discard" << std::endl; 
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
                    case MAVLINK_MSG_ID_MISSION_ACK:	// ACK packet
                    {
                        mavlink_mission_ack_t ack;
                        mavlink_msg_mission_ack_decode(&msg, &ack);

                        if(ack.target_system != this->id){
                            // std::cerr << "Received packet not belong to it. Discard" << std::endl; 
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

                    case MAVLINK_MSG_ID_COMMAND_ACK:
                    {
                        mavlink_command_ack_t ack;
                        mavlink_msg_command_ack_decode(&msg, &ack);
                        
                        if(ack.target_system != this->id){
                            // std::cerr << "Received packet not belong to it. Discard" << std::endl; 
                            return;
                        }
                        
                        // Handle command acknowledgment
                        std::stringstream ss;
                        ss << unsigned(msg.sysid) << ":";
                        
                        if(ack.command == MAV_CMD_NAV_WAYPOINT && ack.result == MAV_RESULT_ACCEPTED){
                            ss << " drone arrived at waypoint";
                            
                            // If this is acknowledgment for an image batch
                            if(ack.progress == 101) { // Special value for image batches
                                ss << " and image batch " << (int)ack.result_param2 << " received";
                                

                            }
                            std_msgs::String response;
                            response.data = ss.str();
                            
                            this->arrive_response_pub.publish(response);
                        }
                        if(ack.command == MAV_CMD_IMAGE_START_CAPTURE && ack.result == MAV_RESULT_ACCEPTED){
                            ss << " drone arrived at waypoint";
                            // Move to next batch if we're the image sender
                            if(this->batch_in_progress) {
                                this->current_batch_idx++;

                                // If there are more batches to send, schedule the next one
                                if(!this->image_batch_queue.empty()) {
                                    ns3::Simulator::ScheduleNow(&rnl::DroneSoc::sendNextImageBatch, this);
                                }
                                else {
                                    this->batch_in_progress = false;
                                    std::cerr << "All image batches sent successfully" << std::endl;
                                }
                            }
                        }
                        else if(ack.result == MAV_RESULT_FAILED) {
                            ss << " command failed";
                        }
                        else if(ack.result == MAV_RESULT_UNSUPPORTED) {
                            ss << " command unsupported";
                        }
                        else if(ack.result == MAV_RESULT_TEMPORARILY_REJECTED) {
                            ss << " command temporarily rejected";
                        }
                        else if(ack.result == MAV_RESULT_DENIED) {
                            ss << " command denied";
                        }
                        else {
                            ss << " unknown result: " << ack.result;
                        }


                        std::cerr << "Node: " << this->id << " received command acknowledgment from " 
                                 << inetSenderAddr.GetIpv4() << " (node " << unsigned(msg.sysid) << "): " 
                                 << ss.str() << std::endl;
                        break;
                    }

                    case MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE: // image metadata packet
                    {  

                        if(this->id > 0){
							// std::cerr << "Received packet not belong to it. Discard" << std::endl; 
							return;
						}

                        std::pair<uint8_t, uint8_t> sender(msg.sysid, msg.compid);

                        mavlink_data_transmission_handshake_t handshake;
                        mavlink_msg_data_transmission_handshake_decode(&msg, &handshake);
                        
                        // Check image data type
                        if (handshake.type == MAVLINK_DATA_STREAM_IMG_JPEG) {
                            std::cerr << "Received image handshake: " << handshake.size << " bytes, " 
                                      << handshake.width << "x" << handshake.height << ", " 
                                      << static_cast<int>(handshake.packets) << " packets, batch index:" 
                                      << static_cast<int>(handshake.jpg_quality) << std::endl;


                            auto& buffer = image_buffers_[sender];
                            if (handshake.jpg_quality == 0) {
                                // First batch: Initialize
                                buffer.reset();
                                buffer.receiving = true;
                                buffer.total_image_size = handshake.size;
                                buffer.total_received = 0;
                                buffer.width = handshake.width;
                                buffer.height = handshake.height;
                                buffer.data.resize(handshake.size);
                            }
                            
                            buffer.batch_idx = handshake.jpg_quality;
                            buffer.batch_byte_offset = buffer.total_received;
                            buffer.batch_packets = handshake.packets;
                            buffer.batch_packet_received.resize(handshake.packets, false);
                            // buffer.packet_received.resize(handshake.packets, false);
                            buffer.last_update = ns3::Simulator::Now().GetMicroSeconds();
                        }
                        break;
                    }

                    case MAVLINK_MSG_ID_ENCAPSULATED_DATA: 
                    {
                        
                        if(this->id > 0){
							std::cerr << "Received packet not belong to it. Discard" << std::endl; 
							return;
						}
                        std::pair<uint8_t, uint8_t> sender(msg.sysid, msg.compid);

                        auto it = image_buffers_.find(sender);
                        if (it == image_buffers_.end() || !it->second.receiving) {
                            // Make sure we are receiving an image
                            break;
                        }
                        auto& buffer = it->second;
                        buffer.last_update = ns3::Simulator::Now().GetMicroSeconds();

                        mavlink_encapsulated_data_t img_data;
                        mavlink_msg_encapsulated_data_decode(&msg, &img_data);
                        
                        // Check if the package number is valid
                        if (img_data.seqnr >= buffer.batch_packets) {
                            std::cerr << "Invalid packet sequence number: " << static_cast<int>(img_data.seqnr) 
                              << " (max: " << static_cast<int>(buffer.batch_packets - 1) << ")" << std::endl;
                            break;
                        }
                        // Calculate where this packet should be placed
                        const size_t CHUNK_SIZE = MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN; // 253 bytes
                        size_t offset = buffer.batch_byte_offset + img_data.seqnr * CHUNK_SIZE;
                        size_t bytes_to_copy = std::min(CHUNK_SIZE, buffer.total_image_size - offset);

                        if (offset + bytes_to_copy <= buffer.data.size()) {

                            memcpy(&buffer.data[offset], img_data.data, bytes_to_copy);
                            buffer.total_received += bytes_to_copy;
                            buffer.batch_packet_received[img_data.seqnr] = true;
                            
                            std::cerr << "Received image packet " << static_cast<int>(img_data.seqnr + 1) 
                              << "/" << static_cast<int>(buffer.batch_packets) << std::endl;
                            
                              if(buffer.isBatchComplete()){
                                std::cerr << "All image packets in "<< static_cast<int>(buffer.batch_idx)<<" batches received, sending ACK..." << std::endl;
                                
                                this->sendArrivedPacket(msg.sysid, MAV_CMD_IMAGE_START_CAPTURE);
                            }

                            // Check if all packets have been received
                            if (buffer.isImageComplete()) {
                                std::cerr << "All image packets received, decoding..." << std::endl;
                                cv::Mat image = cv::imdecode(buffer.data, cv::IMREAD_COLOR);

                                if (image.empty()) {
                                    std::cerr << "Failed to decode image data" << std::endl;
                                    buffer.reset();
                                    return;
                                }
                                
                                // Check that the image size is as expected
                                if (image.cols != buffer.width || image.rows != buffer.height) {
                                    std::cerr << "Decoded image size (" << image.cols << "x" << image.rows 
                                            << ") doesn't match expected size (" << buffer.width << "x" 
                                            << buffer.height << ")" << std::endl;
                                }

                                if (this->imagePublish == 1) {
                                    std_msgs::Header header;
                                    header.stamp = ros::Time::now();          
                                    header.frame_id = std::to_string(msg.sysid); 

                                    sensor_msgs::ImagePtr ros_image = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
                                    this->drone_image_pub.publish(ros_image);
                                    std::cerr << "Image published to ROS topic" << std::endl;
                                }
                                else{
                                    std::string timestamp = std::to_string(ns3::Simulator::Now().GetMicroSeconds());
                                    std::string filename = "mavlink_image_" + timestamp + ".png";
                                    
                                    // Save as png
                                    std::vector<int> compression_params;
                                    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
                                    compression_params.push_back(1); // 0-9, 9 is the highest compression rate
                                    
                                    bool success = cv::imwrite(filename, image, compression_params);
                                    
                                    if (success) {
                                        std::cerr << "Image successfully decoded and saved to: " << filename << std::endl;
                                    } else {
                                        std::cerr << "Failed to save image to file" << std::endl;
                                    }
                                }
                                
                                buffer.reset();
                            }
                        }
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

void rnl::DroneSoc::sendNextImageBatch() {
    std::cerr << "sendNextImageBatch" << std::endl;
    if (this->batch_in_progress || this->image_batch_queue.empty()) {
        return;
    }
    
    this->batch_in_progress = true;
    std::vector<uchar> batch_buffer_ = this->image_batch_queue.front();
    this->image_batch_queue.pop();
    
    // Calculate chunk count for this batch
    const size_t CHUNK_SIZE = MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN;
    size_t chunk_count = (batch_buffer_.size() + CHUNK_SIZE - 1) / CHUNK_SIZE;
    
    // === Step 1: Handshake for this batch ===
    mavlink_data_transmission_handshake_t handshake;
    handshake.type = MAVLINK_DATA_STREAM_IMG_JPEG;
    handshake.size = this->image_info.buffer_size;
    handshake.width = this->image_info.cols;
    handshake.height = this->image_info.rows;
    handshake.packets = chunk_count;
    handshake.payload = CHUNK_SIZE;
    handshake.jpg_quality = this->current_batch_idx; // Use for batch index tracking
    
    mavlink_message_t handshake_msg;
    mavlink_msg_data_transmission_handshake_encode(this->id, 200, &handshake_msg, &handshake);
    
    uint8_t handshake_buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t handshake_len = mavlink_msg_to_send_buffer(handshake_buffer, &handshake_msg);
    ns3::Ptr<ns3::Packet> packet = ns3::Create<ns3::Packet>(handshake_buffer, handshake_len);
    this->source_bc->Send(packet);
    
    std::cerr << "Sent handshake for batch " << this->current_batch_idx + 1 << "/" 
              << this->image_batch_queue.size() + 1 << " (" << chunk_count << " chunks)" << std::endl;
    
    // Schedule chunk sending for this batch
    ns3::Simulator::ScheduleNow(&rnl::DroneSoc::sendImageChunk, this, 0, chunk_count, batch_buffer_);
}

void rnl::DroneSoc::sendImageChunk(uint32_t i, uint32_t chunk_count, std::vector<uchar>& jpeg_buffer) {
    if (i >= chunk_count) {
        std::cerr << "Image transmission complete" << std::endl;
        return;
    }

    const size_t CHUNK_SIZE = MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN; // 253 bytes
    mavlink_message_t data_msg;
    mavlink_encapsulated_data_t img_data;
    img_data.seqnr = i;
    
    // Fill the data field
    size_t bytes_to_copy = (i == chunk_count - 1) ? 
        (jpeg_buffer.size() - i * CHUNK_SIZE) : CHUNK_SIZE;
    
    memset(img_data.data, 0, CHUNK_SIZE);
    memcpy(img_data.data, &jpeg_buffer[i * CHUNK_SIZE], bytes_to_copy);
    
    // Encode the message
    mavlink_msg_encapsulated_data_encode(this->id, 200, &data_msg, &img_data);
    
    // Get the serialized message
    uint8_t data_buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t data_len = mavlink_msg_to_send_buffer(data_buffer, &data_msg);
    
    // In a real application, you would send this buffer over your communication channel
    ns3::Ptr<ns3::Packet> packet = ns3::Create<ns3::Packet> (data_buffer, data_len);
    this->source_bc->Send (packet);
    std::cerr << "Sending MAVLink image chunk "<< unsigned(i+1) <<"/" << unsigned(chunk_count) << ", size: "<< unsigned(bytes_to_copy) << " bytes" << std::endl;

    ns3::Simulator::Schedule(ns3::MilliSeconds(5), &rnl::DroneSoc::sendImageChunk, this, i + 1, chunk_count, jpeg_buffer);
}


void rnl::DroneSoc::sendImagePacket(){

	// Convert ROS image to OpenCV format
	cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(this->imagePtr, "bgr8");

	// Compress the image to JPEG format
	std::vector<uchar> jpeg_buffer;
	std::vector<int> compression_params;
	compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
	compression_params.push_back(this->jpeg_quality);
	
	cv::imencode(".jpg", cv_ptr->image, jpeg_buffer, compression_params);

    this->image_info.reset();
    this->image_info.rows = cv_ptr->image.rows;
    this->image_info.cols = cv_ptr->image.cols;
    this->image_info.buffer_size = jpeg_buffer.size();
	
	// Calculate the number of chunks needed
	const size_t CHUNK_SIZE = MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN; // 253 bytes
    const size_t MAX_CHUNKS = 255;
	size_t total_chunk_count = (jpeg_buffer.size() + CHUNK_SIZE - 1) / CHUNK_SIZE;
    size_t total_batch_count = (total_chunk_count + MAX_CHUNKS - 1) / MAX_CHUNKS;
	
	if (total_batch_count > 1) {
		std::cerr << "Image too large for MAVLink transmission (needs " << total_chunk_count <<" chunks)" << std::endl;
		// return;
	}

    for (size_t batch_idx = 0; batch_idx < total_batch_count; ++batch_idx) {
        size_t batch_chunk_start = batch_idx * MAX_CHUNKS;
        size_t batch_chunk_end = std::min(batch_chunk_start + MAX_CHUNKS, total_chunk_count);

        size_t byte_start = batch_chunk_start * CHUNK_SIZE;
        size_t byte_end = std::min(byte_start + MAX_CHUNKS * CHUNK_SIZE, jpeg_buffer.size());

        std::vector<uchar> batch_buffer(jpeg_buffer.begin() + byte_start, jpeg_buffer.begin() + byte_end);
        image_batch_queue.push(batch_buffer);
        
        
    }
    this->sendNextImageBatch();

	// // Send DATA_TRANSMISSION_HANDSHAKE message, which describe the image to be sent
	// mavlink_message_t handshake_msg;
	// mavlink_data_transmission_handshake_t handshake;
	// handshake.type = MAVLINK_DATA_STREAM_IMG_JPEG;
	// handshake.size = jpeg_buffer.size();
	// handshake.width = cv_ptr->image.cols;
	// handshake.height = cv_ptr->image.rows;
	// handshake.packets = chunk_count;
	// handshake.payload = CHUNK_SIZE;
	// handshake.jpg_quality = this->jpeg_quality;
	
	// mavlink_msg_data_transmission_handshake_encode(this->id, 200, &handshake_msg, &handshake);
	
	// // Get the serialized message
	// uint8_t handshake_buffer[MAVLINK_MAX_PACKET_LEN];
	// uint16_t handshake_len = mavlink_msg_to_send_buffer(handshake_buffer, &handshake_msg);

	// ns3::Ptr<ns3::Packet> packet = ns3::Create<ns3::Packet> (handshake_buffer, handshake_len);
    // this->source_bc->Send (packet);
    // std::cerr <<"Arrived target. Send imgae to GCS" << std::endl;

    // // Send the image data chunks
    // ns3::Simulator::ScheduleNow(&rnl::DroneSoc::SendImageChunk, this, 0, chunk_count, jpeg_buffer);

}

void rnl::DroneSoc::sendArrivedPacket(uint32_t targetId, uint32_t cmdId){
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
    std::cerr << "Arrived target. Send COMMAND_ACK to GCS" << std::endl;
}

// void rnl::DroneSoc::sendImageACKPacket(uint32_t targetId){
//     mavlink_message_t msg;
//     uint8_t buf[MAVLINK_MAX_PACKET_LEN];

//     mavlink_msg_mission_ack_pack(this->id, 200, &msg, targetId, 200, MAV_RESULT_ACCEPTED, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
//     uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

//     ns3::Ptr<ns3::Packet> packet = ns3::Create<ns3::Packet> (buf, len);
//     this->source_bc->Send (packet);
//     std::cerr <<"Arrived target. Send ACK to GCS" << std::endl;
// }

void rnl::DroneSoc::sendGoalPacket (const geometry_msgs::PoseStamped::ConstPtr& _pos) { // send position  and orientation
    
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

void rnl::DroneSoc::posSubCb (const geometry_msgs::PoseStamped& _pos)
{
    this->pos.x = _pos.pose.position.x;
    this->pos.y = _pos.pose.position.y;
    this->pos.z = _pos.pose.position.z;

}

void rnl::DroneSoc::imageSubCb (const sensor_msgs::ImageConstPtr& _msg)
{
    this->imagePtr = _msg;
}

void rnl::DroneSoc::publishLookAhead ()
{
    geometry_msgs::Pose _lka;
    _lka.position.x = this->wpts[this->lookaheadindex].x;
    _lka.position.y = this->wpts[this->lookaheadindex].y;
    _lka.position.z = this->wpts[this->lookaheadindex].z;

    std::vector<float> quaternion = rnl::lookAtOrigin(_lka.position.x, _lka.position.y, _lka.position.z);
    
    _lka.orientation.x = quaternion[0];
    _lka.orientation.y = quaternion[1];
    _lka.orientation.z = quaternion[2];
    _lka.orientation.w = quaternion[3];
    
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
            nsocs[i].arrive_response_pub= nh.advertise<std_msgs::String>("/gcs/arrived", 10);
            nsocs[i].drone_image_pub    = nh.advertise<sensor_msgs::Image>("/gcs/image", 10);
        }
        else{
            nsocs[i].target_pos_sub     = nh.subscribe("/gcs/target_pose_uav" + std::to_string(i),
                                        10, &rnl::DroneSoc::sendGoalPacket, &nsocs[0]);
            nsocs[i].drone_lk_ahead_pub = nh.advertise<geometry_msgs::Pose>( "/uav" + std::to_string(i) + "/sp_pos", 1);
            nsocs[i].drone_pos_sub      = nh.subscribe("/uav" + std::to_string(i) + "/global_pose",
                                        1, &rnl::DroneSoc::posSubCb, &nsocs[i]);
            nsocs[i].drone_image_sub    = nh.subscribe("/uav" + std::to_string(i) + "/image",
                                        1, &rnl::DroneSoc::imageSubCb, &nsocs[i]);
                                        
            std::cerr << i << " Initialized ros params" << std::endl;
            std::cerr <<"/uav" + std::to_string(i) + "/global_pose"  << " Subscriber" << std::endl;
        }   
    }
     
}


void rnl::Planner::initializeSockets (double dist_gcs2building, double _jpeg_quality, bool _imagePublish)
{
    nsocs.clear();
    for (int i = 0 ; i < num_nodes + 1; ++i)
    {
        rnl::DroneSoc  _dsoc;
        _dsoc.id       = i; 
        _dsoc.setBcSender (wifi_prop.c.Get(i), wifi_prop.tid_val());
        _dsoc.toggle_bc = 1;
        _dsoc.imagePublish = _imagePublish;
        _dsoc.jpeg_quality = _jpeg_quality;
        
        if(i == 0){
            _dsoc.pos      = ns3::Vector3D(dist_gcs2building, 0.0 , 0.0);
        }
        else {
            std::string filename    = std::string(std::getenv("HOME")) + "/fans_ws/src/pci/config/uav" + std::to_string(i) + ".yaml";
            YAML::Node config       = YAML::LoadFile(filename);
            _dsoc.pos               = ns3::Vector3D(config["initial_pos"][0].as<double>(),
                                                    config["initial_pos"][1].as<double>(),
                                                    config["initial_pos"][2].as<double>());
        }
        rnl::posHold(&_dsoc.wpts,_dsoc.pos);
        _dsoc.lookaheadindex = 0;
        _dsoc.state    = SMOVE;
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
        if (rnl::Planner::siteReached (nsocs[i].pos, nsocs[i].goal, i) && nsocs[i].state != SSITEREACHED){
            ns3::Simulator::Schedule (ns3::MilliSeconds(2500), &rnl::DroneSoc::sendImagePacket, &nsocs[i]); // Wait 2.5 seconds for the drone to stabilize
            // ns3::Simulator::Schedule (ns3::MilliSeconds(4000), &rnl::DroneSoc::sendArrivedPacket, &nsocs[i], 0, MAV_CMD_NAV_WAYPOINT); // 0 is the id of gcs
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
    for (int i = 0; i < nsocs.size(); ++i)
    {
        nsocs[i].setRecv (wifi_prop.c.Get(i), wifi_prop.tid_val());
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
