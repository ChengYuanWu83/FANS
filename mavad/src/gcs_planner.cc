#include "planner_ns3.h"

rnl::GcsSoc::GcsSoc() {}
rnl::GcsSoc::~GcsSoc() {}

void rnl::GcsSoc::receivePacket(ns3::Ptr<ns3::Socket> soc)
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
                        if(this->id > 0){
							// std::cerr << "Received packet not belong to it. Discard" << std::endl; 
							return;
						}

                        mavlink_odometry_t odom;
                        mavlink_msg_odometry_decode(&msg, &odom);
                        
                        geometry_msgs::Pose _pos;
                        _pos.position.x = odom.x;
                        _pos.position.y = odom.y;
                        _pos.position.z = odom.z;
                        _pos.orientation.x = odom.q[1];
                        _pos.orientation.y = odom.q[2];
                        _pos.orientation.z = odom.q[3];
                        _pos.orientation.w = odom.q[0];
                        drone_camera_pub.publish(_pos);

                        std::cerr << "Publish camera pose" << std::endl;

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
                            buffer.batch_packet_received.clear();
                            buffer.batch_packet_received.resize(handshake.packets, false);
                            
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
                            std::cerr << "Not receiving packet" << std::endl; 
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
                                std::cerr << "All image packets in batch "<< static_cast<int>(buffer.batch_idx)<<" received, sending ACK..." << std::endl;
                                
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

void rnl::GcsSoc::sendGoalPacket(const geometry_msgs::PoseStamped::ConstPtr& _pos) {
    rnl::Soc::sendGoalPacket(_pos); 
}

// void rnl::GcsSoc::setRecv (ns3::Ptr<ns3::Node> node, ns3::TypeId tid)
// {
//     this -> recv_sink = ns3::Socket::CreateSocket (node, tid);
//     ns3::InetSocketAddress local1 = ns3::InetSocketAddress (ns3::Ipv4Address::GetAny (), 9);
//     if (this->recv_sink->Bind(local1) == -1) {
//         std::cerr << "Failed to bind socket for node " << this->id << std::endl;
//     } else {
//         std::cerr << "Successfully bound socket " << this->recv_sink <<" for node "<< this->id << std::endl;
//     }
//     try{
//         this -> recv_sink->SetRecvCallback (ns3::MakeCallback (&rnl::GcsSoc::receivePacket, this));
//     }
//     catch (const char* msg){
//         std::cerr << catch (const char* msg)  <<" on " << this->recv_sink <<" for node "<< this->id << std::endl;
//     }
// }