#include "planner_ns3.h"

rnl::DroneSoc::DroneSoc() {}
rnl::DroneSoc::~DroneSoc() {}


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
                        if(ack.command == MAV_CMD_IMAGE_START_CAPTURE && ack.result == MAV_RESULT_ACCEPTED){
                            ss << " receivied batch";
                            // Move to next batch if we're the image sender
                            
                            if(this->batch_in_progress) {
                                this->current_batch_idx++;
                                this->batch_in_progress = false;
                                // If there are more batches to send, schedule the next one
                                if(!this->image_batch_queue.empty()) {
                                    ns3::Simulator::ScheduleNow(&rnl::DroneSoc::sendNextImageBatch, this);
                                }
                                else {
                                    this->current_batch_idx = 0;

                                    std::cerr << "All image batches sent successfully" << std::endl;
                                    ns3::Simulator::ScheduleNow(&rnl::DroneSoc::sendArrivedPacket, this, 0, MAV_CMD_NAV_WAYPOINT); // 0 is the id of gcs
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
              << this->image_batch_queue.size() + this->current_batch_idx + 1 << " (" << chunk_count << " chunks)" << std::endl;
    
    // Schedule chunk sending for this batch
    ns3::Simulator::Schedule(ns3::MilliSeconds(5), &rnl::DroneSoc::sendImageChunk, this, 0, chunk_count, batch_buffer_);
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
    this->sendOdomPacket(this->cameraPose);

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

}


void rnl::DroneSoc::camPosSubCb (const geometry_msgs::Pose::ConstPtr& _pos)
{
    this->cameraPose = *_pos;
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
    // std::cerr<<"publishLookAhead" << std::endl;
    _lka.position.x = this->wpts[this->lookaheadindex].x;
    _lka.position.y = this->wpts[this->lookaheadindex].y;
    _lka.position.z = this->wpts[this->lookaheadindex].z;

    drone_lk_ahead_pub.publish (_lka);
}

// void rnl::DroneSoc::setRecv (ns3::Ptr<ns3::Node> node, ns3::TypeId tid)
// {
//     this -> recv_sink = ns3::Socket::CreateSocket (node, tid);
//     ns3::InetSocketAddress local1 = ns3::InetSocketAddress (ns3::Ipv4Address::GetAny (), 9);
//     if (this->recv_sink->Bind(local1) == -1) {
//         std::cerr << "Failed to bind socket for node " << this->id << std::endl;
//     } else {
//         std::cerr << "Successfully bound socket " << this->recv_sink <<" for node "<< this->id << std::endl;
//     }
//     try{
//         this -> recv_sink->SetRecvCallback (ns3::MakeCallback (&rnl::DroneSoc::receivePacket, this));
//     }
//     catch (const char* msg){
//         std::cerr << catch (const char* msg)  <<" on " << this->recv_sink <<" for node "<< this->id << std::endl;
//     }
// }