/**
 * @brief Executable file for simulation, contains the main function
 */

#include "planner_ns3.h"
#include "planner_ns3_utils.h"
#include "planner_config.h"

using namespace rnl;
using namespace ns3;

/**
 * Initializing static variables to a fixed value
 */
ns3::Vector3D rnl::Planner::disas_centre = ns3::Vector3D (10,10,3);

NS_LOG_COMPONENT_DEFINE ("Mavad_main");

int main(int argc, char **argv){

    ros::init(argc, argv, "ros_ns3_planner");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ns3::CommandLine cmd(__FILE__);
    int num_nodes = 1;
    int jpeg_quality = 75;
    double dist_gcs2building = 25.0;
    bool band_5GHz_enable = false;
    bool save_image_enable = false;
    std::string scene_type = "metal";

    cmd.AddValue("num", "The number of drone", num_nodes);
    cmd.AddValue("quality", "Desired image quality for jpeg compression [1-100]", jpeg_quality);
    cmd.AddValue("dist", "Distance of GCS from the center of the building (meters)", dist_gcs2building);
    cmd.AddValue("band_5G", "Using 2.4GHz frequency band on WiFi", band_5GHz_enable);
    cmd.AddValue("save_image", "Image publish in ROS or store in PNG", save_image_enable);
    cmd.AddValue("scene_type", "Scene type: wood or metal", scene_type);
    
    cmd.Parse(argc, argv);


    std::cout << "======= Simulation Parameters ======= " << std::endl;
    std::cout << "  Number of drones           : " << num_nodes << std::endl;
    std::cout << "  JPEG quality               : " << jpeg_quality << std::endl;
    std::cout << "  GCS-to-building distance   : " << dist_gcs2building << " meters" << std::endl;
    std::cout << "  5GHz band enabled          : " << std::boolalpha << band_5GHz_enable << std::endl;
    std::cout << "  Store image in PNG enabled : " << std::boolalpha << save_image_enable << std::endl;
    std::cout << "===================================== " << std::endl;


    /**
     * Create an object of properties, give phyMode, rss value and number of nodes 
     */
    Properties prop ("DsssRate11Mbps",-80, num_nodes);
    prop.initialize(true, true); /**< Initializing with realtime simulation and with checksum enabled*/
    prop.setWifi (false, true, band_5GHz_enable); /**<Set wifi without debug and enable pcap and ascii tracing*/
    prop.setInternet (); /**< Set IP*/

    /**
     * Create and start a Planner: ros::NodeHandle& _nh, 
     *                             ros::NodeHandle& _nh_private, 
     *                             rnl::Properties& p, <network propoertity>
                                   int n,              <number of nodes>
                                   float  _pki,        <packet interval>    original: 0.2
                                   float _pos_int,     <pos_interval>       original: 0.1
                                   float _stopTime     <simulation stop time> original: 2500.0
     */
    rnl::Planner plan (nh, nh_private, prop, num_nodes, 0.2, 0.1, 2500.0);  
    plan.initializeSockets (dist_gcs2building, jpeg_quality, !save_image_enable);
    plan.startSimul(scene_type);
    return 0;
}
