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
    int num_nodes = 2;
    cmd.AddValue("num_nodes", "the number of drone", num_nodes);
    cmd.Parse(argc, argv);

    /**
     * Create an object of properties, give phyMode, rss value and number of nodes 
     */
    Properties prop ("DsssRate11Mbps",-80, num_nodes);
    prop.initialize(true, true); /**< Initializing with realtime simulation and with checksum enabled*/
    prop.setWifi (false, true); /**<Set wifi without debug and enable pcap and ascii tracing*/
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
    plan.initializeSockets ();
    plan.startSimul();
    return 0;
}
