#!/bin/bash

echo "NOTE: please edit 'NS3' to the path to your NS3 directory"
NS3=$HOME/ns-allinone-3.40/NS3
echo "NS3 path: $NS3"

cleanup() {
    echo "Terminating all programs..."

    if [ ! -z "$px4_pid" ]; then
        echo "Terminate px4 (PID: $px4_pid)"
        kill -TERM $px4_pid 2>/dev/null
    fi
    
    if [ ! -z "$pci_pid" ]; then
        echo "Terminate pci (PID: $pci_pid)"
        kill -TERM $pci_pid 2>/dev/null
    fi
    
    if [ ! -z "$ns_pid" ]; then
        echo "Terminate ns3 (PID: $ns_pid)"
        kill -TERM $ns_pid 2>/dev/null
    fi
    
    echo "All programs have been terminated"
    exit 0
}

trap cleanup EXIT SIGINT SIGTERM

source ./devel/setup.bash

# Start the program and record the PID
echo "start up px4..."
roslaunch px4 sitlSingleDrone.launch > /dev/null 2>&1 &
px4_pid=$!

sleep 10

echo "start up pci..."
roslaunch pci single_drone.launch > /dev/null 2>&1 &
pci_pid=$!

sleep 5

echo "start up ns3..."
cd $NS3
./build/mavad_main &
ns_pid=$!

echo "All programs have been started, press CTRL+C to terminate"

wait