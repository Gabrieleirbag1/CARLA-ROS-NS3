open_terminal() {
    gnome-terminal -- bash -c "$1; exec bash" &
}
# Run ns-ros-bridge Docker container with ns-ros-bridge installed
open_terminal "cd ~/ns3-ros-bridge/catkin_ws/src/rosns3 && sudo ./run_server.sh"

cleanup() {
    cd ~/ns3-ros-bridge/catkin_ws/src/rosns3 && sudo ./stop_server.sh
}

trap cleanup EXIT

while true; do
    read -p "Press enter or CTRL+C to kill ns-ros-bridge container" enter
    exit 0
done
