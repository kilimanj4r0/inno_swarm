cd /home/sm1rk/drone-games
./stop.sh
sleep 1
./formation.sh nonprof 6 &
sleep 5
cd /home/sm1rk/catkin_ws/src/inno_swarm/src
python3 standard_func.py
