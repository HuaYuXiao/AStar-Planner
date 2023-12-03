## 
gnome-terminal --window -e 'bash -c "roslaunch bluesea2 amov-50C-3.launch;
 exec bash"' \
--tab -e 'bash -c "sleep 5; rostopic echo /prometheus/sensors/2Dlidar_scan;
 exec bash"' \
