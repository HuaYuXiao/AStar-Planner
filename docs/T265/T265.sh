## 
gnome-terminal --window -e 'bash -c "roslaunch p450_experiment rs_t265.launch;
 exec bash"' \
--tab -e 'bash -c "sleep 5; rostopic echo /t265/odom/sample; 
exec bash"' \
