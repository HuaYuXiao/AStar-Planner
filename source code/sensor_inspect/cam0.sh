## 
gnome-terminal --window -e 'bash -c "roslaunch prometheus_detection web_cam0.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; rqt_image_view; exec bash"' \

