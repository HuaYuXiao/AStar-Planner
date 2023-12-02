## 
gnome-terminal --window -e 'bash -c "roslaunch realsense2_camera rs_d400_and_t265.launch; exec bash"' \
--tab -e 'bash -c "sleep 18; rqt_image_view; exec bash"' \

