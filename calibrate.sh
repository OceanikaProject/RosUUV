. /home/pi/drone_ros/devel/setup.bash
systemctl stop oceanika.service
echo "oceanika.service stop"
sleep 2
rosrun drone calibration.py
echo "resuming oceanica.service"
systemctl restart oceanika.service

