```
    mkdir -p {PROJECT}/src
    cd ~{PROJECT}/src
    roscreate_pkg drone roscpp rospy std_msgs geometry_msgs message_generation
    roscreate_pkg drone_description rospy rviz sensor_msgs tf urdf xacro
    cd ~/{PROJECT}
    git pull
```