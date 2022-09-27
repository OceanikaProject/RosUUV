#!/usr/bin/env python3
import roslaunch
import rospy


try:
    package = 'drone'
    executable = 'navigation.py'

    node = roslaunch.core.Node(package, executable)

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    process = launch.launch(node)
    while True:
        pass
except KeyboardInterrupt:
    state = True
    print("Keyboard Interrupt", True)

except Exception as e:
    print(e)

finally:
    print("Exiting process")
    exit()

