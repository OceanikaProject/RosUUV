#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"


class Control
{
    public:
        Control(int argc, char **argv)
        {
            ros::init(argc, argv, "engines");
            pub = n.advertise<std_msgs::Int32MultiArray>("engines_topic", 1);
            sub = n.subscribe("joystick_control", 10, &Control::joystick_control_callback, this);
        }
        void run()
        {
            ros::spin();
        }
    private:
        std_msgs::Int32MultiArray thrust;
        ros::NodeHandle n;
        ros::Publisher pub;
        ros::Subscriber sub;
        static long map(long x, long in_min, long in_max, long out_min, long out_max)
        {
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }
        void joystick_control_callback(const std_msgs::Int32MultiArray& msg)
        {
            thrust.data.clear();
            int lx = map(msg.data[0], 0, 1023, -100, 100);
            int ly = map(msg.data[1], 0, 1023, -100, 100);
            int rx = map(msg.data[2], 0, 1023, -100, 100);
            int ry = map(msg.data[3], 0, 1023, -100, 100);
            thrust.data.push_back(ly/2 + lx/2);
            thrust.data.push_back(ly/2 + lx/2);
            thrust.data.push_back(ry + rx);
            thrust.data.push_back(ry - rx);
            thrust.data.push_back(ly - lx);

            // lx, ly, rx, ry
            // lv, rv, lh, rh, bv
            // [ly/2 + lx/2, ly/2 + lx/2, ry + rx, ry - rx, ly - lx]
        }
};

int main(int argc, char **argv)
{

    Control ctrl(argc, argv);
    ctrl.run();

    return 0;
}