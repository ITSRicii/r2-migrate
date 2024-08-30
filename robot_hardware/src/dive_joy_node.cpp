
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joy_feedback.hpp"
#include <iostream>
#include <chrono>
#include <stdio.h>
#include "robot_hardware/remote_controller/ds4.h"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class JoyNode : public rclcpp::Node {
private:
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher;
    rclcpp::Subscription<sensor_msgs::msg::JoyFeedback>::SharedPtr subscriptor;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr dive_sub;
    rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr dive_fb_pub;
    std::shared_ptr<Joystick> joy;
    // Joystick joy = Joystick("/dev/input/js0");
    sensor_msgs::msg::Joy joy_msg = sensor_msgs::msg::Joy();

public:
    JoyNode() : Node("dive_joy_node") {
        dive_fb_pub = this->create_publisher<sensor_msgs::msg::JoyFeedback>("dive/joy/set_feedback", 10);
        dive_sub  = this->create_subscription<sensor_msgs::msg::Joy> (
                      "dive/joy", 10, std::bind(&JoyNode::diveCallback, this, std::placeholders::_1));

        publisher = this->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
        subscriptor  = this->create_subscription<sensor_msgs::msg::JoyFeedback> (
                      "joy/feedback", 10, std::bind(&JoyNode::feedbackCallback, this, std::placeholders::_1));
    };

    void diveCallback(const sensor_msgs::msg::Joy &msg) {
        auto temp = sensor_msgs::msg::Joy();

        temp.header = msg.header;

        double ftemp;

        ftemp = msg.axes.at(0)*-1.00;
        // ftemp = ftemp*fabs(ftemp);
        temp.axes.push_back(ftemp);

        ftemp = msg.axes.at(1)*-1;
        // ftemp = ftemp*fabs(ftemp);
        temp.axes.push_back(ftemp);
        
        ftemp = (msg.axes.at(2)-1.00)*-0.5;
        // ftemp = ftemp*fabs(ftemp);
        temp.axes.push_back(ftemp);
        
        ftemp = msg.axes.at(3)*-1;
        // ftemp = ftemp*fabs(ftemp);
        temp.axes.push_back(ftemp);
        
        ftemp = msg.axes.at(4)*-1;
        // ftemp = ftemp*fabs(ftemp);
        temp.axes.push_back(ftemp);
        
        ftemp = (msg.axes.at(5)-1.00)*-0.5;
        // ftemp = ftemp*fabs(ftemp);
        temp.axes.push_back(ftemp);

        int btn = 0;
        btn |= (msg.buttons.at(0) << JS_EX);
        btn |= (msg.buttons.at(1) << JS_CR);
        btn |= (msg.buttons.at(2) << JS_TR);
        btn |= (msg.buttons.at(3) << JS_SQ);
        btn |= (msg.buttons.at(4) << JS_L1);
        btn |= (msg.buttons.at(5) << JS_R1);
        btn |= (msg.buttons.at(6) << JS_L2);
        btn |= (msg.buttons.at(7) << JS_R2);
        btn |= (msg.buttons.at(8) << JS_SHARE);
        btn |= (msg.buttons.at(9) << JS_OPT);
        btn |= (msg.buttons.at(10) << JS_PS);
        btn |= (msg.buttons.at(11) << JS_L3);
        btn |= (msg.buttons.at(12) << JS_R3);

        if(msg.axes.at(6) > 0.5) btn |= (1 << JS_LF);
        else if(msg.axes.at(6) < -0.5) btn |= (1 << JS_RG);

        if(msg.axes.at(7) > 0.5) btn |= (1 << JS_UP);
        else if(msg.axes.at(7) < -0.5) btn |= (1 << JS_DW);

        temp.buttons.push_back(btn);

        publisher->publish(temp);
    }

    void feedbackCallback(const sensor_msgs::msg::JoyFeedback &msg) {
        dive_fb_pub->publish(msg);
    };
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
