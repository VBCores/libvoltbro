#pragma once

#include <ros/ros.h>

template <typename T>
class ROSHandler {
private:
    ros::Subscriber subscriber;
public:
    ROSHandler(ros::NodeHandle& node, const std::string& topic, size_t queue_size) {
        subscriber = node.subscribe(topic, queue_size, &ROSHandler::callback, this);
        std::cout << "Subscribed to topic <" << topic << ">" << std::endl;
    }
    ROSHandler(ros::NodeHandle& node, const std::string& topic)
        : ROSHandler(node, topic, 1000) {}
    virtual void callback(const typename T::ConstPtr&) = 0;
};
