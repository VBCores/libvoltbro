#pragma once

#include <ros/ros.h>

template <typename T>
class ROSHandler {
private:
    ros::Subscriber subscriber;
public:
    ROSHandler(ros::NodeHandle& node, const std::string& topic, size_t queue_size) {
        subscriber = node.subscribe(topic, queue_size, &ROSHandler::callback, this);
        ROS_INFO_STREAM("Subscribed to topic <" << topic << ">");
    }
    ROSHandler(ros::NodeHandle& node, const std::string& topic)
        : ROSHandler(node, topic, 100) {}
    virtual void callback(const typename T::ConstPtr&) = 0;
};

template <typename T>
class ServiceProvider {
private:
    ros::ServiceServer service;
public:
    virtual bool callback(typename T::Request&, typename T::Response&) = 0;

    ServiceProvider(std::shared_ptr<ros::NodeHandle>& node, const std::string& service_name) {
        service = node->advertiseService(service_name, &ServiceProvider::callback, this);
        ROS_INFO_STREAM("Providing service <" << service_name << ">");
    }
};
