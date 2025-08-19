#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

template <typename T>
class ROS2Handler {
private:
    typename rclcpp::Subscription<T>::SharedPtr subscriber_;

public:
    ROS2Handler(rclcpp::Node::SharedPtr node, const std::string& topic, size_t queue_size = 100) {
        subscriber_ = node->create_subscription<T>(
            topic,
            queue_size,
            std::bind(&ROS2Handler::callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(node->get_logger(), "Subscribed to topic <%s>", topic.c_str());
    }

    virtual void callback(const std::shared_ptr<const T> &) = 0;
};


template <typename T>
class ROS2ServiceProvider {
private:
    typename rclcpp::Service<T>::SharedPtr service_;

public:
    ROS2ServiceProvider(rclcpp::Node::SharedPtr node, const std::string& service_name) {
        service_ = node->create_service<T>(
            service_name,
            std::bind(&ROS2ServiceProvider::callback, this, std::placeholders::_1, std::placeholders::_2)
        );
        RCLCPP_INFO(node->get_logger(), "Providing service <%s>", service_name.c_str());
    }

    virtual void callback(
        const std::shared_ptr<typename T::Request> request,
        std::shared_ptr<typename T::Response> response
    ) = 0;
};
