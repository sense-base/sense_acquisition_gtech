//------------------------------------------------------------------------------
//Copyright 2026 University College London. Partly derived from main.cpp expample file
//courtesy g.tec Copyright (C) 2014-2016 g.tec medical engineering GmbH.

#include <iostream>
#include <cstdio>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "eeg_msgs/msg/eeg_block.hpp"
#include <gAPI.h>

unsigned char usr_buffer_master[ 32768 ];

class GtecEEGPublisher : public rclcpp::Node {
public:
    GtecEEGPublisher();
    ~GtecEEGPublisher();
    int num_channels;
    int num_samples;
    float sampling_rate;
    std::string serial_num;
    rclcpp::Publisher<eeg_msgs::msg::EEGBlock>::SharedPtr publisher;
};

