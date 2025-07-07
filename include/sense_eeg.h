//------------------------------------------------------------------------------
//Copyright 2025 University College London. Partly derived from main.cpp expample file
//courtesy g.tec Copyright (C) 2014-2016 g.tec medical engineering GmbH.

#include "rclcpp/rclcpp.hpp"
#include "eeg_msgs/msg/eeg_block.hpp"

class GtecEEGPublisher: public rclcpp::Node {
public:
  GtecEEGPublisher();
  ~GtecEEGPublisher();
  int num_channels;
  float sampling_rate;
  std::string serial_num;
  rclcpp::Publisher < eeg_msgs::msg::EEGBlock > ::SharedPtr publisher;
};
