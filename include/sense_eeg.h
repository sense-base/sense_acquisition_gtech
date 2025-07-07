//------------------------------------------------------------------------------
//Copyright 2025 University College London. Partly derived from main.cpp expample file
//courtesy g.tec Copyright (C) 2014-2016 g.tec medical engineering GmbH.

#ifndef SENSE_EEG_H
#define SENSE_EEG_H

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "eeg_msgs/msg/eeg_block.hpp"

class GtecEEGPublisher: public rclcpp::Node {
public:
  GtecEEGPublisher();
  ~GtecEEGPublisher();
  int num_channels;
  int sampling_rate;
  std::string serial_num;
  rclcpp::Publisher < eeg_msgs::msg::EEGBlock > ::SharedPtr publisher;

private:
  int number_of_scans;
  int ao_frequency;
  int ao_amplitude;
  int ao_offset;
  bool enable_trigger_line;
  bool scan_dio;
  bool slave_mode;
  bool enable_sc;

};

#endif // SENSE_EEG_H
