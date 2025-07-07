//------------------------------------------------------------------------------
//Copyright 2025 University College London. Partly derived from main.cpp expample file
//courtesy g.tec Copyright (C) 2014-2016 g.tec medical engineering GmbH.

#include <iostream>
#include <cstdio>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "eeg_msgs/msg/eeg_block.hpp"
#include <gAPI.h>
#include <sense_eeg.h>

void publish_data(void * eeg_publisher)
{
  auto msg = eeg_msgs::msg::EEGBlock();
  GtecEEGPublisher * publisher = static_cast<GtecEEGPublisher *>(eeg_publisher);
  msg.header.stamp = publisher->now();
  msg.num_channels = publisher->num_channels;
  msg.sampling_rate = publisher->sampling_rate;

  // check the number of valid bytes in the device
  int cnt_master = GT_GetSamplesAvailable(publisher->serial_num.c_str() );
  int uchar_to_float = sizeof(float) / sizeof(unsigned char);
  if (cnt_master % uchar_to_float != 0) {
    RCLCPP_ERROR(
      publisher->get_logger(),
      "Bytes available not compatable with float32 data type (%d not divisible by %d), aborting acquisition.",
      cnt_master, uchar_to_float);
    return;
  }

  int total_samples = cnt_master / uchar_to_float;

  // use num_samples to store the number of scans per channel
  msg.num_samples = (total_samples) / publisher->num_channels;

  msg.data.resize(total_samples);

  // I've used new and delete here because I struggled to get the call back working with anything other than
  // a plain unsigned char* buffer. I've put the acquisition code in a try ... catch to try and ensure we
  // always call delete.
  unsigned char * buffer = new unsigned char [cnt_master];
  try {
    int sample_count = GT_GetData(publisher->serial_num.c_str(), buffer, cnt_master);
    if (sample_count != cnt_master) {
      RCLCPP_WARN(
        publisher->get_logger(), "Incomplete data acquistion, expected %d samples, got %d samples.",
        cnt_master, sample_count);
    }

    // copy the buffer into the message body, casting uchar to float
    std::memcpy(&(msg.data[0]), reinterpret_cast<float *>(buffer), cnt_master);

    publisher->publisher->publish(msg);
    RCLCPP_DEBUG(publisher->get_logger(), "Published EEGBlock with %ld samples", msg.data.size());
  } catch (...) {
    RCLCPP_ERROR(
      publisher->get_logger(),
      "Exception thrown during acquisition block");
    delete buffer;
    return;
  }
  delete buffer;
}

GtecEEGPublisher::GtecEEGPublisher()
: Node("gtec_eeg_publisher"),

  num_channels(declare_parameter<int>("num_channels", 1)),
  sampling_rate(declare_parameter<int>("sampling_rate", 256)),
  serial_num(declare_parameter<std::string>("serial_num", "UR-2017.06.12")),

  number_of_scans(declare_parameter<int>("number_of_scans", GT_NOS_AUTOSET)),
  ao_frequency(declare_parameter<int>("ao_frequency", 10)),
  ao_amplitude(declare_parameter<int>("ao_amplitude", 200)),
  ao_offset(declare_parameter<int>("ao_offset", 0)),
  enable_trigger_line(declare_parameter<bool>("enable_trigger_line", false)),
  scan_dio(declare_parameter<bool>("scan_dio", false)),
  slave_mode(declare_parameter<bool>("slave_mode", false)),
  enable_sc(declare_parameter<bool>("enable_sc", false))
{
  GT_ShowDebugInformation(GT_TRUE);

  const int sample_rate = sampling_rate;
  gt_usbamp_analog_out_config ao_config_master;
  ao_config_master.shape = GT_ANALOGOUT_SINE;
  ao_config_master.frequency = ao_frequency;
  ao_config_master.amplitude = ao_amplitude;
  ao_config_master.offset = ao_offset;

  gt_usbamp_config config_master;
  config_master.ao_config = &ao_config_master;
  config_master.sample_rate = sample_rate;
  // number_of_scans sets how many scans are included in each message.
  // setting to GT_NOS_AUTOSET trys to automatically set a value that
  // balances packet size against CPU load. Lots of small messages creates
  // high CPU load.
  config_master.number_of_scans = number_of_scans;
  config_master.enable_trigger_line = enable_trigger_line ? GT_TRUE : GT_FALSE;
  config_master.scan_dio = scan_dio ? GT_TRUE : GT_FALSE;
  config_master.slave_mode = slave_mode ? GT_TRUE : GT_FALSE;
  config_master.enable_sc = enable_sc ? GT_TRUE : GT_FALSE;
  config_master.mode = GT_MODE_NORMAL;
  config_master.num_analog_in = num_channels;

  for (unsigned int i = 0; i < GT_USBAMP_NUM_GROUND; i++) {
    config_master.common_ground[i] = GT_TRUE;
    config_master.common_reference[i] = GT_TRUE;
  }

  for (unsigned char i = 0; i < config_master.num_analog_in; i++) {
    config_master.analog_in_channel[i] = i + 1;
    config_master.bandpass[i] = GT_FILTER_NONE;
    config_master.notch[i] = GT_FILTER_NONE;
    config_master.bipolar[i] = GT_BIPOLAR_DERIVATION_NONE;
  }

  char ** device_list = 0;
  size_t list_size = 0;
  GT_UpdateDevices();
  list_size = GT_GetDeviceListSize();
  device_list = GT_GetDeviceList();

  GT_FreeDeviceList(device_list, list_size);

  if (GT_OpenDevice(serial_num.c_str() ) ) {
    std::string log_message = "Opened g.tec device : " + serial_num;
    RCLCPP_INFO(this->get_logger(), log_message.c_str());
  } else {
    std::string log_message = "Could not open device " + serial_num;
    RCLCPP_ERROR(this->get_logger(), log_message.c_str());
    return;
  }
  if (GT_SetConfiguration(serial_num.c_str(), &config_master) ) {
    RCLCPP_INFO(this->get_logger(), "Master:  Applied config master.");
  } else {
    throw std::invalid_argument("Invalid configuration");
  }

  // Set the depth of the publisher message queue. I think a higher number here
  // will make it less likely that messages will be dropped, at the expense
  // of system resources
  int qos_history_depth = 10;
  publisher = this->create_publisher<eeg_msgs::msg::EEGBlock>("/eeg/raw", qos_history_depth);

  RCLCPP_INFO(this->get_logger(), "Config: %d channels", num_channels);
  GT_SetDataReadyCallBack(serial_num.c_str(), &publish_data, (void *)(this));
  RCLCPP_INFO(this->get_logger(), "Starting DAQ ... ");
  GT_StartAcquisition(serial_num.c_str() );
  RCLCPP_INFO(this->get_logger(), "DAQ started ");

}
GtecEEGPublisher::~GtecEEGPublisher()
{
  RCLCPP_INFO(this->get_logger(), "Stopping DAQ ... ");
  GT_StopAcquisition(serial_num.c_str() );
  RCLCPP_INFO(this->get_logger(), "DAQ stopped ");

  GT_CloseDevice(serial_num.c_str() );
}

//------------------------------------------------------------------------------
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GtecEEGPublisher>());
  } catch (const std::invalid_argument &) {
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return 0;

}
