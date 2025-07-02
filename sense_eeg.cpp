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

void publish_data(void* context)
{
    auto msg = eeg_msgs::msg::EEGBlock();
    GtecEEGPublisher* config = static_cast<GtecEEGPublisher*>(context);
    msg.header.stamp = config->now();
    msg.num_channels = config->num_channels;
    msg.num_samples = config->num_samples;
    msg.sampling_rate = config->sampling_rate;

    size_t cnt_master = GT_GetSamplesAvailable( config->serial_num.c_str() );
    msg.data.reserve(cnt_master);

    GT_GetData( config->serial_num.c_str(), usr_buffer_master, cnt_master);
    float* float_buffer = reinterpret_cast<float *>(usr_buffer_master);

    for (size_t i = 0; i < cnt_master / 4; ++i) {
        msg.data.push_back(float_buffer[i]);
    }
    config->publisher->publish(msg);
    RCLCPP_DEBUG(config->get_logger(), "Published EEGBlock with %ld samples", msg.data.size());
}

GtecEEGPublisher::GtecEEGPublisher() : Node("gtec_eeg_publisher")
{
    num_channels = 1;
    num_samples = 2;
    sampling_rate = 256;
    serial_num = "UR-2017.06.12";

    GT_ShowDebugInformation( GT_TRUE );

    const int sample_rate = sampling_rate;
    gt_usbamp_analog_out_config ao_config_master;
    ao_config_master.shape = GT_ANALOGOUT_SINE;
    ao_config_master.frequency = 10;
    ao_config_master.amplitude = 200;
    ao_config_master.offset = 0;

    gt_usbamp_config config_master;
    config_master.ao_config = &ao_config_master;
    config_master.sample_rate = sample_rate;
    // number_of_scans sets how many scans are included in each message.
    // setting to GT_NOS_AUTOSET trys to automatically set a value that
    // balances packet size against CPU load. Lots of small messages creates
    // high CPU load.
    config_master.number_of_scans = GT_NOS_AUTOSET;
    config_master.enable_trigger_line = GT_FALSE;
    config_master.scan_dio = GT_FALSE;
    config_master.slave_mode = GT_FALSE;
    config_master.enable_sc = GT_FALSE;
    config_master.mode = GT_MODE_NORMAL;
    config_master.num_analog_in = num_channels;

    for ( unsigned int i = 0; i < GT_USBAMP_NUM_GROUND; i++ )
    {
        config_master.common_ground[i] = GT_TRUE;
        config_master.common_reference[i] = GT_TRUE;
    }

    for ( unsigned char i = 0; i < config_master.num_analog_in ; i++ )
    {
        config_master.analog_in_channel[i] = i + 1;
        config_master.bandpass[ i ] = GT_FILTER_NONE;
        config_master.notch[ i ] =  GT_FILTER_NONE;
        config_master.bipolar[ i ] = GT_BIPOLAR_DERIVATION_NONE;
    }

    char** device_list = 0;
    size_t list_size = 0;
    GT_UpdateDevices();
    list_size = GT_GetDeviceListSize();
    device_list = GT_GetDeviceList();

    GT_FreeDeviceList( device_list, list_size );

    if ( GT_OpenDevice( serial_num.c_str() ) )
    {
        std::string log_message = "Opened g.tec device : " + serial_num;
        RCLCPP_INFO(this->get_logger(), log_message.c_str());
    }
    else
    {
        std::string log_message = "Could not open device " + serial_num;
        RCLCPP_ERROR(this->get_logger(), log_message.c_str());
        return;
    }
    if ( GT_SetConfiguration( serial_num.c_str(), &config_master ) )
    {
        RCLCPP_INFO(this->get_logger(), "Master:  Applied config master.");
    }

    // Second argument (10) below is
    // qos_history_depth The depth of the publisher message queue
    publisher = this->create_publisher<eeg_msgs::msg::EEGBlock>("/eeg/raw", 10);

    RCLCPP_INFO(this->get_logger(), "Config: %d channels and %d samples", num_channels, num_samples);
    GT_SetDataReadyCallBack( serial_num.c_str(), &publish_data, (void*)(this)) ;
    RCLCPP_INFO(this->get_logger(), "Starting DAQ ... ");
    GT_StartAcquisition( serial_num.c_str() );
    RCLCPP_INFO(this->get_logger(), "DAQ started ");

}
GtecEEGPublisher::~GtecEEGPublisher()
{
    RCLCPP_INFO(this->get_logger(), "Stopping DAQ ... ");
    GT_StopAcquisition( serial_num.c_str() );
    RCLCPP_INFO(this->get_logger(), "DAQ stopped ");

    GT_CloseDevice( serial_num.c_str() );
}

//------------------------------------------------------------------------------
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GtecEEGPublisher>());
    rclcpp::shutdown();
    return 0;
}
