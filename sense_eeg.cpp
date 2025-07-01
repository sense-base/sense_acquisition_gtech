//------------------------------------------------------------------------------
//Copyright 2026 University College London. Partly derived from main.cpp expample file
//courtesy g.tec Copyright (C) 2014-2016 g.tec medical engineering GmbH.

#include <iostream>
#include <cstdio>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "eeg_msgs/msg/eeg_block.hpp"
#include <gAPI.h>
//------------------------------------------------------------------------------
#define MASTER "UR-2017.06.12"
#define SAMPLERATE 512
#define TIME 5

//------------------------------------------------------------------------------
std::string master;

unsigned char usr_buffer_master[ 32768 ];

rclcpp::Publisher<eeg_msgs::msg::EEGBlock>::SharedPtr publisher_;
int num_channels_ = 1;
int num_samples_ = 2;
float sampling_rate_ = 256;
std::string serial_num_ = "UR-2017.06.12";

class GtecEEGPublisher : public rclcpp::Node {
public:
    GtecEEGPublisher()
    : Node("gtec_eeg_publisher")
    {
        GT_ShowDebugInformation( GT_TRUE );
        master = serial_num_;

        const int sample_rate = sampling_rate_;
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
        config_master.num_analog_in = num_channels_;

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

        if ( GT_OpenDevice( master.c_str() ) )
        {
            std::cout << "Master : " << master << " open" << std::endl;
        }
        else
        {
            std::cout << "Could not open device " << master << std::endl;
            return;
        }
        if ( GT_SetConfiguration( master.c_str(), &config_master ) )
        {
            std::cout << "Master:  Applied config master." << std::endl;
        }

        // Second argument (10) below is
        // qos_history_depth The depth of the publisher message queue
        publisher_ = this->create_publisher<eeg_msgs::msg::EEGBlock>("/eeg/raw", 10);

        int dummy_arg = 0;
        // GT_SetDataReadyCallBack( master.c_str(), &GtecEEGPublisher::publish_data, (void*)(&dummy_arg)) ;
        GT_SetDataReadyCallBack( master.c_str(), GtecEEGPublisher::publish_data_wrapper, (void*)(&dummy_arg)) ;
        std::cout << "Start DAQ ... ";
        GT_StartAcquisition( master.c_str() );
        std::cout << "started" << std::endl;

    }
    ~GtecEEGPublisher()
    {
        std::cout << "Stop DAQ ... ";
        GT_StopAcquisition( master.c_str() );
        std::cout << "stopped"  << std::endl;

        GT_CloseDevice( master.c_str() );
    }

    static void publish_data_wrapper(void* context) {
        static_cast<GtecEEGPublisher*>(context)->publish_data();
    }

    void publish_data()
    {
        auto msg = eeg_msgs::msg::EEGBlock();
        //msg.header.stamp = this->now();
        msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();  // calling this->now does not work (runtime error)
        msg.num_channels = num_channels_;
        msg.num_samples = num_samples_;
        msg.sampling_rate = sampling_rate_;

        size_t cnt_master = GT_GetSamplesAvailable( master.c_str() );
        msg.data.reserve(cnt_master);
        //msg.data.reserve(cnt_master);

        GT_GetData( master.c_str(), usr_buffer_master, cnt_master);
        float* float_buffer = reinterpret_cast<float *>(usr_buffer_master);

        for (size_t i = 0; i < cnt_master / 4; ++i) {
            msg.data.push_back(float_buffer[i]);
        }
        //msg.data.data = usr_buffer_master
        publisher_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "Published EEGBlock with %ld samples", msg.data.size());
    }

   };
//------------------------------------------------------------------------------
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GtecEEGPublisher>());
    rclcpp::shutdown();
    return 0;
}
