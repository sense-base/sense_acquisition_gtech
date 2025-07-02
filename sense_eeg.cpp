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
//std::string slave_0;
FILE* data_file_master = 0;
//FILE* data_file_slave_0 = 0;

unsigned char usr_buffer_master[ 32768 ];
rclcpp::Publisher<eeg_msgs::msg::EEGBlock>::SharedPtr publisher_;
std::string serial_num_ = "UR-2017.06.12";

class GtecEEGPublisher : public rclcpp::Node {
public:
    GtecEEGPublisher()
    : Node("gtec_eeg_publisher"),
        num_channels_(declare_parameter<int>("num_channels", 1)),
        num_samples_(declare_parameter<int>("num_samples", 1)),
        sampling_rate_(declare_parameter<double>("sampling_rate", 256.0)),
        serial_num_(declare_parameter<std::string>("serial_num", "UR-2017.06.12")),
        
        number_of_scans_(declare_parameter<int>("number_of_scans", GT_NOS_AUTOSET)),
        ao_frequency_(declare_parameter<int>("ao_frequency", 10)),
        ao_amplitude_(declare_parameter<int>("ao_amplitude", 200)),
        ao_offset_(declare_parameter<int>("ao_offset", 0)),
        enable_trigger_line_(declare_parameter<bool>("enable_trigger_line", false)),
        scan_dio_(declare_parameter<bool>("scan_dio", false)),
        slave_mode_(declare_parameter<bool>("slave_mode", false)),
        enable_sc_(declare_parameter<bool>("enable_sc", false))

    {
        //print out for testing
        std::cout << "Loaded parameters:" << std::endl;
        std::cout << "num_channels: " << num_channels_ << std::endl;
        std::cout << "num_samples: " << num_samples_ << std::endl;
        std::cout << "sampling_rate: " << sampling_rate_ << std::endl;
        std::cout << "serial_num: " << serial_num_ << std::endl;

        std::cout << "number_of_scans: " << number_of_scans_ << std::endl;
        std::cout << "ao_frequency: " << ao_frequency_ << std::endl;
        std::cout << "ao_amplitude: " << ao_amplitude_ << std::endl;
        std::cout << "ao_offset: " << ao_offset_ << std::endl;
        std::cout << "enable_trigger_line: " << enable_trigger_line_ << std::endl;
        std::cout << "scan_dio: " << scan_dio_ << std::endl;
        std::cout << "slave_mode: " << slave_mode_ << std::endl;
        std::cout << "enable_sc: " << enable_sc_ << std::endl;
        
        GT_ShowDebugInformation( GT_TRUE );
        master = serial_num_;
        data_file_master = fopen( "data_master.bin", "wb" );

        const int sample_rate = sampling_rate_;
        gt_usbamp_analog_out_config ao_config_master;
        ao_config_master.shape = GT_ANALOGOUT_SINE;
        ao_config_master.frequency = ao_frequency_;
        ao_config_master.amplitude = ao_amplitude_;
        ao_config_master.offset = ao_offset_;

        gt_usbamp_config config_master;
        config_master.ao_config = &ao_config_master;
        config_master.sample_rate = sample_rate;
        config_master.number_of_scans = number_of_scans_;
        config_master.enable_trigger_line = enable_trigger_line_ ? GT_TRUE : GT_FALSE;
        config_master.scan_dio = scan_dio_ ? GT_TRUE : GT_FALSE;
        config_master.slave_mode = slave_mode_ ? GT_TRUE : GT_FALSE;
        config_master.enable_sc = enable_sc_ ? GT_TRUE : GT_FALSE;
        config_master.mode = GT_MODE_NORMAL;
        config_master.num_analog_in = num_channels_;

        gt_usbamp_asynchron_config asynchron_config_master;
        for ( unsigned int i = 0; i < GT_USBAMP_NUM_GROUND; i++ )
        {
            asynchron_config_master.digital_out[i] = GT_FALSE;
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
            fclose( data_file_master );
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
        fclose( data_file_master );
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
        msg.data.reserve(num_channels_ * num_samples_);
        

        for (int i = 0; i < num_channels_ * num_samples_; ++i) { //placeholder until I work out how to put actual data in
            msg.data.push_back(0.0);
        }
        //int* void2int = (int*)(dummy);
        //(*void2int)++;
        size_t cnt_master = GT_GetSamplesAvailable( master.c_str() );
        std::cout << "called back";
        GT_GetData( master.c_str(), usr_buffer_master, cnt_master);
        publisher_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "Published EEGBlock with %ld samples", msg.data.size());
        std::cout << "Published EEGBlock with " << msg.data.size() << " samples";
    }

    private:
    int num_channels_;
    int num_samples_;
    double sampling_rate_;
    std::string serial_num_;
    int number_of_scans_;
    int ao_frequency_;
    int ao_amplitude_;
    int ao_offset_;
    bool enable_trigger_line_;
    bool scan_dio_;
    bool slave_mode_;
    bool enable_sc_;

   };
//------------------------------------------------------------------------------
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GtecEEGPublisher>());
    rclcpp::shutdown();
    return 0;
}
