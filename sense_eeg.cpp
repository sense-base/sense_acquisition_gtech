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

class GtecEEGPublisher: public rclcpp::None {
public:
    GtecEEGPublisher()
    : Node("gtec_eeg_publisher"),
      num_channels_ (16),
      num_samples_(32),
      sampling_rate_(256),
      serial_num_("UR-2017.06.12")

    {
        GT_ShowDebugInformation( GT_TRUE );
        master = serial_num_;
        data_file_master = fopen( "data_master.bin", "wb" );

        const int sample_rate = sampling_rate_;
        gt_usbamp_analog_out_config ao_config_master;
        ao_config_master.shape = GT_ANALOGOUT_SINE;
        ao_config_master.frequency = 10;
        ao_config_master.amplitude = 200;
        ao_config_master.offset = 0;

        gt_usbamp_config config_master;
        config_master.ao_config = &ao_config_master;
        config_master.sample_rate = sample_rate;
        config_master.number_of_scans = GT_NOS_AUTOSET;
        config_master.enable_trigger_line = GT_FALSE;
        config_master.scan_dio = GT_FALSE;
        config_master.slave_mode = GT_FALSE;
        config_master.enable_sc = GT_FALSE;
        config_master.mode = GT_MODE_NORMAL;
        config_master.num_analog_in = _num_channels;

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
            return -1;
        }
        if ( GT_SetConfiguration( master.c_str(), &config_master ) )
        {
            std::cout << "Master:  Applied config master." << std::endl;
        }

        // Second argument (10) below is
        // qos_history_depth The depth of the publisher message queue
        publisher_ = this->create_publisher<eeg_msgs::msg::EEGBlock>("/eeg/raw", 10);

        int dummy_arg = 0;
        GT_SetDataReadyCallBack( master.c_str(), this->publish_data, (void*)(&dummy_arg) );
        std::cout << "Start DAQ ... ";
        GT_StartAcquisition( master.c_str() );
        std::cout << "started" << std::endl;

    }

private:
    void publish_data(void* dummy)
    {
        int* void2int = (int*)(dummy);
        (*void2int)++;
        size_t cnt_master = GT_GetSamplesAvailable( master.c_str() );
        std::cout << "called back";
        if ( GT_GetData( master.c_str(), usr_buffer_master, cnt_master) )
            fwrite( usr_buffer_master, 1, cnt_master, data_file_master );
    }

    rclcpp::Publisher<eeg_msgs::msg::EEGBlock>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int num_channels_;
    int num_samples_;
    float sampling_rate_;
};

//------------------------------------------------------------------------------
unsigned char usr_buffer_master[ 32768 ];
void CallBackMaster( void* dummy )
{
    int* void2int = (int*)(dummy);
    (*void2int)++;
    size_t cnt_master = GT_GetSamplesAvailable( master.c_str() );
    std::cout << "called back";
    if ( GT_GetData( master.c_str(), usr_buffer_master, cnt_master) )
      fwrite( usr_buffer_master, 1, cnt_master, data_file_master );
}

//------------------------------------------------------------------------------
int main()
{
  GT_ShowDebugInformation( GT_TRUE );
  master = MASTER;
  data_file_master = fopen( "data_master.bin", "wb" );

  const int sample_rate = SAMPLERATE;
  gt_usbamp_analog_out_config ao_config_master;
  ao_config_master.shape = GT_ANALOGOUT_SINE;
  ao_config_master.frequency = 10;
  ao_config_master.amplitude = 200;
  ao_config_master.offset = 0;

  gt_usbamp_config config_master;
  config_master.ao_config = &ao_config_master;
  config_master.sample_rate = sample_rate;
  config_master.number_of_scans = GT_NOS_AUTOSET;
  config_master.enable_trigger_line = GT_FALSE;
  config_master.scan_dio = GT_FALSE;
  config_master.slave_mode = GT_FALSE;
  config_master.enable_sc = GT_FALSE;
  config_master.mode = GT_MODE_NORMAL;
  config_master.num_analog_in = 16;

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
    return -1;
  }
  if ( GT_SetConfiguration( master.c_str(), &config_master ) )
  {
    std::cout << "Master:  Applied config master." << std::endl;
  }

  int dummy_arg = 0;
  GT_SetDataReadyCallBack( master.c_str(), &CallBackMaster, (void*)(&dummy_arg) );
  std::cout << "Start DAQ ... ";
  GT_StartAcquisition( master.c_str() );
  std::cout << "started" << std::endl;


  sleep( TIME );
  std::cout << "Stop DAQ ... ";
  GT_StopAcquisition( master.c_str() );
  std::cout << "stopped"  << std::endl;

  GT_CloseDevice( master.c_str() );
  fclose( data_file_master );
}
