#include <ace/Log_Msg.h>
#include <ace/OS_NS_stdlib.h>

#include "SensorDataListener.hpp"
#include "CarlaDataTypeSupportC.h"
#include "CarlaDataTypeSupportImpl.h"
#include "adas_features.hpp"

#include <iostream>

void
SensorDataListener::on_requested_deadline_missed(
  DDS::DataReader_ptr /*reader*/,
  const DDS::RequestedDeadlineMissedStatus& /*status*/)
{
}

void
SensorDataListener::on_requested_incompatible_qos(
  DDS::DataReader_ptr /*reader*/,
  const DDS::RequestedIncompatibleQosStatus& /*status*/)
{
}

void
SensorDataListener::on_sample_rejected(
  DDS::DataReader_ptr /*reader*/,
  const DDS::SampleRejectedStatus& /*status*/)
{
}

void
SensorDataListener::on_liveliness_changed(
  DDS::DataReader_ptr /*reader*/,
  const DDS::LivelinessChangedStatus& /*status*/)
{
}

void
SensorDataListener::on_data_available(DDS::DataReader_ptr reader)
{
  CarlaData::SensorDataDataReader_var sensor_reader =
    CarlaData::SensorDataDataReader::_narrow(reader);
  std::cout << "inside on_data_available()\n";
  if (!sensor_reader) {
    ACE_ERROR((LM_ERROR,
               ACE_TEXT("ERROR: %N:%l: on_data_available() -")
               ACE_TEXT(" _narrow failed!\n")));
    ACE_OS::exit(1);
  }
  CarlaData::SensorData sensor_payload;

  DDS::SampleInfo info;
  const DDS::ReturnCode_t error = sensor_reader->take_next_sample(sensor_payload, info);

  if (error == DDS::RETCODE_OK) {
    std::cout << "SampleInfo.sample_rank = " << info.sample_rank << std::endl;
    std::cout << "SampleInfo.instance_state = " << OpenDDS::DCPS::InstanceState::instance_state_mask_string(info.instance_state) << std::endl;

    if (info.valid_data) {
      std::cout 
        << std::to_string(std::chrono::system_clock::now().time_since_epoch().count())
        << " received CarlaData::SensorData\n";

        {
          std::lock_guard<std::mutex> lock(g_sensor_mutex);
          g_sensor_data = sensor_payload;
        }

        std::cout << "radar ["
          << sensor_payload.m_radardata.depth << ","
          << sensor_payload.m_radardata.velocity << ","
          << sensor_payload.m_radardata.azimuth << ","
          << sensor_payload.m_radardata.altitude
          << "]\n";

        cv::Mat img(
          sensor_payload.m_imagedata.height,
          sensor_payload.m_imagedata.width,
          sensor_payload.m_imagedata.image_type,
          sensor_payload.m_imagedata.raw_data.get_buffer()
        );
        
        if(img.size().area() > 0) {
          cv::imshow("test-window",img);
          cv::waitKey(1);
        }

    }

  } else {
    ACE_ERROR((LM_ERROR,
               ACE_TEXT("ERROR: %N:%l: on_data_available() -")
               ACE_TEXT(" take_next_sample failed!\n")));
  }
}

void
SensorDataListener::on_subscription_matched(
  DDS::DataReader_ptr reader,
  const DDS::SubscriptionMatchedStatus& status
) {
  std::cout << "called on_subscription_matched()\n";
  std::cout << "total_count           :" << status.total_count << std::endl;
  std::cout << "total_count_change    :" << status.total_count_change << std::endl;
  std::cout << "current_count         :" << status.current_count << std::endl;
  std::cout << "current_count_change  :" << status.current_count_change << std::endl;
  // DDS::InstanceHandle_t last_publication_handle;
}

void
SensorDataListener::on_sample_lost(
  DDS::DataReader_ptr /*reader*/,
  const DDS::SampleLostStatus& /*status*/)
{
}

CarlaData::VehicleOdometry&
SensorDataListener::get_odometry() {
  std::lock_guard<std::mutex> lock_guard(m_odometry_mutex);
  return m_odometry_data;
}