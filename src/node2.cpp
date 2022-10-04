#include <dds_subscriber.hpp>
#include <dds_publisher.hpp>
#include <topics.hpp>
#include <adas_features.hpp>
#include "VehicleOdometryListener.h"
#include "SensorDataListener.hpp"
#include "CarlaDataTypeSupportImpl.h"
#include <thread>

/******* global variables ********/
CarlaData::SensorData g_sensor_data;
std::mutex g_sensor_mutex;
/*********************************/

std::atomic<uint8_t> close_flag{0};
std::mutex g_odometry_mutex;
CarlaData::VehicleOdometry g_odometry;

void signal_handler(int _signal) {
	close_flag.store(1);
	std::cout << ": ran signal_handler()\n";
}

int main(int argc, char *argv[]) {
	signal(SIGINT,signal_handler);
	dds_node this_node(argc,argv);

	this_node.create_topic<
		CarlaData::VehicleOdometryTypeSupport_ptr,
		CarlaData::VehicleOdometryTypeSupportImpl
	> (topic_names[1].c_str());

	this_node.create_topic<
		CarlaData::SensorDataTypeSupport_ptr,
		CarlaData::SensorDataTypeSupportImpl
	> (topic_names[3].c_str());

	dds_subscriber subscriber(this_node);
	dds_publisher publisher(this_node);

	SensorDataListener *sensor_listener = new SensorDataListener();
	
	publisher.create_writer<CarlaData::VehicleOdometryDataWriter>(
		this_node, topic_names[1]);
	subscriber.create_reader<CarlaData::SensorDataDataReader>(
		this_node,topic_names[3],(sensor_listener));

	std::thread main_sub_thread([&](){
		subscriber.wait_for_publisher(topic_names[3]);
		// std::cout << "main sub subscriber got it's publisher\n";
	});

	/**
	 * making this thread in detached state might cause
	 * itself to be pre-empted too much due to low priority
	 * thus never gets a subscriber
	*/

	// this thread will run the acc_algo in adas_features 
	std::thread([&](){
		while(close_flag.load() == 0) {
			CarlaData::VehicleOdometry local_odometry;
			{
				// std::lock_guard<std::mutex> lock(g_sensor_mutex);
				local_odometry = adas_features::run_acc_algo(g_sensor_data.m_radardata);
				// adas_features::run_object_detection_algo({});
			}
			std::cout << "g_odometry.throttle:" << local_odometry.throttle
				<< " g_odometry.brake: " << local_odometry.brake
				<< " g_odometry.steer: " << local_odometry.steering
				<< "\n";
			std::cout << "radar.depth:" << g_sensor_data.m_radardata.depth
				<< " radar.alt: " << g_sensor_data.m_radardata.altitude
				<< " radar.azim: " << g_sensor_data.m_radardata.azimuth
				<< "\n";
			{
				std::lock_guard<std::mutex> lock(g_odometry_mutex);
				g_odometry = local_odometry;
				// std::cout << "g_odometry.throttle:" << g_odometry.throttle
				// 	<< "g_odometry.brake: " << g_odometry.brake
				// 	<< "g_odometry.steer: " << g_odometry.steering
				// 	<< "\n";
			}
			std::this_thread::sleep_for(
				std::chrono::milliseconds(250)
			);
		}
	}).detach();

	std::thread([&](){
		while(close_flag.load() == 0) {
			{
				std::lock_guard<std::mutex> lock(g_odometry_mutex);
				publisher.write<
					CarlaData::VehicleOdometry,
					CarlaData::VehicleOdometryDataWriter
				>(g_odometry,topic_names[1]);
				std::this_thread::sleep_for(
					std::chrono::microseconds(500)
				);
			}
		}
	}).detach();

	while(close_flag.load() == 0);
	main_sub_thread.detach();
	// sub_thread.join();
    return 0;
}
