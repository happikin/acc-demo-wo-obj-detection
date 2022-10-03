#include <dds_subscriber.hpp>
#include <dds_publisher.hpp>
#include "VehicleOdometryListener.h"
#include "SensorDataListener.hpp"
#include "CarlaDataTypeSupportImpl.h"
#include "adas_features.hpp"
#include <topics.hpp>
#include <thread>

std::atomic<uint8_t> close_flag{0};
std::mutex odometry_mutex;
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
	});

	/**
	 * making this thread in detached state might cause
	 * itself to be pre-empted too much due to low priority
	 * thus never gets a subscriber
	*/

	while(close_flag.load() == 0);
	main_sub_thread.detach();
	// sub_thread.join();
    return 0;
}
