#include <dds_publisher.hpp>
#include <dds_subscriber.hpp>
#include "VehicleOdometryListener.h"
#include "CarlaDataTypeSupportImpl.h"
#include <topics.hpp>
#include <atomic>
#include <thread>
#include <cstring>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>

std::atomic<int> close_flag{0};

void signal_handler(int _signal) {
	close_flag.store(1);
	std::cout << ": ran signal_handler()\n";
}

int main(int argc, char *argv[]) {

	char *args[] = {"./node1","-DCPSConfigFile","../rtps.ini"};
	signal(SIGINT,signal_handler);
	dds_node this_node(3,args);

	this_node.create_topic<
		CarlaData::VehicleOdometryTypeSupport_ptr,
		CarlaData::VehicleOdometryTypeSupportImpl
	> (topic_names[1].c_str());
	this_node.create_topic<
		CarlaData::SensorDataTypeSupport_ptr,
		CarlaData::SensorDataTypeSupportImpl
	> (topic_names[3].c_str());
	
	// this topic has both RadarSensor + ImageSensor data in it

	dds_publisher publisher(this_node);
	dds_subscriber subscriber(this_node);

    subscriber.create_reader<CarlaData::VehicleOdometryDataReader>(
		this_node,topic_names[1],(new VehicleOdometryListener));
		
	publisher.create_writer<CarlaData::SensorDataDataWriter>(
		this_node, topic_names[3]);

	std::thread([&](){
		cv::Mat read_image = cv::imread("/home/fev/Pictures/vibe1.png");
		publisher.wait_for_subscriber(topic_names[3]);
		for(size_t i{}; i<100; i++) {

			CarlaData::SensorData sensor_data;

			sensor_data.m_radardata.depth = i;
			sensor_data.m_radardata.velocity = i+2;
			sensor_data.m_radardata.azimuth = i*2;
			sensor_data.m_radardata.altitude = i*3;

			long image_size = read_image.rows * read_image.cols * read_image.elemSize();
			sensor_data.m_imagedata.height = read_image.rows;
			sensor_data.m_imagedata.width = read_image.cols;
			sensor_data.m_imagedata.pixel_size = read_image.elemSize();
			sensor_data.m_imagedata.image_type = read_image.type();
			sensor_data.m_imagedata.raw_data.replace(
				image_size,
				image_size,
				read_image.data,
				false
			);

			publisher.write<CarlaData::SensorData,CarlaData::SensorDataDataWriter>(
				sensor_data, topic_names[3]);
				
			std::this_thread::sleep_for(
				std::chrono::milliseconds(250)
			);
		}
		publisher.wait_for_acknowledgments(topic_names[3]);
	}).detach();
	
	std::thread sub_thread([&](){
		subscriber.wait_for_publisher(topic_names[1]);
	});

	while(close_flag.load() == 0);
	sub_thread.detach();
	// sub_thread.join();
    return 0;
}
