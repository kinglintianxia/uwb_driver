// king@2018.05.21
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

class UWBDriver
{
private:
  bool init();

  void receive_msg();
  void handle_dist_msg(const std::string &line);

  void check(uint8_t* data, size_t len, uint8_t& dest);
  std::string print_hex(uint8_t* data, int length);
public:
  UWBDriver();
  ~UWBDriver();

  void run();
private:
  bool parse_flag_;

  boost::system::error_code ec_;
  boost::asio::io_service io_service_;
  serial_port_ptr port_;
  boost::mutex mutex_;

  tf2_ros::TransformBroadcaster br_;
  geometry_msgs::TransformStamped transformStamped_;

  std::string port_name_;
  int baud_rate_;

  std::string base_frame_;

  bool start_flag_;
  int sensor_rate_;
  ros::Time last_time_, now_;
  // Distances
  // std::vector<double> dist_vec;
  std::vector<double> dist_vec;
  std::vector<double> cal_dist;	// the last calculate distance (x,y)
};

UWBDriver::UWBDriver():start_flag_(true){}

UWBDriver::~UWBDriver(){
  boost::mutex::scoped_lock look(mutex_);
  parse_flag_ = false;
  if (port_) {
    port_->cancel();
    port_->close();
    port_.reset();
  }
  io_service_.stop();
  io_service_.reset();
}

bool UWBDriver::init(){
  std::printf("%s\n", "Initialing...");
  if (port_) {
    ROS_ERROR("error : port is already opened...");
    return false;
  }
  port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
  port_->open(port_name_, ec_);
  if (ec_) {
    ROS_INFO_STREAM("error : port_->open() failed...port_name=" << port_name_ << ", e=" << ec_.message().c_str());
    return false;
  }
  // option settings...
  port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  port_->set_option(boost::asio::serial_port_base::character_size(8));
  port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
  std::cout << "Serial_port settings:\n" << "Port name: " << port_name_ << std::endl
  			<< "Baud_rate: " << baud_rate_ << std::endl
  			<< "Character_size: " << 8 << std::endl
  			<< "Stop_bits: " << 1 << std::endl
  			<< "Parity: " << "none." << std::endl;

  std::printf("%s\n", "Initialize Done!");
  ros::Rate loop_rate(sensor_rate_);
  return true;
}


/////////////////
/// 消息处理函数
/////////////////
void UWBDriver::handle_dist_msg(const std::string &line){
	std::string str(line);
	int anchor_id = 0;
	static int max_anchor_id = 0;
	double dist = 0.0;
	std::string tmp_dist;
	// Max anchor num '6'
	// std::vector<double> dist_vec(6, 0.0);
	dist_vec.resize(6);
	// for (int i=0; i< dist_vec.size(); i++)
	// 	dist_vec[i] = 0.0;
	cal_dist.resize(2);	// (x,y)
	// Only process "distance..." lines
	if (str.find("distance") != std::string::npos)
	{
		// get anchor
		anchor_id = str[9]-'0';		// char to int
		max_anchor_id = anchor_id>max_anchor_id ? anchor_id: max_anchor_id;
		// get dist
		for (int i=11; i<18; i++)
		{
			tmp_dist += str[i];
		}
		// str to float && 'mm' to 'm'
	  	dist = std::atof(tmp_dist.c_str())/1000.0;	
	  	
	  	// push back dist
	  	dist_vec[anchor_id-1] = dist;
	  	// for test
	  	// std::cout << "str[9]: " << str[9] << ", anchor_id: " << anchor_id << std::endl
	  	// 		  << "tmp_dist:" << tmp_dist << ", dist: " << dist << std::endl;
	  	std::cout << "dist_vec.size: " << dist_vec.size() << std::endl;
	  	for (int i = 0; i < dist_vec.size(); i ++)
	  		std::cout << "dist_vec[" << i << "]: " << dist_vec[i] << std::endl;
	  	
	}
	// Send tf
	if (anchor_id == max_anchor_id)
	{
		/*****************TO DO***********************************/
	  	// cal the real dist
	  	cal_dist[0] = dist_vec[0]; 	// x
		cal_dist[1] = 0.0; 	// y
		
		// Send tf
		transformStamped_.header.stamp = ros::Time::now();
			// std::string frame_id = "anchor_" + std::to_string(0+1);
	    transformStamped_.header.frame_id = base_frame_;
	    transformStamped_.child_frame_id = "tag_0";
	    transformStamped_.transform.translation.x = cal_dist[0];
	    transformStamped_.transform.translation.y = cal_dist[1];
	    transformStamped_.transform.translation.z = 0.0;
	    tf2::Quaternion q;
	    q.setRPY(0, 0, 0);
	    transformStamped_.transform.rotation.x = q.x();
	    transformStamped_.transform.rotation.y = q.y();
	    transformStamped_.transform.rotation.z = q.z();
	    transformStamped_.transform.rotation.w = q.w();

	    br_.sendTransform(transformStamped_);
		// for reuse
		dist_vec.clear();
	}
	

}

void UWBDriver::receive_msg(){
  // uint8_t buffer_data[255];
  parse_flag_ = true;
  while (parse_flag_){
    
      // boost::asio::read(*port_.get(), boost::asio::buffer(buffer_data), boost::asio::transfer_all(), ec_);
      // // std::size_t s1 = boost::asio::buffer_size(buffer_data);
      // // std::cout << "buffer_data: \n" << vec_rec << std::endl;
      // std::printf("%s", buffer_data);
      
      // Read lines
      boost::asio::streambuf pic_info;
	  std::size_t n = boost::asio::read_until(*port_.get(), pic_info, '\n', ec_);
	  boost::asio::streambuf::const_buffers_type bufs = pic_info.data(); 
	  std::string line(boost::asio::buffers_begin(bufs), boost::asio::buffers_begin(bufs) + n); 
	  // std::ostream ss(&pic_info);
	  // ss > str_rec;
	  std::cout << line;
	  handle_dist_msg(line);
  }
}

/////////////////
/// 工具函数
/////////////////

std::string UWBDriver::print_hex(uint8_t* data, int length){
  std::string output ="";
  static char hex[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
  for (int i=0; i<length; i++){
    output += hex[(*(data+i) >> 4) & 0xf] + hex[*(data+i) & 0xf] + " ";
  }
  return output;
}

void UWBDriver::check(uint8_t* data, size_t len, uint8_t& dest){
  dest = 0x00;
  for (int i=0; i<len; i++){
    dest = dest ^ *(data + i);
  }
}

/////////////////
/// 主执行函数
/////////////////

void UWBDriver::run(){
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  private_node.param<std::string>("port_name", port_name_, std::string("/dev/ttyUSB0"));
  private_node.param<std::string>("base_frame", base_frame_, std::string("base_link"));

  private_node.param<int>("baud_rate", baud_rate_, 115200);
  private_node.param<int>("sensor_rate", sensor_rate_, 10);

  if (init()){
   	// ros::Timer handle_dist_timer = node.createTimer(ros::Duration(100.0/sensor_rate_), &UWBDriver::handle_dist_msg, this);
    boost::thread parse_thread(boost::bind(&UWBDriver::receive_msg, this));
    ros::spin();
    return;
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "uwb_driver");
  UWBDriver driver;
  driver.run();
  return 0;
}