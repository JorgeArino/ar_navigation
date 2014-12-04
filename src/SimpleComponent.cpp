#include <rcomponent/RComponent.h>

using namespace std;

//! Test class that inherits RComponent
class SimpleComponent: public RComponent{
	private:
	//! Name of the topic
	string topic_name_;
	
	//! Publishes the status of the robot
	ros::Publisher status_pub_;
	
	
	// Constructor
	SimpleComponent(double hz, ros::NodeHandle h):RComponent(hz, h){
		
	}
	
	// Inherits from RComponent
	int rosSetup(){
		if(RComponent::rosSetup() == OK){
			pnh.param<string>("topic_name", topic_name_, "status");
			pnh.param("desired_freq", desired_freq, desired_freq);
			
			status_pub_ = pnh.advertise<std_msgs::String>(topic_name_, 1);
		}
	}
	// Inherits from RComponent
	int rosShutdown(){
		if(RComponent::rosShutdown() == OK){
			ROS_INFO("rosShutdown");
		}
	}
	
	//  
	void rosPublish(){
		status_pub_.publish("SimpleComponent");
	}
	
};


// MAIN
int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_component");
	
	ros::NodeHandle n;		
  	SimpleComponent simple(2.0, n);
	
	simple.start();

	return (0);
}
