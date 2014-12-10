#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <string.h>


#include <ar_navigation/save_qr.h>
#include <boost/lexical_cast.hpp>

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/emitter.h>

#include <iostream>
#include <fstream>

#define PI 3.1415

using namespace std;

typedef struct {
   std::string frame_id;
   geometry_msgs::Pose pose;
} qr_position;

/*!
 * /brief Class to define the qr_saver component
 */
class QRLocalization{
	public:
		
		// Variables
		ros::NodeHandle nh_;
		ros::Subscriber ar_marker_sub_;
		ros::ServiceServer save_qr_srv;
		ros::Publisher ar_pose_pub_;
		
		float desired_freq_;
		tf::TransformListener* tf_listener_;	
		tf::TransformBroadcaster* tf_broadcaster_;	 		
			
		
		geometry_msgs::Pose amcl_pose;
		geometry_msgs::PoseWithCovarianceStamped qr_pose_msg_;
		int qr_pose_msg_seq_;
		
		// Waypoint array (not limited)
		std::vector<qr_position> qr_positions_vector_;
		
		// To publish the map_odom tf 
		bool publish_tf;
		
		
			
		// Methods
		QRLocalization()
		{
			ROS_INFO("SETUP");
			
			desired_freq_ = 1.0;
			
			// new tf listener and broadcaster
			tf_listener_ = new tf::TransformListener();
			tf_broadcaster_ = new tf::TransformBroadcaster();

			// Publishers			
			ar_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("ar_pose",1);
			
			// Subscribers
			ar_marker_sub_ = nh_.subscribe<ar_track_alvar::AlvarMarkers>("/ar_pose_marker", 1, &QRLocalization::ar_pose_callback, this);
			
			// Services
			save_qr_srv = nh_.advertiseService("/save_qr", &QRLocalization::save_qr, this);

			qr_pose_msg_seq_ = 0;
			publish_tf = true;
		
			//YAML::Emitter out;
			//out << YAML::BeginSeq;
			//out << "eggs";
			//out << "bread";
			//out << "milk";
			//out << YAML::EndSeq;
			//std::cout << "Here's the output YAML:\n" << out.c_str(); 


			//qr_positions_file = new ofstream("qr_positions.yaml", std::ofstream::out);
		  //qr_positions_file->open ("qr_positions.yaml");
		  
		  ////qr_positions_file << out.c_str();
		  //qr_positions_file->close();

			ROS_INFO("Waiting");
			ros::Duration(5.0).sleep();
			
			ROS_INFO("Setup finished");
		};
		
		
		void start()
		{
			ros::Rate r(desired_freq_);
			while(ros::ok())
			{
				ros::spinOnce();
				
				// NODE MAIN LOOP
				//ROS_INFO("Main loop");
	
				r.sleep();
			}
			
			//qr_positions_file->close();
			ros::shutdown();
		};
		
		
		bool save_qr(ar_navigation::save_qr::Request &req, ar_navigation::save_qr::Response &res )
		{
			ROS_INFO("Call to save_qr service");

			int qr_code = req.qr_code;
			
			string qr_frame("ar_marker_");
			string str_qr_code = boost::lexical_cast<string>( qr_code );
			qr_frame.append(str_qr_code);
			
			string base_frame("camera_link");
			string map_frame("map");
			
			ROS_INFO("Request transform from frame: %s, to: %s", map_frame.c_str(), qr_frame.c_str());
			
			tf::StampedTransform qr_to_map;
	    try{
	      tf_listener_->lookupTransform(map_frame, qr_frame,  
	                               ros::Time(0), qr_to_map);
	    }
	    catch (tf::TransformException ex){
	      ROS_ERROR("%s",ex.what());
	      ros::Duration(1.0).sleep();
	    }
			ROS_INFO("QR transform requested x: %f, y: %f, z: %f", qr_to_map.getOrigin().x(), qr_to_map.getOrigin().y(), qr_to_map.getOrigin().z() );
			
			//add_qr_position_to_param(qr_code, qr_to_map);
			
			return true;
		}
		
		
		/*
		 *	\brief Gets the frame of the selected qr code
		 */
		string get_qr_frame(int qr_code)
		{
			string qr_frame("ar_marker_");
			string str_qr_code = boost::lexical_cast<string>( qr_code );
			qr_frame.append(str_qr_code);
			return qr_frame;
		}
		
				
		void ar_pose_callback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg)
		{
			ROS_INFO("AR pose markers received");
			int size = msg->markers.size();

			
			for(int i=0; i<size; i++)
			{
				int qr_id = msg->markers[i].id;
				ROS_INFO("Ar marker id: %d", qr_id);
				
				ros::Time qr_pose_time(msg->markers[i].header.stamp.sec, msg->markers[i].header.stamp.nsec);
				ROS_INFO("ar_pose_time, sec: %d, nsec: %d", qr_pose_time.sec, qr_pose_time.nsec);
				
				ros::Time current_time = ros::Time::now();
				ROS_INFO("current_time, sec: %d, nsec: %d", current_time.sec, current_time.nsec);
				
				string qr_frame = get_qr_frame(qr_id);
				string base_frame("base_footprint");
				string odom_frame("odom");
				string map_frame("map");
				
				//tf::StampedTransform base_to_qr;
				tf::StampedTransform odom_to_qr;
				try{
					// wait until the transform is available
					tf_listener_->waitForTransform(odom_frame, qr_frame,
                              ros::Time(0), ros::Duration(3.0));
          
          //ros::Time current_time_after_wait = ros::Time::now();
					//ROS_INFO("current_time_after_wait, sec: %d, nsec: %d", current_time_after_wait.sec, current_time_after_wait.nsec);
                              
          // get transform
		      tf_listener_->lookupTransform(odom_frame, qr_frame,  
		                               ros::Time(0), odom_to_qr); //ros::Time(0) means the latest transform available
		      ROS_INFO("After lookupTransform");
		    }
		    catch (tf::TransformException ex){
		      ROS_ERROR("%s",ex.what());
		      ros::Duration(1.0).sleep();
		    }
		    
		    
		    
		    // Get the known position of the qr marker
		    for(int i=0; i<qr_positions_vector_.size(); i++)
		    {
					ROS_INFO("Comparing %s with %s", qr_frame.c_str(), qr_positions_vector_[i].frame_id.c_str());
					if(qr_frame.compare(qr_positions_vector_[i].frame_id)==0) // is 0 if the strings are equal
					{
						ROS_INFO("QR position exists");
						
						// get known transform map->qr
						const tf::Quaternion quaternion(qr_positions_vector_[i].pose.orientation.x,
																			qr_positions_vector_[i].pose.orientation.y,
																			qr_positions_vector_[i].pose.orientation.z,
																			qr_positions_vector_[i].pose.orientation.w);
						
						const tf::Vector3 position(qr_positions_vector_[i].pose.position.x,
																qr_positions_vector_[i].pose.position.y,
																qr_positions_vector_[i].pose.position.z);		
																
						tf::Transform map_to_qr(quaternion, position);


						// 1. get inverse of map->qr to obtain qr->map
						tf::Stamped<tf::Pose> qr_to_map_stamped (map_to_qr.inverse(),
                                              qr_pose_time,
                                              qr_frame);
						
						
						// 2. get transform odom->map,  transforming qr->map to ofom frame
						tf::Stamped<tf::Pose> odom_to_map;
						tf_listener_->transformPose( odom_frame,
                                 qr_to_map_stamped,
                                 odom_to_map);
                     
                             
            // 3. get the inverse of odom->map to obtain map->odom 
            tf::Transform map_to_odom(tf::Quaternion(odom_to_map.getRotation()),
																			tf::Point(odom_to_map.getOrigin()));    
																			
		        tf::StampedTransform map_to_odom_stamped(map_to_odom.inverse(),
											                          qr_pose_time,
											                          map_frame, odom_frame);      
                                 
            //ROS_INFO("map->odom transform x:%f, y:%f, z:%f", map_to_odom_stamped.getOrigin().x(), map_to_odom_stamped.getOrigin().y(), map_to_odom_stamped.getOrigin().z());
            
            
            // publish map->odom as a geometry_msgs/PoseWithCovarianceStamped
         		qr_pose_msg_ = get_msg_from_tf(map_to_odom_stamped);
						ar_pose_pub_.publish(qr_pose_msg_);
						
						// publish transform map->odom on the tf tree
						if(publish_tf)
						{
							this->tf_broadcaster_->sendTransform(map_to_odom_stamped);
						}
						
						break;
					}
				}
			}
			
		}
		
		
		/*
		 *	\brief Returns a geometry_msgs::PoseWithCovarianceStamped obtained from a tf::StampedTransform
		 */
		geometry_msgs::PoseWithCovarianceStamped get_msg_from_tf(tf::StampedTransform transform)
		{
			geometry_msgs::Pose qr_pose;
			qr_pose.position.x = transform.getOrigin().x();
			qr_pose.position.y = transform.getOrigin().y();
			qr_pose.position.z = transform.getOrigin().z();
			qr_pose.orientation.x = transform.getRotation().x();
			qr_pose.orientation.y = transform.getRotation().y();
			qr_pose.orientation.z = transform.getRotation().z();
			qr_pose.orientation.w = transform.getRotation().w();
			
			geometry_msgs::PoseWithCovariance qr_pose_cov;
			qr_pose_cov.pose = qr_pose;
			// TODO: fill covariance
			
			geometry_msgs::PoseWithCovarianceStamped qr_pose_msg;
			qr_pose_msg.header.seq = qr_pose_msg_seq_;
			qr_pose_msg_seq_++;
			qr_pose_msg.header.stamp = transform.stamp_;
			qr_pose_msg.header.frame_id = "map";
			qr_pose_msg.pose = qr_pose_cov;
			
			return qr_pose_msg;
		}
		
		
		
		/*
		 *	\brief Parses yaml file with the qr positions
		 *			saves qr positions in qr_positions_vector_
		 */
		bool parse_qr_positions_yaml()
		{
		  qr_position qr_position_aux;
		
		  // Parse yaml file 
		  XmlRpc::XmlRpcValue qr_positions_array;
		  if(nh_.getParam("qr_positions", qr_positions_array))
		  {
				if(qr_positions_array.getType() == XmlRpc::XmlRpcValue::TypeArray)
		      {
						for(int i = 0; i < qr_positions_array.size(); ++i)
						{
				     //ROS_INFO("waypoint_array[i].getType = %d", waypoint_array[i].getType() );         
				     if(qr_positions_array[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
				     {
								if(qr_positions_array[i].hasMember("frame_id") && qr_positions_array[i].hasMember("position"))
								{	
				          //read positions 
				          std::string frame_id = qr_positions_array[i]["frame_id"];		 
									qr_position_aux.frame_id = frame_id;
									ROS_INFO("Frame_id: %s", frame_id.c_str());
								  qr_position_aux.pose.position.x = qr_positions_array[i]["position"][0];		
								  qr_position_aux.pose.position.y = qr_positions_array[i]["position"][1];		
								  qr_position_aux.pose.position.z = qr_positions_array[i]["position"][2];	
								  qr_position_aux.pose.orientation.x = qr_positions_array[i]["position"][3];	
								  qr_position_aux.pose.orientation.y = qr_positions_array[i]["position"][4];	
								  qr_position_aux.pose.orientation.z = qr_positions_array[i]["position"][5];	
								  qr_position_aux.pose.orientation.w = qr_positions_array[i]["position"][6];	
								  qr_positions_vector_.push_back( qr_position_aux );		 
									ROS_INFO("frame_id=%s [x=%5.2f, y=%5.2f, z=%5.2f, x=%5.2f, y=%5.2f, z=%5.2f, w=%5.2f]", frame_id.c_str(),qr_position_aux.pose.position.x,
										qr_position_aux.pose.position.y, qr_position_aux.pose.position.z, qr_position_aux.pose.orientation.x, qr_position_aux.pose.orientation.y,
										qr_position_aux.pose.orientation.z, qr_position_aux.pose.orientation.w); 		
				        }
							} 
						}
				}
			}
			
			ROS_INFO("Parsed %d qr_positions", (int)qr_positions_vector_.size());
			
		}
	
};
		




int main(int argc, char** argv)
{
	ros::init(argc, argv, "qr_localization");


	QRLocalization qrLocalization = QRLocalization();
	
	qrLocalization.parse_qr_positions_yaml();
	
	qrLocalization.start();
	
	return 0;
}
