#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
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
		ros::Subscriber odom_sub_;
		ros::ServiceServer save_qr_srv;
		ros::Publisher ar_pose_pub_;
		
		float desired_freq_;
		tf::TransformListener* tf_listener_;	
		tf::TransformBroadcaster* tf_broadcaster_;	 		
			
		
		geometry_msgs::Pose amcl_pose;
		geometry_msgs::PoseWithCovarianceStamped qr_pose_msg_;
		int qr_pose_msg_seq_;
		
		// AlvarMarkers msg received
		//ar_track_alvar::AlvarMarkers current_alvar_markers_;
		
		// Waypoint array (not limited)
		std::vector<qr_position> qr_positions_vector_;
		
		// To publish the map_odom tf 
		bool publish_tf;
		
		
		
			
		// Methods
		QRLocalization()
		{
			ROS_INFO("SETUP");
			
			desired_freq_ = 5.0;
			
			// new tf listener and broadcaster
			tf_listener_ = new tf::TransformListener();
			tf_broadcaster_ = new tf::TransformBroadcaster();

			// Publishers			
			ar_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("ar_pose",1);
			
			// Subscribers
			ar_marker_sub_ = nh_.subscribe<ar_track_alvar::AlvarMarkers>("/ar_pose_marker", 1, &QRLocalization::ar_pose_callback, this);
			//odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 1, &QRLocalization::odom_callback, this);
			

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

			//ROS_INFO("Waiting ...");
			//ros::Duration(5.0).sleep();
			
			ROS_INFO("Setup finished");
		};
		
		
		void start()
		{
			ros::Rate r(desired_freq_);
			while(ros::ok())
			{
				ros::spinOnce();
				r.sleep();
			}
			
			//qr_positions_file->close();
			ros::shutdown();
		};
		
		
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
			//ROS_INFO("AR pose markers received");

			ros::Time start_callback_time = ros::Time::now();

			int size = msg->markers.size();

			float x_map_base;
			float y_map_base;
			float yaw_map_base;
			
			// publish the map->base transform for each qr in the received message
			for(int i=0; i<size; i++)
			{
				int qr_id = msg->markers[i].id;
				//ROS_INFO("Ar marker id: %d", qr_id);
				
				string qr_frame = get_qr_frame(qr_id);
				string base_frame("base_footprint");
				string odom_frame("odom");
				string map_frame("map");
				
				float x_msg = msg->markers[i].pose.pose.position.x;
				float y_msg = msg->markers[i].pose.pose.position.y;
			  tf::Quaternion quaternion_msg(msg->markers[i].pose.pose.orientation.x,
																			msg->markers[i].pose.pose.orientation.y,
																			msg->markers[i].pose.pose.orientation.z,
																			msg->markers[i].pose.pose.orientation.w);
				float yaw_msg = getYaw(quaternion_msg);
				//ROS_INFO("x_msg: %f, y_msg: %f, yaw_msg: %f", x_msg, y_msg, yaw_msg);
			


		    // Get the known position of the qr marker
		    for(int i=0; i<qr_positions_vector_.size(); i++)
		    {
					//ROS_INFO("Comparing %s with %s", qr_frame.c_str(), qr_positions_vector_[i].frame_id.c_str());
					if(qr_frame.compare(qr_positions_vector_[i].frame_id)==0) // is 0 if the strings are equal
					{
						//ROS_INFO("QR position exists");
						
						// get known transform map->qr 
						const tf::Quaternion quaternion(qr_positions_vector_[i].pose.orientation.x,
																			qr_positions_vector_[i].pose.orientation.y,
																			qr_positions_vector_[i].pose.orientation.z,
																			qr_positions_vector_[i].pose.orientation.w);
						const tf::Vector3 position(qr_positions_vector_[i].pose.position.x,
																qr_positions_vector_[i].pose.position.y,
																qr_positions_vector_[i].pose.position.z);		
															
						tf::Transform map_to_qr(quaternion, position);
						float map_to_qr_yaw = getYaw(quaternion); 
						//ROS_INFO("map->qr x: %f,  y: %f, o: %f", map_to_qr.getOrigin().x(), map_to_qr.getOrigin().y(), map_to_qr_yaw);

						
						float t_map_qr[3][3] = { {cos(map_to_qr_yaw), -sin(map_to_qr_yaw), map_to_qr.getOrigin().x()},
																			{sin(map_to_qr_yaw), cos(map_to_qr_yaw), map_to_qr.getOrigin().y()},
																			{0, 0, 1} };
						float rot_qr_cam[3][3] = { {cos(-yaw_msg), -sin(-yaw_msg), 0},
																			{sin(-yaw_msg), cos(-yaw_msg), 0},
																			{0, 0, 1}};
						float trans_qr_cam[3][3] = {{ 1, 0, -x_msg},
																			{0, 1, -y_msg},
																			{0, 0, 1}};
																																
						float beta_increment = PI/2; // before PI/4
						float t_cam_base[3][3] = { {cos(beta_increment), -sin(beta_increment), 0},  // TODO: some translation here?
																			{sin(beta_increment), cos(beta_increment), 0},
																			{0, 0, 1}};		
	
						float t_aux_1[3][3];
						float t_map_cam[3][3];
						float t_map_base[3][3];
						multiply3x3Matrix(t_map_qr, rot_qr_cam, t_aux_1);
						multiply3x3Matrix(t_aux_1, trans_qr_cam, t_map_cam);
						multiply3x3Matrix(t_map_cam, t_cam_base, t_map_base);
						
						x_map_base = t_map_base[0][2];
						y_map_base = t_map_base[1][2];
						yaw_map_base = atan2(t_map_base[1][0],t_map_base[0][0]);																												
						//ROS_INFO("x_map_base: %f, y_map_base: %f, yaw_map_base: %f", x_map_base, y_map_base, yaw_map_base);

			    
					
						tf::Transform map_to_base(tf::createQuaternionFromYaw(yaw_map_base),  
													 tf::Vector3(x_map_base, y_map_base, 0.0));
						tf::StampedTransform map_to_base_stamped(map_to_base,
											ros::Time::now(),  
											map_frame, base_frame);
            
            // publish map->odom as a geometry_msgs/PoseWithCovarianceStamped
         		qr_pose_msg_ = get_msg_from_tf(map_to_base_stamped);
						ar_pose_pub_.publish(qr_pose_msg_);
						
						// publish transform map->base on the tf tree
						if(publish_tf)
						{
							this->tf_broadcaster_->sendTransform(map_to_base_stamped);
						}
						
						break;  // Stop searching for a qr match
					}
				} // end iterating qr_positions_vector_
			} // end iterating msg->markers 
		}  // end AR callback


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
		
		void multiply3x3Matrix( float input1[3][3], float input2[3][3], float (&result)[3][3])
		{
			for(int i=0; i<3; i++)
			{
				for(int j=0; j<3; j++)
				{
					result[i][j] = input1[i][0]*input2[0][j] + input1[i][1]*input2[1][j] + input1[i][2]*input2[2][j];
				}
			}
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
