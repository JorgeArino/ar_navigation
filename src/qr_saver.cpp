#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <string.h>

#include <ar_navigation/save_qr.h>
#include <boost/lexical_cast.hpp>

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/emitter.h>

#include <iostream>
#include <fstream>

#define PI 3.1415

using namespace std;

typedef struct
{
  std::string frame_id;
  geometry_msgs::Pose pose;
} qr_position;

/*!
 * /brief Class to define the qr_saver component
 */
class QRSaver
{
public:

  // Variables
  ros::NodeHandle nh_;
  ros::Publisher cmd_pub_;
  ros::Subscriber amcl_pose_sub_;
  ros::ServiceServer save_qr_srv;

  float desired_freq_;
  tf::TransformListener * tf_listener_;

  //ofstream* qr_positions_file;


  geometry_msgs::Pose amcl_pose;

  // Waypoint array (not limited)
  std::vector < qr_position > qr_positions_vector_;

  // Methods
  QRSaver ()
  {
    ROS_INFO ("SETUP");

    desired_freq_ = 20.0;
    tf_listener_ = new tf::TransformListener ();

    // Publishers                   
    //cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/summit_xl/helios_diff_controller/cmd_vel",1);

    // Subscribers
    amcl_pose_sub_ =
    nh_.subscribe < geometry_msgs::PoseWithCovarianceStamped > ("amcl_pose", 1, &QRSaver::amcl_pose_callback, this);

    // Services
    save_qr_srv = nh_.advertiseService ("/save_qr", &QRSaver::save_qr, this);

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


    ROS_INFO ("Setup finished");
  };


  void start ()
  {
    ros::Rate r (desired_freq_);
    while (ros::ok ())
    {
      ros::spinOnce ();

      // NODE MAIN LOOP
      //ROS_INFO("Main loop");

      r.sleep ();
    }

    //qr_positions_file->close();
    ros::shutdown ();
  };


  bool save_qr (ar_navigation::save_qr::Request & req, ar_navigation::save_qr::Response & res)
  {
    ROS_INFO ("Call to save_qr service");

    int qr_code = req.qr_code;

    string qr_frame ("ar_marker_");
    string str_qr_code = boost::lexical_cast < string > (qr_code);
    qr_frame.append (str_qr_code);

    string base_frame ("camera_link");
    string map_frame ("map");

    ROS_INFO ("Request transform from frame: %s, to: %s", map_frame.c_str (), qr_frame.c_str ());

    tf::StampedTransform qr_to_map;
    try
    {
      tf_listener_->lookupTransform (map_frame, qr_frame, ros::Time (0), qr_to_map);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR ("%s", ex.what ());
      ros::Duration (1.0).sleep ();
    }
    ROS_INFO ("QR transform requested x: %f, y: %f, z: %f", qr_to_map.getOrigin ().x (), qr_to_map.getOrigin ().y (),
              qr_to_map.getOrigin ().z ());

    add_qr_position_to_param (qr_code, qr_to_map);

    return true;
  }

  bool add_qr_position_to_param (int qr_code, tf::StampedTransform transform)
  {
    // Parse yaml file 
    XmlRpc::XmlRpcValue qr_positions_array;
    if (nh_.getParam ("qr_positions", qr_positions_array))
    {
      int size = qr_positions_array.size ();
      ROS_INFO ("qr_positions size: %d", size);

      int qr_index = size + 1;

      // gets qr_frame name
      string qr_frame ("qr_marker_");
      string str_qr_code = boost::lexical_cast < string > (qr_code);
      qr_frame.append (str_qr_code);

      qr_positions_array[qr_index]["frame_id"] = qr_frame;
      qr_positions_array[qr_index]["position"][0] = transform.getOrigin ().x ();
      qr_positions_array[qr_index]["position"][1] = transform.getOrigin ().y ();
      qr_positions_array[qr_index]["position"][2] = transform.getOrigin ().z ();
      qr_positions_array[qr_index]["position"][3] = transform.getRotation ().x ();
      qr_positions_array[qr_index]["position"][4] = transform.getRotation ().y ();
      qr_positions_array[qr_index]["position"][5] = transform.getRotation ().z ();
      qr_positions_array[qr_index]["position"][6] = transform.getRotation ().w ();

      nh_.setParam ("qr_positions", qr_positions_array);
    }
  }

  void amcl_pose_callback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg)
  {
    ROS_INFO ("Amcl msg received");
    amcl_pose = msg->pose.pose;
  }

  /*
   *      \brief Reads yaml file with the qr positions
   *
   */
  bool parse_qr_positions_yaml ()
  {
    qr_position qr_position_aux;

    // Parse yaml file 
    XmlRpc::XmlRpcValue qr_positions_array;
    if (nh_.getParam ("qr_positions", qr_positions_array))
    {
      if (qr_positions_array.getType () == XmlRpc::XmlRpcValue::TypeArray)
      {
        for (int i = 0; i < qr_positions_array.size (); ++i)
        {
          //ROS_INFO("waypoint_array[i].getType = %d", waypoint_array[i].getType() );         
          if (qr_positions_array[i].getType () == XmlRpc::XmlRpcValue::TypeStruct)
          {
            if (qr_positions_array[i].hasMember ("frame_id") && qr_positions_array[i].hasMember ("position"))
            {
              //read positions 
              std::string frame_id = qr_positions_array[i]["frame_id"];
              qr_position_aux.frame_id = frame_id;
              ROS_INFO ("Frame_id: %s", frame_id.c_str ());
              qr_position_aux.pose.position.x = qr_positions_array[i]["position"][0];
              qr_position_aux.pose.position.y = qr_positions_array[i]["position"][1];
              qr_position_aux.pose.position.z = qr_positions_array[i]["position"][2];
              qr_position_aux.pose.orientation.x = qr_positions_array[i]["position"][3];
              qr_position_aux.pose.orientation.y = qr_positions_array[i]["position"][4];
              qr_position_aux.pose.orientation.z = qr_positions_array[i]["position"][5];
              qr_position_aux.pose.orientation.w = qr_positions_array[i]["position"][6];
              qr_positions_vector_.push_back (qr_position_aux);
              ROS_INFO ("frame_id=%s [x=%5.2f, y=%5.2f, z=%5.2f, x=%5.2f, y=%5.2f, z=%5.2f, w=%5.2f]",
                        frame_id.c_str (), qr_position_aux.pose.position.x, qr_position_aux.pose.position.y,
                        qr_position_aux.pose.position.z, qr_position_aux.pose.orientation.x,
                        qr_position_aux.pose.orientation.y, qr_position_aux.pose.orientation.z,
                        qr_position_aux.pose.orientation.w);
            }
          }
        }
      }
    }
  }

};





int main (int argc, char **argv)
{
  ros::init (argc, argv, "qr_saver");


  QRSaver qrSaver = QRSaver ();

  qrSaver.parse_qr_positions_yaml ();

  qrSaver.start ();

  return 0;
}
