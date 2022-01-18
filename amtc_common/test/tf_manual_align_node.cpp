#include <ros/ros.h>
#include <amtc_common/Communications.h>
#include <vector>
#include <boost/foreach.hpp>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>


amtc::Communications* comms_ptr;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "pose_manual_align_node");
  ros::NodeHandle nh("~");
  comms_ptr = new amtc::Communications(nh);

  std::string   frame_id;
  std::string   fixed_frame_id;
//  std::string fixed_frame_id = get_local_param<std::string>("frame_id", "base_link");

  LOAD_PARAM(nh,std::string("child_frame_id"),frame_id,std::string("frame_id"));
  LOAD_PARAM(nh,std::string("frame_id"),fixed_frame_id,std::string("base_link"));

  static tf2_ros::StaticTransformBroadcaster br;
  //static tf2_ros::TransformBroadcaster br;

  double delta          = 0.1;

  double offset_x       = 0.0;
  double offset_y       = 0.0;
  double offset_z       = 0.0;
  double offset_roll    = 0.0;
  double offset_pitch   = 0.0;
  double offset_yaw     = 0.0;

  struct termios old_tio, new_tio;
  //unsigned char c;

  /* get the terminal settings for stdin */
  tcgetattr(STDIN_FILENO,&old_tio);

  /* we want to keep the old setting to restore them a the end */
  new_tio=old_tio;

  /* disable canonical mode (buffered i/o) and local echo */
  new_tio.c_lflag &=(~ICANON & ~ECHO);

  /* set the new settings immediately */
  tcsetattr(STDIN_FILENO,TCSANOW,&new_tio);
  bool end = false;
  geometry_msgs::TransformStamped transformStamped;

  while(end == false && ros::ok())
  {
    comms_ptr->spin();

    switch(getchar())
    {
    case '1':
      delta = 0.001;
      break;
    case '2':
      delta = 0.01;
      break;
    case '3':
      delta = 0.1;
      break;
    case '4':
      delta = 1.0;
      break;
    case 'w':
      offset_x += delta;
      break;
    case 's':
      offset_x -= delta;
      break;
    case 'a':
      offset_y += delta;
      break;
    case 'd':
      offset_y -= delta;
      break;
    case 'r':
      offset_z += delta;
      break;
    case 'f':
      offset_z -= delta;
      break;
    case 'i':
      offset_pitch += delta;
      break;
    case 'k':
      offset_pitch -= delta;
      break;
    case 'j':
      offset_roll += delta;
      break;
    case 'l':
      offset_roll -= delta;
      break;
    case 'u':
      offset_yaw += delta;
      break;
    case 'o':
      offset_yaw -= delta;
      break;
    case 'q':
      end = true;
      ROS_INFO_NODE("\n<launch>\n  <node pkg=\"tf2_ros\" type=\"static_transform_publisher\" name=\"%s_to_%s_broadcaster\" args=\"%.5f %.5f %.5f %.5f %.5f %.5f %.5f %s %s\" />\n</launch>",fixed_frame_id.c_str(), frame_id.c_str(),transformStamped.transform.translation.x, transformStamped.transform.translation.y,transformStamped.transform.translation.z,transformStamped.transform.rotation.x,transformStamped.transform.rotation.y,transformStamped.transform.rotation.z,transformStamped.transform.rotation.w,fixed_frame_id.c_str(), frame_id.c_str());
      ROS_INFO_NODE("\n<launch>\n  <node pkg=\"tf\" type=\"static_transform_publisher\" name=\"%s_to_%s_broadcaster\" args=\"%.5f %.5f %.5f %.5f %.5f %.5f %.5f %s %s 100\" />\n</launch>",fixed_frame_id.c_str(), frame_id.c_str(),transformStamped.transform.translation.x, transformStamped.transform.translation.y,transformStamped.transform.translation.z,transformStamped.transform.rotation.x,transformStamped.transform.rotation.y,transformStamped.transform.rotation.z,transformStamped.transform.rotation.w,fixed_frame_id.c_str(), frame_id.c_str());

      //ros::shutdown();
      //continue;
      break;
    default:
      break;
    }
    ROS_INFO_NODE("delta: %.3f\tx: %.3f\ty: %.3f\tz: %.3f\troll: %.3f\tpitch: %.3f\tyaw: %.3f", delta, offset_x, offset_y, offset_z, offset_roll, offset_pitch, offset_yaw );


    transformStamped.header.stamp         = ros::Time::now();
    transformStamped.header.frame_id      = fixed_frame_id;
    transformStamped.child_frame_id       = frame_id;

    transformStamped.transform.translation.x  =  offset_x;
    transformStamped.transform.translation.y  =  offset_y;
    transformStamped.transform.translation.z  =  offset_z;

    tf2::Quaternion q_orig, q_rot, q_new;
    q_orig.setW( 1.0 );
    q_orig.setX( 0.0 );
    q_orig.setY( 0.0 );
    q_orig.setZ( 0.0 );

    q_rot.setRPY( offset_roll, offset_pitch, offset_yaw);
    q_new   = q_rot*q_orig;
    q_new.normalize();

    transformStamped.transform.rotation.w     = q_new.w();
    transformStamped.transform.rotation.x     = q_new.x();
    transformStamped.transform.rotation.y     = q_new.y();
    transformStamped.transform.rotation.z     = q_new.z();

    br.sendTransform(transformStamped);
  }
  
  tcsetattr(STDIN_FILENO,TCSANOW,&old_tio);
  return 0;
}





