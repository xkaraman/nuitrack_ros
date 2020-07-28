#ifndef NUITRACK_CORE_H
#define NUITRACK_CORE_H

#include <ros/ros.h>
#include <nuitrack/Nuitrack.h>
#include <sensor_msgs/PointCloud2.h>

using namespace tdv::nuitrack;

class NuitrackCore
{
public:
  NuitrackCore(std::string name);
  ~NuitrackCore();

  void init(const std::string& config);
  void run();

private:
  void timerCallback(const ros::TimerEvent& event);
  void onNewUser(int id);
  void onLostUser(int id);
  void onUserUpdate(UserFrame::Ptr frame);
  void onNewRGBFrame(RGBFrame::Ptr frame);
  void onNewDepthFrame(DepthFrame::Ptr frame);
  void onSkeletonUpdate(SkeletonData::Ptr skeletonData);
  void onHandUpdate(HandTrackerData::Ptr handData);
  void onNewGesture(GestureData::Ptr gestureData);

private:
    std::string nodeName_;
    std::string camera_depth_frame_;
    std::string camera_color_frame_;

    bool enable_publishing_frames_;

    sensor_msgs::PointCloud2 cloud_msg_; // color and depth point cloud

    ros::NodeHandle nodeHandle;


    ros::Timer timer_;
    ros::Publisher pub_rgb_data_;
    ros::Publisher pub_pcl_data_;
    ros::Publisher pub_user_data_;
    ros::Publisher pub_skeleton_data_;
    ros::Publisher pub_event_person_appeared_;
    ros::Publisher pub_event_person_disappeared_;

    int frame_width_;
    int frame_height_;

    int last_id_;
    std::vector<int> current_user_list_;

    ColorSensor::Ptr colorSensor_;
    DepthSensor::Ptr depthSensor_;
    UserTracker::Ptr userTracker_;
    SkeletonTracker::Ptr skeletonTracker_;
    tdv::nuitrack::HandTracker::Ptr handTracker_;
    tdv::nuitrack::GestureRecognizer::Ptr gestureRecognizer_;
    OutputMode outputMode_;
};

#endif // NUITRACK_CORE_H
