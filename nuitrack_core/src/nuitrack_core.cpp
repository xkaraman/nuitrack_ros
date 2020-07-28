#include <nuitrack_core.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <nuitrack_msgs/UserData.h>
#include <nuitrack_msgs/UserDataArray.h>
#include <nuitrack_msgs/EventUserUpdate.h>
#include <nuitrack_msgs/SkeletonData.h>
#include <nuitrack_msgs/SkeletonDataArray.h>

using namespace tdv::nuitrack;

std::map<int, std::string> JOINT_NAMES = {{JOINT_HEAD, "joint_head"}, {JOINT_NECK, "joint_neck"}, {JOINT_TORSO, "joint_torso"}, {JOINT_WAIST, "joint_waist"},
                                          {JOINT_LEFT_COLLAR, "joint_left_collar"}, {JOINT_LEFT_SHOULDER, "joint_left_shoulder"}, {JOINT_LEFT_ELBOW, "joint_left_elbow"}, {JOINT_LEFT_WRIST, "joint_left_wrist"}, {JOINT_LEFT_HAND, "joint_left_hand"},
                                          {JOINT_RIGHT_COLLAR, "joint_right_collar"}, {JOINT_RIGHT_SHOULDER, "joint_right_shoulder"}, {JOINT_RIGHT_ELBOW, "joint_right_elbow"}, {JOINT_RIGHT_WRIST, "joint_right_wrist"}, {JOINT_RIGHT_HAND, "joint_right_hand"},
                                          {JOINT_LEFT_HIP, "joint_left_hip"}, {JOINT_LEFT_KNEE, "joint_left_knee"}, {JOINT_LEFT_ANKLE, "joint_left_ankle"},
                                          {JOINT_RIGHT_HIP, "joint_right_hip"}, {JOINT_RIGHT_KNEE, "joint_right_knee"}, {JOINT_RIGHT_ANKLE, "joint_right_ankle"}};

NuitrackCore::NuitrackCore(std::string name):
nodeName_(name),
nodeHandle()
{
  // Read Parameters
  nodeHandle.param<std::string>("camera_depth_frame",camera_depth_frame_,"camera_depth_frame");
  nodeHandle.param<std::string>("camera_color_frame",camera_color_frame_,"camera_color_frame");
  nodeHandle.param<bool>("enable_publishing_frames",enable_publishing_frames_,true);

  pub_rgb_data_ = nodeHandle.advertise<sensor_msgs::Image>("/nuitrack/rgb/image_raw", 1);
  pub_pcl_data_ = nodeHandle.advertise<sensor_msgs::PointCloud2>("/nuitrack/depth/points", 1);
  pub_skeleton_data_ = nodeHandle.advertise<nuitrack_msgs::SkeletonDataArray>("/nuitrack/skeletons", 1);
  pub_user_data_ = nodeHandle.advertise<nuitrack_msgs::UserDataArray>("/nuitrack/detected_users", 10);
  pub_event_person_appeared_ = nodeHandle.advertise<nuitrack_msgs::EventUserUpdate>("/nuitrack/event/person_appeared", 10);
  pub_event_person_disappeared_ = nodeHandle.advertise<nuitrack_msgs::EventUserUpdate>("/nuitrack/event/person_disappeared", 10);

  ROS_INFO("Initialized %s...",nodeName_.c_str());
}

    NuitrackCore::~NuitrackCore()
    {
        try
        {
            timer_.stop();
            Nuitrack::release();
        }
        catch (const Exception& e) {} // Do nothing
    }

    void NuitrackCore::timerCallback(const ros::TimerEvent& event)
    {
        try
        {
            Nuitrack::update(colorSensor_);
            Nuitrack::update(depthSensor_);
            Nuitrack::update(userTracker_);
            Nuitrack::update(skeletonTracker_);
        }
        catch (LicenseNotAcquiredException& e)
        {
            std::cerr << "LicenseNotAcquired exception (ExceptionType: " << e.type() << ")" << std::endl;
            assert(false);
        }
        catch (const Exception& e)
        {
            std::cerr << "Nuitrack update failed (ExceptionType: " << e.type() << ")" << std::endl;
            assert(false);
        }
    }

    void NuitrackCore::onNewUser(int id)
    {
        current_user_list_.push_back(id);

        nuitrack_msgs::EventUserUpdate msg;
        msg.key_id = id;
        msg.user_ids = current_user_list_;

        pub_event_person_appeared_.publish(msg);
    }

    void NuitrackCore::onLostUser(int id)
    {
        bool found_exist = false;
        for(size_t i = 0; i < current_user_list_.size(); i++)
        {
            if(current_user_list_[i] == id)
            {
                current_user_list_.erase(current_user_list_.begin() + i);
                found_exist = true;
                break;
            }
        }
        if(!found_exist)
            return;

        nuitrack_msgs::EventUserUpdate msg;
        msg.key_id = id;
        msg.user_ids = current_user_list_;

        pub_event_person_disappeared_.publish(msg);
    }

    void NuitrackCore::onUserUpdate(UserFrame::Ptr frame)
    {
        nuitrack_msgs::UserDataArray msg;
        std::vector<User> users = frame->getUsers();

        int width = frame->getCols();
        int height = frame->getRows();

        for(size_t i = 0; i < users.size(); i++)
        {
            nuitrack_msgs::UserData user;

            user.id = users[i].id;
            user.real.x = users[i].real.x;  user.real.y = users[i].real.y; user.real.z = users[i].real.z;
            user.proj.x = users[i].proj.x;  user.proj.y = users[i].proj.y; user.proj.z = users[i].proj.z;
            user.box.x_offset = users[i].box.left * width;
            user.box.y_offset = users[i].box.top * height;
            user.box.height = (users[i].box.bottom - users[i].box.top) * height;
            user.box.width = (users[i].box.right - users[i].box.left) * width;
            user.occlusion = users[i].occlusion;

            msg.users.push_back(user);
        }

        pub_user_data_.publish(msg);
    }

    void NuitrackCore::onNewRGBFrame(RGBFrame::Ptr frame)
    {
        const Color3* colorPtr = frame->getData();

        sensor_msgs::Image img;
        img.header.stamp = ros::Time::now();

        img.width = frame->getCols();
        img.height = frame->getRows();
        img.encoding = "rgb8";
        img.is_bigendian = 0;
        img.step = 3 * frame->getCols();

        img.data.resize(img.width * img.height * 3);

        for(size_t i = 0; i < img.width * img.height; i++)
        {
            img.data[i * 3 + 0] = colorPtr[i].blue;
            img.data[i * 3 + 1] = colorPtr[i].green;
            img.data[i * 3 + 2] = colorPtr[i].red;
        }

        pub_rgb_data_.publish(img);
    }

//    sensor_msgs::PointCloud2 depthToCloud(int width, int height, const uint16_t* depth)
//    {
//        sensor_msgs::PointCloud2 cloud;
//        cloud.header.frame_id = "nuitrack_link";
//        cloud.header.stamp = ros::Time::now();

//        cloud.is_bigendian = false;
//        cloud.is_dense = false;

//        sensor_msgs::PointCloud2Modifier modifier(cloud);
//        modifier.setPointCloud2FieldsByString(1, "xyz");
//        modifier.resize(width * height);

//        cloud.width = width;
//        cloud.height = height;
//        cloud.row_step = 16 * width;

//        sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
//        sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
//        sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");

//        for(int h = 0; h < height; h++)
//        {
//            for(int w = 0; w < width; w++)
//            {
//                Vector3 data = depthSensor_->convertProjToRealCoords(w, h, depth[h * width + w]);

//                *out_x = data.z / 1000.0;
//                *out_y = -data.x / 1000.0;
//                *out_z = data.y / 1000.0;

//                ++out_x;
//                ++out_y;
//                ++out_z;
//            }
//        }

//        return cloud;
//    }

    void NuitrackCore::onNewDepthFrame(DepthFrame::Ptr frame)
    {
//        const uint16_t* depthPtr = frame->getData();

//        sensor_msgs::PointCloud2 points = depthToCloud(frame->getCols(), frame->getRows(), depthPtr);
//        pub_pcl_data_.publish(points);
    }

    void NuitrackCore::onSkeletonUpdate(SkeletonData::Ptr skeletonData)
    {
        nuitrack_msgs::SkeletonDataArray msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "nuitrack_link";

        auto skeletons = skeletonData->getSkeletons();
        for(size_t i = 0; i < skeletonData->getNumSkeletons(); i++)
        {
            nuitrack_msgs::SkeletonData data;

            data.id = skeletons[i].id;

            for(auto const& j : JOINT_NAMES)
            {
                data.joints.push_back(j.second);

                geometry_msgs::Point p;
                p.x = skeletons[i].joints[j.first].real.x;
                p.y = skeletons[i].joints[j.first].real.y;
                p.z = skeletons[i].joints[j.first].real.z;

                data.joint_pos.push_back(p);
            }

            msg.skeletons.push_back(data);
        }

        pub_skeleton_data_.publish(msg);
    }
    void NuitrackCore::onHandUpdate(HandTrackerData::Ptr handData){

    }

    void NuitrackCore::onNewGesture(GestureData::Ptr gestureData){

    }


    void NuitrackCore::init(const std::string& config)
    {
      // Initialize Nuitrack first, then create Nuitrack modules
      ROS_INFO("%s: Initializing...", nodeName_.c_str());
      // std::cout << "Nuitrack: Initializing..." << std::endl;

      try
      {
        tdv::nuitrack::Nuitrack::init(config);
      }
      catch (const tdv::nuitrack::Exception& e)
      {
        std::cerr <<
        "Can not initialize Nuitrack (ExceptionType: " << e.type() << ")" << std::endl;
        exit(EXIT_FAILURE);
      }

      // Set config values.  Overrides $NUITRACK_HOME/data/nuitrack.config

      // Align depth and color
      Nuitrack::setConfigValue("DepthProvider.Depth2ColorRegistration", "true");

      // Realsense Depth Module - force to 848x480 @ 60 FPS
      Nuitrack::setConfigValue("Realsense2Module.Depth.Preset", "5");
      Nuitrack::setConfigValue("Realsense2Module.Depth.RawWidth", "848");
      Nuitrack::setConfigValue("Realsense2Module.Depth.RawHeight", "480");
      Nuitrack::setConfigValue("Realsense2Module.Depth.ProcessWidth", "848");
      Nuitrack::setConfigValue("Realsense2Module.Depth.ProcessHeight", "480");
      Nuitrack::setConfigValue("Realsense2Module.Depth.FPS", "60");

      // Realsense RGB Module - force to 848x480 @ 60 FPS
      Nuitrack::setConfigValue("Realsense2Module.RGB.RawWidth", "848");
      Nuitrack::setConfigValue("Realsense2Module.RGB.RawHeight", "480");
      Nuitrack::setConfigValue("Realsense2Module.RGB.ProcessWidth", "848");
      Nuitrack::setConfigValue("Realsense2Module.RGB.ProcessHeight", "480");
      Nuitrack::setConfigValue("Realsense2Module.RGB.FPS", "60");

      // Enable face tracking
      Nuitrack::setConfigValue("Faces.ToUse", "true");

      //Options for debug
      //Nuitrack::setConfigValue("Skeletonization.ActiveUsers", "1");
      //Nuitrack::setConfigValue("DepthProvider.Mirror", "true");
//      depth_frame_number_ = 0;
//      color_frame_number_ = 0;

      // Create all required Nuitrack modules

      std::cout << "Nuitrack: DepthSensor::create()" << std::endl;
      depthSensor_ = tdv::nuitrack::DepthSensor::create();
      // Bind to event new frame
      depthSensor_->connectOnNewFrame(std::bind(
      &NuitrackCore::onNewDepthFrame, this, std::placeholders::_1));

      std::cout << "Nuitrack: ColorSensor::create()" << std::endl;
      colorSensor_ = tdv::nuitrack::ColorSensor::create();
      // Bind to event new frame
      colorSensor_->connectOnNewFrame(std::bind(
      &NuitrackCore::onNewRGBFrame, this, std::placeholders::_1));

      outputMode_ = depthSensor_->getOutputMode();
      OutputMode colorOutputMode = colorSensor_->getOutputMode();
      if ((colorOutputMode.xres != outputMode_.xres) || (colorOutputMode.yres != outputMode_.yres))
      {
          ROS_WARN("%s: WARNING! DEPTH AND COLOR SIZE NOT THE SAME!", nodeName_.c_str() );
      }

      // Use depth as the frame size
      frame_width_ = outputMode_.xres;
      frame_height_ = outputMode_.yres;
      last_id_ = -1;
      std::cout << "========= Nuitrack: GOT DEPTH SENSOR =========" << std::endl;
      std::cout << "Nuitrack: Depth:  width = " << frame_width_ <<
        "  height = " << frame_height_ << std::endl;

      // Point Cloud message (includes depth and color)
      int numpoints = frame_width_ * frame_height_;
      cloud_msg_.header.frame_id = camera_depth_frame_;
      //cloud_msg_.header.stamp = ros::Time::now();
      cloud_msg_.width  = numpoints;
      cloud_msg_.height = 1;
      cloud_msg_.is_bigendian = false;
      cloud_msg_.is_dense = false; // there may be invalid points

      sensor_msgs::PointCloud2Modifier modifier(cloud_msg_);
      modifier.setPointCloud2FieldsByString(2,"xyz","rgb");
      modifier.resize(numpoints);


      std::cout << "Nuitrack: UserTracker::create()" << std::endl;
      userTracker_ = tdv::nuitrack::UserTracker::create();
      // Bind to event update user tracker
      userTracker_->connectOnUpdate(std::bind(
        &NuitrackCore::onUserUpdate, this, std::placeholders::_1));

      std::cout << "Nuitrack: SkeletonTracker::create()" << std::endl;
      skeletonTracker_ = tdv::nuitrack::SkeletonTracker::create();
      // Bind to event update skeleton tracker
      skeletonTracker_->connectOnUpdate(std::bind(
        &NuitrackCore::onSkeletonUpdate, this, std::placeholders::_1));

      std::cout << "Nuitrack: HandTracker::create()" << std::endl;
      handTracker_ = tdv::nuitrack::HandTracker::create();
      // Bind to event update Hand tracker
      handTracker_->connectOnUpdate(std::bind(
        &NuitrackCore::onHandUpdate, this, std::placeholders::_1));

      std::cout << "Nuitrack: GestureRecognizer::create()" << std::endl;
      gestureRecognizer_ = tdv::nuitrack::GestureRecognizer::create();
      gestureRecognizer_->connectOnNewGestures(std::bind(
        &NuitrackCore::onNewGesture, this, std::placeholders::_1));

      ROS_INFO("%s: Init complete.  Waiting for frames...", nodeName_.c_str());

    }

    void NuitrackCore::run()
    {
      // Initialize Nuitrack first, then create Nuitrack modules
      ROS_INFO("%s: Running...", nodeName_.c_str());
      // std::cout << "Nuitrack: Running..." << std::endl;
      // Start Nuitrack
      try
      {
          tdv::nuitrack::Nuitrack::run();
      }
      catch (const tdv::nuitrack::Exception& e)
      {
          std::cerr << "Can not start Nuitrack (ExceptionType: "
            << e.type() << ")" << std::endl;
          return;
      }

      ROS_INFO("%s: Waiting for person to be detected...", nodeName_.c_str());

      // Run Loop
      ros::Rate r(60); // hz
      while (ros::ok())
      {
          // std::cout << "Nuitrack: Looping..." << std::endl;

          try
          {
              // Wait for new person tracking data
              tdv::nuitrack::Nuitrack::waitUpdate(skeletonTracker_);
          }
          catch (tdv::nuitrack::LicenseNotAcquiredException& e)
          {
              std::cerr << "LicenseNotAcquired exception (ExceptionType: "
                << e.type() << ")" << std::endl;
              break;
          }
          catch (const tdv::nuitrack::Exception& e)
          {
              std::cerr << "Nuitrack update failed (ExceptionType: "
                << e.type() << ")" << std::endl;
          }

          // std::cout << "Nuitrack: Sleeping..." << std::endl;
          ros::spinOnce();
          r.sleep();
      }

      // Release Nuitrack
      try
      {
          tdv::nuitrack::Nuitrack::release();
      }
      catch (const tdv::nuitrack::Exception& e)
      {
          std::cerr << "Nuitrack release failed (ExceptionType: "
            << e.type() << ")" << std::endl;
      }

    } // Run()
