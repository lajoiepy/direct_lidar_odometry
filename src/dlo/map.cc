/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include "dlo/map.h"

std::atomic<bool> dlo::MapNode::abort_(false);


/**
 * Constructor
 **/

dlo::MapNode::MapNode() : Node("dlo_map_node") {

  this->getParams();

  // this->abort_timer = this->nh.createTimer(ros::Duration(0.01), &dlo::MapNode::abortTimerCB, this);

  // if (this->publish_full_map_){
  //   this->publish_timer = this->nh.createTimer(ros::Duration(this->publish_freq_), &dlo::MapNode::publishTimerCB, this);
  // }
  
  this->keyframe_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("keyframes", 1, std::bind(&dlo::MapNode::keyframeCB, this, std::placeholders::_1));
  this->map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 1);

  this->save_pcd_srv = this->create_service<direct_lidar_odometry::srv::SavePCD>("save_pcd", std::bind(&dlo::MapNode::savePcd, this, std::placeholders::_1, std::placeholders::_2));

  // initialize map
  this->dlo_map = std::make_shared<pcl::PointCloud<PointType>>();

  // ROS_INFO("DLO Map Node Initialized");

}


/**
 * Destructor
 **/

dlo::MapNode::~MapNode() {}


/**
 * Get Params
 **/

void dlo::MapNode::getParams() {

  this->declare_parameter<std::string>("~dlo/odomNode/odom_frame", "odom");
  this->declare_parameter<bool>("~dlo/mapNode/publishFullMap", true);
  this->declare_parameter<double>("~dlo/mapNode/publishFreq", 1.0);
  this->declare_parameter<double>("~dlo/mapNode/leafSize", 0.5);

  this->get_parameter("~dlo/odomNode/odom_frame", this->odom_frame);
  this->get_parameter("~dlo/mapNode/publishFullMap", this->publish_full_map_);
  this->get_parameter("~dlo/mapNode/publishFreq", , this->publish_freq_);
  this->get_parameter("~dlo/mapNode/leafSize", this->leaf_size_);

  // // Get Node NS and Remove Leading Character
  // std::string ns = ros::this_node::getNamespace();
  // ns.erase(0,1);

  // // Concatenate Frame Name Strings
  // this->odom_frame = ns + "/" + this->odom_frame;

}


/**
 * Start Map Node
 **/

void dlo::MapNode::start() {
  // ROS_INFO("Starting DLO Map Node");
}


// /**
//  * Stop Map Node
//  **/

// void dlo::MapNode::stop() {
//   ROS_WARN("Stopping DLO Map Node");

//   // shutdown
//   ros::shutdown();
// }


// /**
//  * Abort Timer Callback
//  **/

// void dlo::MapNode::abortTimerCB(const ros::TimerEvent& e) {
//   if (abort_) {
//     stop();
//   }
// }


/**
 * Publish Timer Callback
 **/

// void dlo::MapNode::publishTimerCB(const ros::TimerEvent& e) {

//   if (this->dlo_map->points.size() == this->dlo_map->width * this->dlo_map->height) {
//     sensor_msgs::PointCloud2 map_ros;
//     pcl::toROSMsg(*this->dlo_map, map_ros);
//     map_ros.header.stamp = ros::Time::now();
//     map_ros.header.frame_id = this->odom_frame;
//     this->map_pub.publish(map_ros);
//   }
  
// }


/**
 * Node Callback
 **/

void dlo::MapNode::keyframeCB(const sensor_msgs::PointCloud2ConstPtr& keyframe) {

  // convert scan to pcl format
  pcl::PointCloud<PointType>::Ptr keyframe_pcl = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*keyframe, *keyframe_pcl);

  // voxel filter
  this->voxelgrid.setLeafSize(this->leaf_size_, this->leaf_size_, this->leaf_size_);
  this->voxelgrid.setInputCloud(keyframe_pcl);
  this->voxelgrid.filter(*keyframe_pcl);

  // save keyframe to map
  this->map_stamp = keyframe->header.stamp;
  *this->dlo_map += *keyframe_pcl;

  if (!this->publish_full_map_) {
    if (keyframe_pcl->points.size() == keyframe_pcl->width * keyframe_pcl->height) {
      sensor_msgs::PointCloud2 map_ros;
      pcl::toROSMsg(*keyframe_pcl, map_ros);
      map_ros.header.stamp = this->now();
      map_ros.header.frame_id = this->odom_frame;
      this->map_pub.publish(map_ros);
    }
  }

}

bool dlo::MapNode::savePcd(direct_lidar_odometry::save_pcd::Request& req,
                           direct_lidar_odometry::save_pcd::Response& res) {

  pcl::PointCloud<PointType>::Ptr m =
    pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>(*this->dlo_map));

  float leaf_size = req.leaf_size;
  std::string p = req.save_path;

  std::cout << std::setprecision(2) << "Saving map to " << p + "/dlo_map.pcd" << "... "; std::cout.flush();

  // voxelize map
  pcl::VoxelGrid<PointType> vg;
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.setInputCloud(m);
  vg.filter(*m);

  // save map
  int ret = pcl::io::savePCDFileBinary(p + "/dlo_map.pcd", *m);
  res.success = ret == 0;

  if (res.success) {
    std::cout << "done" << std::endl;
  } else {
    std::cout << "failed" << std::endl;
  }

  return res.success;

}
