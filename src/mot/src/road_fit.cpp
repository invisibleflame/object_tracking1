#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#define PCL_NO_PRECOMPILE
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <velodyne_pointcloud/point_types.h>

#include <Eigen/Dense>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
using Eigen::MatrixXf;

using Eigen::MatrixXf;
using Eigen::VectorXf;
#define VPoint velodyne_pointcloud::PointXYZIR

namespace clustering
{
  struct PointXYZIRL
  {
    PCL_ADD_POINT4D;                    
    float    intensity;                 
    uint16_t ring;                      
    uint16_t label;                     
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     
  } EIGEN_ALIGN16;

};
POINT_CLOUD_REGISTER_POINT_STRUCT(clustering::PointXYZIRL,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (uint16_t, label, label))
#define ClusterPointXYZIRL clustering::PointXYZIRL

pcl::PointCloud<VPoint>::Ptr ground_points(new pcl::PointCloud<VPoint>());
pcl::PointCloud<VPoint>::Ptr road_points(new pcl::PointCloud<VPoint>());
pcl::PointCloud<VPoint>::Ptr lane_points(new pcl::PointCloud<VPoint>());
//pcl::PointCloud<VPoint>::Ptr ring_points(new pcl::PointCloud<VPoint>());

bool ring_comparison(VPoint a , VPoint b){
  return a.ring<b.ring;
};

bool x_comparison(VPoint a , VPoint b){
  return a.x<b.x;
};

class road3d{
public:
  road3d();
private:
  ros::NodeHandle nh;
  ros::Subscriber ground_points_sub;
  ros::Publisher road_points_pub;
  ros::Publisher lane_points_pub;
  ros::Publisher quadrant_pub;



  // Parameters for sensors and plane fit 
  void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& ground_msg);
  void road_extractor(const pcl::PointCloud<VPoint>& ring_points);

    //Compute normals  
  //   *model_normals=input_cloud;
  // }
  int t;
  float threshold_ang=100;

};

road3d::road3d()
{
  ground_points_sub = nh.subscribe("/points_ground", 2, &road3d::velodyne_callback,this);
  road_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_road",2);
  lane_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_lane",2);
  quadrant_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_Quadrant",2);
}

void road3d::road_extractor(const pcl::PointCloud<VPoint>& ring_points)
{ 
  int l =0;
  int r=0;
  std::cout<<"function properly called, fingers crossed"<<std::endl;
  std::cout<<ring_points.points.size()<<std::endl;
  bool s=true;
  for(size_t i=0;/*i<ring_points.size() &&*/ s;i++)
  {
    if(ring_points.points[i].x>-0.5 && ring_points.points[i].x<0.5)
    {
      t=i;
      s=false;
    }
  }
  std::cout<<t<<"ok"<<ring_points.size()<<std::endl;
  
  for(size_t j=t-1;j<ring_points.size();j++)
  {
    float D12 = sqrt(pow(ring_points.points[j+1].x- ring_points.points[j].x, 2) + pow(ring_points.points[j+1].y-ring_points.points[j].y, 2) + pow(ring_points.points[j+1].z-ring_points.points[j].z, 2));
    float D23 = sqrt(pow(ring_points.points[j+2].x-ring_points.points[j+1].x , 2) + pow(ring_points.points[j+2].y-ring_points.points[j+1].y , 2) + pow(ring_points.points[j+2].z-ring_points.points[j+1].z , 2));
    float D31 = sqrt(pow(ring_points.points[j+2].x-ring_points.points[j].x , 2) + pow(ring_points.points[j+2].y-ring_points.points[j].y , 2) + pow(ring_points.points[j+2].z-ring_points.points[j].z , 2));
    float Alpha = acos( (pow(D12,2) + pow(D23,2) - pow(D31,2))/ (2*D12*D23));
    //std::cout<<"hhh"<<Alpha<<std::endl;
    if(l<10){
      if(Alpha<threshold_ang or Alpha!= Alpha)
      {
        road_points->points.push_back(ring_points.points[j]);
        if(Alpha!=Alpha)l++;
        //std::cout<<t<<"okay"<<road_points.size()<<std::endl;
      }
      else
        break;
    }
    else{
      if(Alpha<threshold_ang)
      {
        road_points->points.push_back(ring_points.points[j]);
      }
      else
        break;
    }
  }
  std::cout<<t<<"okay"<<(*road_points).size()<<std::endl;
  std::cout<<ring_points.points[t].ring<<"qqqqq"<<std::endl;
  std::cout<<t-ring_points.size()<<"ppppp"<<std::endl;
    for(size_t j=t-2;j>0;j--)
  {
    float D12 = sqrt(pow(ring_points.points[j+1].x- ring_points.points[j].x, 2) + pow(ring_points.points[j+1].y-ring_points.points[j].y, 2) + pow(ring_points.points[j+1].z-ring_points.points[j].z, 2));
    float D23 = sqrt(pow(ring_points.points[j+2].x-ring_points.points[j+1].x , 2) + pow(ring_points.points[j+2].y-ring_points.points[j+1].y , 2) + pow(ring_points.points[j+2].z-ring_points.points[j+1].z , 2));
    float D31 = sqrt(pow(ring_points.points[j+2].x-ring_points.points[j].x , 2) + pow(ring_points.points[j+2].y-ring_points.points[j].y , 2) + pow(ring_points.points[j+2].z-ring_points.points[j].z , 2));
    float Alpha = acos( (pow(D12,2) + pow(D23,2) - pow(D31,2))/ (2*D12*D23));
    std::cout<<"fff"<<Alpha<<std::endl;
    
      if(r<70){
      if(Alpha<threshold_ang or Alpha!= Alpha)
      {
        road_points->points.push_back(ring_points.points[j]);
        if(Alpha!=Alpha)r++;
        //std::cout<<t<<"okay"<<road_points.size()<<std::endl;
      }
      else
        break;
    }
    else{
      if(Alpha<threshold_ang)
      {
        road_points->points.push_back(ring_points.points[j]);
      }
      else
        break;
    }
}}

void road3d::velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& ground_msg)
{
  pcl::PointCloud<VPoint>::Ptr ring_points(new pcl::PointCloud<VPoint>());
  ring_points->clear();
  std::cout<<"atleast call back works"<<std::endl;
  pcl::PointCloud<VPoint>::Ptr model_normals (new pcl::PointCloud<VPoint> ());
  pcl::PointCloud<VPoint> laserCloudInput;
  // conversion of velodyne point struct to pcl data point 
  pcl::fromROSMsg(*ground_msg, laserCloudInput);

  VPoint point;
  std::cout<<"I am your father, Luke"<<std::endl;
  
  for(size_t i=0;i<laserCloudInput.points.size();i++)
  {
      point.x = laserCloudInput.points[i].x;
      point.y = laserCloudInput.points[i].y;
      point.z = laserCloudInput.points[i].z;
      point.intensity = laserCloudInput.points[i].intensity;
      point.ring = laserCloudInput.points[i].ring;
      ground_points->points.push_back(point);
  }
  
  sort(ground_points->points.begin(), ground_points->points.end(),ring_comparison);
  std::cout<<"about to call the road road_extractor"<<std::endl;
  double pcloud_size = ground_points->points.size();
  int ring_number = ground_points->points[pcloud_size-1].ring;
  size_t l = 0;
  std::cout<<pcloud_size<<"hhh"<<std::endl;
  for(int k = 0; k<ring_number; k++)
  {
    while( l < pcloud_size)
    {
      if (ground_points->points[k].ring == k)
      {
        point.x = ground_points->points[l].x;
        point.y = ground_points->points[l].y;
        point.z = ground_points->points[l].z;
        point.ring = ground_points->points[l].ring;
        point.intensity = ground_points->points[l].intensity;
        ring_points->points.push_back(point);
        l++;
      }
      else
      {
        l++;
        break;
      }
    }
    std::cout<<(*ring_points).size()<<"ppp"<<std::endl;
    sort(ring_points->points.begin(), ring_points->points.end(),x_comparison);
    road_extractor(*(ring_points));
    
  }
  // Publish the points
  sensor_msgs::PointCloud2 road_msg;
  pcl::toROSMsg(*road_points,road_msg);
  road_msg.header.stamp = ground_msg->header.stamp;
  road_msg.header.frame_id = ground_msg->header.frame_id;
  road_points_pub.publish(road_msg);

  sensor_msgs::PointCloud2 quadrant_msg;
  pcl::toROSMsg(*ground_points,quadrant_msg);
  quadrant_msg.header.stamp = ground_msg->header.stamp;
  quadrant_msg.header.frame_id = ground_msg->header.frame_id;
  quadrant_pub.publish(quadrant_msg);


  road_points->clear();
  ground_points->clear();
  std::cout<<"I am Inevitable"<<std::endl;
}

int main(int argc, char **argv)
{
  std::cout<<"The Wands true owner wasn't Snape"<<std::endl;
  ros::init(argc, argv,"road3d_node");
  road3d node;
  ros::spin();
  return 0;
}