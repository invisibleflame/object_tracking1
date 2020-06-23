#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#define PCL_NO_PRECOMPILE
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <velodyne_pointcloud/point_types.h>
#include <stdlib.h>
#include <Eigen/Dense>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
using Eigen::MatrixXf;
using Eigen::JacobiSVD;
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
                                  (float, intensity, intensity1)
                                  (uint16_t, ring, ring)
                                  (uint16_t, label, label))

#define ClusterPointXYZIRL clustering::PointXYZIRL

pcl::PointCloud<ClusterPointXYZIRL>::Ptr cluster(new pcl::PointCloud<ClusterPointXYZIRL>());
std::vector<ClusterPointXYZIRL> centroid;


class Kmeans{
private:
	ros::NodeHandle nh;
	ros::Subscriber points_sub;
	ros::Publisher cluster_pub;

	int iter_no=100;

	void callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
	double distance(ClusterPointXYZIRL a , ClusterPointXYZIRL b);
	
public:
Kmeans();
};

Kmeans::Kmeans(){
	points_sub = nh.subscribe("/velodyne_points", 2, &Kmeans::callback,this);
	
	cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("/cluster",2);
}

double Kmeans::distance(ClusterPointXYZIRL a , ClusterPointXYZIRL b){

	double dis;
	double X_real = a.x - b.x;
	double Y_real = a.y - b.y; 
	double Z_real = a.z - b.z; 

	dis = (pow(X_real,2) + pow(Y_real,2)+ pow(Z_real,2));
	double d = sqrt(dis); 
	return d;
}

void Kmeans::callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

	int n=10;

	pcl::PointCloud<ClusterPointXYZIRL> laserCloudInput;
 	pcl::fromROSMsg(*laserCloudMsg, laserCloudInput);

 	ClusterPointXYZIRL point;
 	int j=0;
 	//choose n random points
 	for(int i=0;i<n;i++){
       
       j = rand()%(laserCloudInput.points.size());
       
       //verify whether j is already used or not
       point.x = laserCloudInput.points[j].x;
	   point.y = laserCloudInput.points[j].y;
	   point.z = laserCloudInput.points[j].z;
	   point.intensity = laserCloudInput.points[j].intensity;
	   point.ring = laserCloudInput.points[j].ring;
	   point.label = j;// 0 means uncluster

	   centroid.push_back(point);

 	}
 	
 	ClusterPointXYZIRL point1;
 	for(int i=0;i<iter_no;i++){
 	
 	// change cluster_id a/c to nearest centroid
 		cluster-> clear();
 		for(size_t k=0;i<laserCloudInput.points.size();k++){
 			double min_dist=200;		
 			point1.x = laserCloudInput.points[k].x;
	        point1.y = laserCloudInput.points[k].y;
	        point1.z = laserCloudInput.points[k].z;
	        point1.intensity = laserCloudInput.points[k].intensity;
	  		point1.ring = laserCloudInput.points[k].ring;
	       	int count=0;
	        for(int q=0;q<10;q++){

	        	double a = distance(point1,centroid[q]);
	        	if(a < min_dist){
					min_dist=a;	
	        		count=q;
	        	}
	        }
	    point1.label = count;
	   	cluster->points.push_back(point1);
	   	count=0;
 		}

 		
 	// Recalculating the center of each cluster
 		for(int q=0;q<10;q++){
 			float sum_x=0;
 			float sum_y=0;
 			float sum_z=0;
 			int num_points=0;
 		
 			
			for (size_t i = 0; i < laserCloudInput.points.size(); i++){

 				if( (laserCloudInput.points[i].label)==q ){
 					sum_x += laserCloudInput.points[i].x;
 					sum_y += laserCloudInput.points[i].y;
 					sum_z += laserCloudInput.points[i].z;
 					num_points += 1;

 				}
 			}
 			centroid[q].x=sum_x/num_points;
 			centroid[q].y=sum_y/num_points;
 			centroid[q].z=sum_z/num_points;
 		}



 	}

	sensor_msgs::PointCloud2 cluster_msg;
	pcl::toROSMsg(*cluster,cluster_msg);
	cluster_msg.header.stamp = laserCloudMsg->header.stamp;
    cluster_msg.header.frame_id = laserCloudMsg->header.frame_id;
    cluster_pub.publish(cluster_msg);

}

int main(int argc, char **argv)
{

    ros::init(argc, argv,"k_means_node");
    Kmeans node;
    ros::spin();
    return 0;
}