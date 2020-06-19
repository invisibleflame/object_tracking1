

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
using Eigen::JacobiSVD;
using Eigen::VectorXf;
std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > sourceClouds(100);

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



pcl::PointCloud<VPoint>::Ptr g_seeds_points(new pcl::PointCloud<VPoint>()); // after seed extraction 
pcl::PointCloud<VPoint>::Ptr g_ground_points(new pcl::PointCloud<VPoint>());
pcl::PointCloud<VPoint>::Ptr g_not_ground_points(new pcl::PointCloud<VPoint>());
// pcl::PointCloud<VPoint>::Ptr segment1(new pcl::PointCloud<VPoint>());
// pcl::PointCloud<VPoint>::Ptr segment2(new pcl::PointCloud<VPoint>());
// pcl::PointCloud<VPoint>::Ptr segment3(new pcl::PointCloud<VPoint>());
// pcl::PointCloud<VPoint>::Ptr segment4(new pcl::PointCloud<VPoint>());
// pcl::PointCloud<VPoint>::Ptr segment5(new pcl::PointCloud<VPoint>());
pcl::PointCloud<ClusterPointXYZIRL>::Ptr g_all_pc(new pcl::PointCloud<ClusterPointXYZIRL>());
std::vector < pcl::PointCloud<VPoint>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <VPoint>::Ptr > >  pg_seeds_points;
std::vector < pcl::PointCloud<VPoint>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <VPoint>::Ptr > >  pg_ground_points;
std::vector < pcl::PointCloud<VPoint>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <VPoint>::Ptr > >  pg_not_ground_points;
std::vector < pcl::PointCloud<ClusterPointXYZIRL>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <ClusterPointXYZIRL>::Ptr > > pg_all_pc;
//sorting points based on z points 
bool z_comparison(VPoint a , VPoint b){
	return a.z<b.z;
};

bool x_comparison(VPoint a , VPoint b){
	return a.x<b.x;
};
class plane_fit{
public:
	plane_fit();
private:
	ros::NodeHandle nh;
	ros::Subscriber points_sub;
	ros::Publisher ground_points_pub;
	ros::Publisher groudless_points_pub;
	ros::Publisher all_points_pub;

	// Parameters for sensors and plane fit 
	//CHeck once with papere

	double sensor_height = 3;
	int seg_no = 1; /// to be  implemented after tuning 
	int iter_no = 4;
	int lpr_no = 20;
	double th_seeds = 1.2;
	double th_dist = 0.5;
	int plane_parts=9;

	void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
	void estimate_plane(int p);
	void extract_initial_seeds(const pcl::PointCloud<VPoint>& p_sorted, int p);
	void copyPointCloud( const pcl::PointCloud<VPoint>::Ptr &model_normals ,const pcl::PointCloud<VPoint> input_cloud){
    //Compute normals  
	 	*model_normals=input_cloud;
  }

	float d; /// eqation parameter 
	// Normal  parameters
	MatrixXf normal; // current size 0x0;

	float th_dist_d;





};

	plane_fit::plane_fit(){
		// points_sub = nh.subscribe("/ns1/velodyne_points", 2, &plane_fit::velodyne_callback,this);
		points_sub = nh.subscribe("/velodyne_points", 2, &plane_fit::velodyne_callback,this);

		groudless_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_no_ground",2);
		ground_points_pub= nh.advertise<sensor_msgs::PointCloud2>("/points_ground",2);
		all_points_pub=nh.advertise<sensor_msgs::PointCloud2>("/all_points",2);

	}

	void plane_fit::extract_initial_seeds(const pcl::PointCloud<VPoint>& p_sorted, int p){
		double sum = 0;
		int count = 0;
		
		for(size_t i =0; i<p_sorted.points.size() ; i++){
			sum += p_sorted.points[i].z;
			count++;

		}
		//cout count;
		double lpr_height = count!=0?sum/count:0;
		pg_seeds_points[p]->clear();
		for(size_t i=0; i<p_sorted.points.size(); i++){
			if(p_sorted.points[i].z<lpr_height + th_seeds){
				pg_seeds_points[p]->points.push_back(p_sorted.points[i]);
			}
		}




	}

	void plane_fit::estimate_plane(int p){
		Eigen::Matrix3f covariance_matrix;
		Eigen::Vector4f points_mean;

		pcl::computeMeanAndCovarianceMatrix(*pg_ground_points[p], covariance_matrix, points_mean); /////Amazing Amazing amazing Amazing 
		JacobiSVD<MatrixXf> svd(covariance_matrix,Eigen::DecompositionOptions::ComputeFullU);

		////Check these function from Matrixf 
		normal = svd.matrixU().col(2);
		// A= UBUt 
		// last eigen vector will give normal 

		// check from paper once 

		Eigen::Vector3f seeds_mean = points_mean.head<3>();

		d= -(normal.transpose()*seeds_mean)(0,0);
		 th_dist_d = th_dist -d ;

		// Check for covariance 



	}

	void plane_fit::velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
		pcl::PointCloud<VPoint> laserCloudInput;
		pcl::PointCloud<VPoint>::Ptr model_normals (new pcl::PointCloud<VPoint> ());
		std::vector < pcl::PointCloud<VPoint>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <VPoint>::Ptr > >  parts;
		parts.reserve(laserCloudInput.points.size());
		std::vector < pcl::PointCloud<VPoint>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <VPoint>::Ptr > >  parts_original;
		parts_original.reserve(laserCloudInput.points.size());

		// conversion of velodyne point struct to pcl data point 
		pcl::fromROSMsg(*laserCloudMsg, laserCloudInput);

		pcl::PointCloud<VPoint> laserCloudInput_original;
		// conversion of velodyne point struct to pcl data point 
		pcl::fromROSMsg(*laserCloudMsg, laserCloudInput_original);
		ClusterPointXYZIRL point;
    	for(size_t i=0;i<laserCloudInput.points.size();i++){
	        point.x = laserCloudInput.points[i].x;
	        point.y = laserCloudInput.points[i].y;
	        point.z = laserCloudInput.points[i].z;
	        point.intensity = laserCloudInput.points[i].intensity;
	        point.ring = laserCloudInput.points[i].ring;
	        point.label = 0u;// 0 means uncluster
	        g_all_pc->points.push_back(point);
	    }
	    
	    std::vector < pcl::PointCloud<VPoint>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <VPoint>::Ptr > > sourceClouds;
	    pcl::PointCloud<VPoint>::Ptr cloudPTR(new pcl::PointCloud<VPoint>);
		*cloudPTR = laserCloudInput;
	    sourceClouds.push_back(cloudPTR);
	    //copyPointCloud(&(sourceClouds[0]),laserCloudInput);
	    std::cout<<"i am feeling amzing"<<" "<<sourceClouds[0]->points.size()<<" "<<laserCloudInput.points.size();
		// Sort on basis of z axis value to get initial seeds 

		sort(laserCloudInput.points.begin(), laserCloudInput.points.end(),z_comparison);


		pcl::PointCloud<VPoint>::iterator it = laserCloudInput.points.begin();
		for(size_t i =0;i<laserCloudInput.points.size();i++){
			if(laserCloudInput.points[i].z < -1.5*sensor_height){
				it++;
			}
			else {
				break;
			}
		}


		laserCloudInput.points.erase(laserCloudInput.points.begin(),it);

		// float min_x = laserCloudInput.points[0].x;
		// float max_x = laserCloudInput.points[0].x;

		// for (int i = 1; i < laserCloudInput.points.size (); ++i){
  //      		if(laserCloudInput.points[i].x <= min_x ){
  //         		min_x = laserCloudInput.points[i].x;
  //      		}
  //         	else if(laserCloudInput.points[i].x >= max_x ){
  //          		max_x = laserCloudInput.points[i].x;
  //          	}
  //       }
  //       std::cout << "Min x: " << min_x <<"\t";
  //  		std::cout << "Max x: " << max_x;

  //  		float variation= (max_x - min_x)/5;

  //  		for (int i = 1; i < laserCloudInput.points.size (); ++i){
  //      		if(laserCloudInput.points[i].x > min_x && laserCloudInput.points[i].x <= (min_x + variation)){
  //         		segment1->points.push_back(laserCloudInput.points[i]);
  //      		}
  //         	else if(laserCloudInput.points[i].x > (min_x + variation) && laserCloudInput.points[i].x <= (min_x + 2*variation)){
  //          		segment2->points.push_back(laserCloudInput.points[i]);
  //          	}
  //          	else if(laserCloudInput.points[i].x > (min_x + 2*variation) && laserCloudInput.points[i].x <= (min_x + 3*variation)){
  //          		segment3->points.push_back(laserCloudInput.points[i]);
  //          	}
  //          	else if(laserCloudInput.points[i].x > (min_x + 3*variation) && laserCloudInput.points[i].x <= (min_x + 4*variation)){
  //          		segment4->points.push_back(laserCloudInput.points[i]);
  //          	}
  //          	else if(laserCloudInput.points[i].x > (min_x + 4*variation) && laserCloudInput.points[i].x <= (min_x + 5*variation)){
  //          		segment5->points.push_back(laserCloudInput.points[i]);
  //          	}
  //       }
		sort(laserCloudInput.points.begin(), laserCloudInput.points.end(),x_comparison);
		int n=1;
		for(int s=0;s<plane_parts;s++){
			int u= (laserCloudInput.points.size())/9;
	    	for(int i=((n-1)*u);i<(n*u);i++){

	        	parts[s]->points.push_back(laserCloudInput.points[i]);
	        	parts_original[s]->points.push_back(laserCloudInput.points[i]);

	     //    (*parts[s]).points[i].x = laserCloudInput.points[i].x;
	     //    (*parts_original[s]).points[i].x = laserCloudInput.points[i].x;
	     //    (*parts[s]).points[i].y = laserCloudInput.points[i].y;
	     //    (*parts_original[s]).points[i].y = laserCloudInput.points[i].y;
	     //    (*parts[s]).points[i].z = laserCloudInput.points[i].z;
	     //    (*parts_original[s]).points[i].z = laserCloudInput.points[i].z;
	     //    (*parts[s]).points[i].intensity = laserCloudInput.points[i].intensity;
	     //    (*parts_original[s]).points[i].intensity = laserCloudInput.points[i].intensity;
	     //    (*parts[s]).points[i].ring = laserCloudInput.points[i].ring;
	     //    (*parts_original[s]).points[i].ring = laserCloudInput.points[i].ring;
			       
	    	}
	    	n++;

	    }

	    for(int i=0;i<plane_parts;i++){

		extract_initial_seeds(*parts[i], i);
		pg_ground_points[i]=pg_seeds_points[i];
		for(int k=0;k< iter_no ;k++){
			estimate_plane(i);
			pg_ground_points[i]->clear();
			pg_not_ground_points[i]->clear();

			Eigen::MatrixXf points((*parts_original[i]).points.size(),3);

		// extract_initial_seeds(laserCloudInput);
		// g_ground_points=g_seeds_points;
		// for(int i=0;i< iter_no ;i++){
		// 	estimate_plane();
		// 	g_ground_points->clear();
		// 	g_not_ground_points->clear();

		// 	Eigen::MatrixXf points(laserCloudInput_original.points.size(),3);

        	int j =0;
        	for(auto p:(*parts_original[i]).points){
            	points.row(j++)<<p.x,p.y,p.z;
        	}
			/// need to find distace of points from the plane 

			Eigen::VectorXf result = points*normal;

			for(int r=0;r<result.rows();r++){
				if(result[r]<th_dist_d){
					pg_all_pc[i]->points[r].label = 1u;// means ground
					pg_ground_points[i]->points.push_back((*parts_original[i])[r]);
				}
				else{
					pg_all_pc[i]->points[r].label = 0u;// means not ground and non clusterred
					pg_not_ground_points[i]->points.push_back((*parts_original[i])[r]);
				}
			}


		}
		}


		//concatenation
		for(int i=0; i<plane_parts; i++){
			for(size_t m=0; m<pg_ground_points[i]->points.size(); m++){
				g_ground_points->push_back(pg_ground_points[i]->points[m]);
			}
			for(size_t m=0; m<pg_not_ground_points[i]->points.size(); m++){
				g_not_ground_points->push_back(pg_not_ground_points[i]->points[m]);
			}
			for(size_t m=0; m<pg_all_pc[i]->points.size(); m++){		
				g_all_pc->push_back(pg_all_pc[i]->points[m]);
			}
		}

		// Publish the fucking points

		sensor_msgs::PointCloud2 ground_msg;
		pcl::toROSMsg(*g_ground_points,ground_msg);
		ground_msg.header.stamp = laserCloudMsg->header.stamp;
    	ground_msg.header.frame_id = laserCloudMsg->header.frame_id;
    	ground_points_pub.publish(ground_msg);

	    sensor_msgs::PointCloud2 groundless_msg;
	    pcl::toROSMsg(*g_not_ground_points, groundless_msg);
	    groundless_msg.header.stamp = laserCloudMsg->header.stamp;
	    groundless_msg.header.frame_id = laserCloudMsg->header.frame_id;
	    groudless_points_pub.publish(groundless_msg);
	    
	    sensor_msgs::PointCloud2 all_points_msg;
	    pcl::toROSMsg(*g_all_pc, all_points_msg);

	    all_points_msg.header.stamp = laserCloudMsg->header.stamp;
	    all_points_msg.header.frame_id = laserCloudMsg->header.frame_id;
	    all_points_pub.publish(all_points_msg);
	    g_all_pc->clear();




	}


int main(int argc, char **argv){
	std::cout<<"b1";
    ros::init(argc, argv,"plane_fit_node");
    plane_fit node;
    ros::spin();

    return 0;

}


/// Model parameter for ground is assumed to be  ax+ by +cz+d =0
//normal = a,b,c 
/// th_dist_d = threshold_dist -d ;

