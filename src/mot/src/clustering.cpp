


#include <iostream>
#include <forward_list>
// For disable PCL complile lib, to use PointXYZIR, and customized pointcloud    
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <velodyne_pointcloud/point_types.h>



#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include <pcl/common/common.h>
#include <pcl/common/centroid.h>



using namespace std;

//Customed Point Struct for holding clustered points
namespace scan_line_run
{
  
  struct PointXYZIRL
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    uint16_t label;                     ///< point label
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

}; // namespace scan_line_run

#define SLRPointXYZIRL scan_line_run::PointXYZIRL
#define VPoint velodyne_pointcloud::PointXYZIR
#define RUN pcl::PointCloud<SLRPointXYZIRL>
// Register custom point struct according to PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(scan_line_run::PointXYZIRL,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (uint16_t, label, label))

#define dist(a,b) sqrt(((a).x-(b).x)*((a).x-(b).x)+((a).y-(b).y)*((a).y-(b).y))


class ScanLineRun{
public:
    ScanLineRun();
private:
    ros::NodeHandle node_handle_;
    ros::Subscriber points_node_sub_;
    ros::Publisher cluster_points_pub_;
    ros::Publisher ring_points_pub_;

    std::string point_topic_;

    int sensor_model_ = 32 ;
    double th_run_ = 0.7;// thresold of distance of points belong to the same run.
    double th_merge_ =1;// threshold of distance of runs to be merged.

    // For organization of points.
    std::vector<std::vector<SLRPointXYZIRL> > laser_frame_;
    std::vector<SLRPointXYZIRL> laser_row_;
    
    std::vector<std::forward_list<SLRPointXYZIRL*> > runs_;// For holding all runs.
    uint16_t max_label_;// max run labels, for disinguish different runs.
    std::vector<std::vector<int> > ng_idx_;// non ground point index.

    // Call back funtion.
    void velodyne_callback_(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
    // For finding runs on a scanline.
    void find_runs_(int scanline);
    // For update points cluster label after merge action.
    void update_labels_(int scanline);
    // For merge `current` run to `target` run.
    void merge_runs_(uint16_t cur_label, uint16_t target_label);
    

    int smart_idx_(int local_idx, int n_i, int n_j, bool inverse);


    // For display markers only, however, currently the orientation is bad.
    ros::Publisher marker_array_pub_;

    // Dummy object to occupy idx 0.
    std::forward_list<SLRPointXYZIRL*> dummy_;

    

};    


ScanLineRun::ScanLineRun(){


    // Init Ptrs with vectors
    for(int i=0;i<sensor_model_;i++){
        std::vector<int> dummy_vec;
        ng_idx_.push_back(dummy_vec);
    }

    // Init LiDAR frames with vectors and points
    SLRPointXYZIRL p_dummy;
    p_dummy.intensity = -1;// Means unoccupy by any points
    laser_row_ = std::vector<SLRPointXYZIRL>(2251, p_dummy);
    laser_frame_ = std::vector<std::vector<SLRPointXYZIRL> >(32, laser_row_);
    
    // Init runs, idx 0 for interest point, and idx for ground points
    max_label_ = 1;
    runs_.push_back(dummy_);
    runs_.push_back(dummy_);

   
    // Subscriber to velodyne topic
    points_node_sub_ = node_handle_.subscribe("/all_points", 2, &ScanLineRun::velodyne_callback_, this);
    
    // Publisher Init
    std::string cluster_topic;
  
    cluster_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2 >("/clusterpointcloud", 10);
 

    marker_array_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("/cluster_ma", 10);


    

}


void ScanLineRun::find_runs_(int scan_line){

    int point_size = ng_idx_[scan_line].size();
    if(point_size<=0)
        return;

    int non_g_pt_idx = ng_idx_[scan_line][0]; // The first non ground point
    int non_g_pt_idx_l = ng_idx_[scan_line][point_size - 1]; // The last non ground point

    for(int i_idx=0;i_idx<point_size-1;i_idx++){
        int i = ng_idx_[scan_line][i_idx];
        int i1 = ng_idx_[scan_line][i_idx+1];

        if(i_idx == 0){
            // The first point, make a new run.
            auto &p_0 = laser_frame_[scan_line][i];
            max_label_ += 1;
            runs_.push_back(dummy_);
            laser_frame_[scan_line][i].label = max_label_;
            runs_[p_0.label].insert_after(runs_[p_0.label].cbefore_begin(), &laser_frame_[scan_line][i]);

            if(p_0.label == 0)
                ROS_ERROR("p_0.label == 0");
        }

        // Compare with the next point
        auto &p_i = laser_frame_[scan_line][i];
        auto &p_i1 = laser_frame_[scan_line][i1];

        // If next point is ground point, skip.
        if(p_i1.label == 1u){
            // Add to ground run `runs_[1]`
            runs_[p_i1.label].insert_after(runs_[p_i1.label].cbefore_begin(), &laser_frame_[scan_line][i1]);
            continue;
        }

       
        if(p_i.label != 1u && dist(p_i,p_i1) < th_run_){
            p_i1.label = p_i.label; 
        }else{
            max_label_ += 1;
            p_i1.label = max_label_;
            runs_.push_back(dummy_);
        }

        // Insert the index.
        runs_[p_i1.label].insert_after(runs_[p_i1.label].cbefore_begin(), &laser_frame_[scan_line][i1]);
        
        if(p_i1.label == 0)
            ROS_ERROR("p_i1.label == 0");    
    }
    
    // Compare the last point and the first point, for laser scans is a ring.
    if(point_size>1){
        auto &p_0 = laser_frame_[scan_line][non_g_pt_idx];
        auto &p_l = laser_frame_[scan_line][non_g_pt_idx_l];

        // Skip, if one of the start point or the last point is ground point.
        if(p_0.label == 1u || p_l.label == 1u){
            return ;
        }else if(dist(p_0,p_l) < th_run_){
            if(p_0.label==0){
                ROS_ERROR("Ring Merge to 0 label");
            }
            /// If next point is within threshold, then merge it into the same run.
            merge_runs_(p_l.label, p_0.label);
        }
    }else if(point_size == 1){
            // The only point, make a new run.
            auto &p_0 = laser_frame_[scan_line][non_g_pt_idx];
            max_label_ += 1;
            runs_.push_back(dummy_);
            laser_frame_[scan_line][non_g_pt_idx].label = max_label_;
            runs_[p_0.label].insert_after(runs_[p_0.label].cbefore_begin(), &laser_frame_[scan_line][non_g_pt_idx]);
    }
    
}


void ScanLineRun::update_labels_(int scan_line){
    // Iterate each point of this scan line to update the labels.
    int point_size_j_idx = ng_idx_[scan_line].size();
    // Current scan line is emtpy, do nothing.
    if(point_size_j_idx==0) return;

    // Iterate each point of this scan line to update the labels.
    for(int j_idx=0;j_idx<point_size_j_idx;j_idx++){
        int j = ng_idx_[scan_line][j_idx];

        auto &p_j = laser_frame_[scan_line][j];

        // Runs above from scan line 0 to scan_line
        for(int l=scan_line-1;l>=0;l--){
            if(ng_idx_[l].size()==0)
                continue;

            // Smart index for the near enough point, after re-organized these points.
            int nn_idx = j;

            if(laser_frame_[l][nn_idx].intensity ==-1 || laser_frame_[l][nn_idx].label == 1u){
                continue;
            }

            // Nearest neighbour point
            auto &p_nn = laser_frame_[l][nn_idx];
            // Skip, if these two points already belong to the same run.
            if(p_j.label == p_nn.label){
                continue;
            }
            double dist_min = dist(p_j, p_nn);

            /* Otherwise,
            If the distance of the `nearest point` is within `th_merge_`, 
            then merge to the smaller run.
            */
            if(dist_min < th_merge_){
                uint16_t  cur_label = 0, target_label = 0;

                if(p_j.label ==0 || p_nn.label==0){
                    ROS_ERROR("p_j.label:%u, p_nn.label:%u", p_j.label, p_nn.label);
                }
                // Merge to a smaller label cluster
                if(p_j.label > p_nn.label){
                    cur_label = p_j.label;
                    target_label = p_nn.label;
                }else{
                    cur_label = p_nn.label;
                    target_label = p_j.label;
                }

                // Merge these two runs.
                merge_runs_(cur_label, target_label);
            }
        }
    }

}

void ScanLineRun::merge_runs_(uint16_t cur_label, uint16_t target_label){
    if(cur_label ==0||target_label==0){
        ROS_ERROR("Error merging runs cur_label:%u target_label:%u", cur_label, target_label);
    }
    // First, modify the label of current run.
    for(auto &p:runs_[cur_label]){
        p->label = target_label;
    }
    // Then, insert points of current run into target run.
    runs_[target_label].insert_after(runs_[target_label].cbefore_begin(), runs_[cur_label].begin(),runs_[cur_label].end() );
    runs_[cur_label].clear();
}

[[deprecated("Not useful in my case.")]] 
int ScanLineRun::smart_idx_(int local_idx, int n_i, int n_j, bool inverse=false){
    if(inverse==false){
        // In case of zero-divide.
        if(n_i == 0 ) return 0;
        float rate = (n_j*1.0f)/n_i;
        int idx = floor(rate*local_idx);

        // In case of overflow
        if(idx>n_j){
            idx = n_j>1?n_j-1:0;
        }
        return idx;
    }else{
        // In case of zero-divide.
        if(n_j == 0 ) return 0;
        float rate = (n_i*1.0f)/n_j;
        int idx = ceil(rate*local_idx);

        // In case of overflow
        if(idx>n_i){
            idx = n_i>1?n_i-1:0;
        }
        return idx;
    }
    
}



visualization_msgs::Marker mark_cluster(pcl::PointCloud<SLRPointXYZIRL>::Ptr cloud_cluster, 
    std::string ns ,int id, float r, float g, float b) { 
  Eigen::Vector4f centroid; 
  Eigen::Vector4f min; 
  Eigen::Vector4f max; 
  
  pcl::compute3DCentroid (*cloud_cluster, centroid); 
  pcl::getMinMax3D (*cloud_cluster, min, max); 
  
  uint32_t shape = visualization_msgs::Marker::CUBE; 
  visualization_msgs::Marker marker; 
  marker.header.frame_id = "/velodyne"; 
  marker.header.stamp = ros::Time::now(); 
  
  marker.ns = ns; 
  marker.id = id; 
  marker.type = shape; 
  marker.action = visualization_msgs::Marker::ADD; 
  
  marker.pose.position.x = centroid[0]; 
  marker.pose.position.y = centroid[1]; 
  marker.pose.position.z = centroid[2]; 
  marker.pose.orientation.x = 0.0; 
  marker.pose.orientation.y = 0.0; 
  marker.pose.orientation.z = 0.0; 
  marker.pose.orientation.w = 1.0; 
  
  marker.scale.x = (max[0]-min[0]); 
  marker.scale.y = (max[1]-min[1]); 
  marker.scale.z = (max[2]-min[2]); 
  
  if (marker.scale.x ==0) 
      marker.scale.x=0.1; 

  if (marker.scale.y ==0) 
    marker.scale.y=0.1; 

  if (marker.scale.z ==0) 
    marker.scale.z=0.1; 
    
  marker.color.r = r; 
  marker.color.g = g; 
  marker.color.b = b; 
  marker.color.a = 0.5; 

  marker.lifetime = ros::Duration(0.1); 

  return marker; 
}





void ScanLineRun::velodyne_callback_(const sensor_msgs::PointCloud2ConstPtr& in_cloud_msg){
    // Msg to pointcloud
    pcl::PointCloud<SLRPointXYZIRL> laserCloudIn;
    pcl::fromROSMsg(*in_cloud_msg, laserCloudIn);

    /// Clear and init.
    // Clear runs in the previous scan.
    max_label_ = 1;
    if(!runs_.empty()){
        runs_.clear();
        runs_.push_back(dummy_);// dummy for index `0`
        runs_.push_back(dummy_);// for ground points
    }
    
    // Init laser frame.
    SLRPointXYZIRL p_dummy;
    p_dummy.intensity = -1;
    laser_row_ = std::vector<SLRPointXYZIRL> (2251, p_dummy);
    laser_frame_ = std::vector<std::vector<SLRPointXYZIRL> >(32, laser_row_);
    
    // Init non-ground index holder.
    for(int i=0;i<sensor_model_;i++){
        ng_idx_[i].clear();
    }

    double range = 0;
    int row = 0;
    for(auto &point:laserCloudIn.points){
        if(point.ring<sensor_model_&&point.ring>=0){
            

            // Compute and angle. 
            // @Note: In this case, `x` points right and `y` points forward.
            range = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
            if(point.x>=0){
                row = int(563 - asin(point.y/range)/0.00279111);
            }else if(point.x<0 && point.y <=0){
                row = int(1688 + asin(point.y/range)/0.00279111);
            }else {
                row = int(1688 + asin(point.y/range)/0.00279111);
            }

            if(row>2250||row<0){
                ROS_ERROR("Row: %d is out of index.", row);
                return;
            }else{
                laser_frame_[point.ring][row] = point;
            }
            
            if(point.label != 1u){
                ng_idx_[point.ring].push_back(row);
            }else{
                runs_[1].insert_after(runs_[1].cbefore_begin(), &point);
            }
        } 
    }


    // Main processing
    for(int i=0;i<sensor_model_;i++){
        // get runs on current scan line i
        find_runs_(i);
        update_labels_(i);
    }
    

    // Extract Clusters
    // re-organize scan-line points into cluster point cloud
    pcl::PointCloud<SLRPointXYZIRL>::Ptr laserCloud(new pcl::PointCloud<SLRPointXYZIRL>());
    pcl::PointCloud<SLRPointXYZIRL>::Ptr clusters(new pcl::PointCloud<SLRPointXYZIRL>());
    pcl::PointCloud<SLRPointXYZIRL>::Ptr vectorcloud(new pcl::PointCloud<SLRPointXYZIRL>());



  
    visualization_msgs::MarkerArray ma;

    int cnt = 0;
    
    // Re-organize pointcloud clusters for PCD saving or publish
#ifdef INTEREST_ONLY
    for(size_t i=0;i<1;i++){
#else
    for(size_t i=2;i<runs_.size();i++){
#endif  
        //pcl::PointCloud<SLRPointXYZIRL>::Ptr dummycloud(new pcl::PointCloud<SLRPointXYZIRL>());
        pcl::PointCloud<SLRPointXYZIRL> dummycloud;
        
        if(!runs_[i].empty()){
            cnt++;
            
            int ccnt = 0;
            int tub=0;

            // adding run current for publishing
            
            for(auto &p:runs_[i]){
                // Reorder the label id
                                
                p->label = cnt;
                if(p->x<20&&p->x>-20 ) vectorcloud->points.push_back(*p);
                tub = ccnt;
                ccnt++;
                //dummycloud->points.push_back(*p);
                dummycloud.points.push_back(*p);


                // clusters->points.push_back(*p);

            } 

            int to=0;
            //std::cout<<"bazingaaaaa    "<<dummycloud.points<<endl;
            // if(dummycloud.points.size()<100){
                for(int ui=0;ui<dummycloud.points.size();ui++){
                    if(dummycloud.points[ui].x<10&&dummycloud.points[ui].x>-10& dummycloud.points[ui].y<10&&dummycloud.points[ui].y>-10) {laserCloud->points.push_back(dummycloud.points[ui]); to++;}

                    //std::cout<<"life sucks so does my code  "<<dummycloud.points[ui].z<<endl;
                }
            // }

            
            	Eigen::Vector4f centre;
            
            pcl::compute3DCentroid (*vectorcloud, centre);
            //std::cout<<centre[0];
            //std::cout<<"life sucks so does my code  "<<vectorcloud->points.size()<<endl;
            if(vectorcloud->points.size()>50){
            ma.markers.push_back(mark_cluster(vectorcloud,"box", cnt, 1.0f,1.0f, 1.0f));}
            //}
            //std::cout<<"Cape Crusader  "<<vectorcloud->points.size()<<"Fuck Superman  "<<to<<endl;
            std::cout<<"Cape Crusader  " <<vectorcloud->points.size()-to<<endl;

            vectorcloud->clear();

 
   // clusters->clear();
        }
    }
    ROS_INFO("Total cluster: %d", cnt);
    // Publish Cluster Points
    if(laserCloud->points.size()>0){
        sensor_msgs::PointCloud2 cluster_msg;
        pcl::toROSMsg(*laserCloud, cluster_msg);
        cluster_msg.header.frame_id = "/velodyne";
        cluster_points_pub_.publish(cluster_msg);

        marker_array_pub_.publish(ma);
    }
    

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ScanLineRun");
    ScanLineRun node;
    ros::spin();

    return 0;

}