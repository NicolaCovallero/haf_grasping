/*
 * David Fischinger - Vienna University of Technology
 * March 2015
 *
 * ** Grasp Action Client - for Testing **
 *
 * This node is serving as a client for the calculation of grasp points
 * based on the action server for grasp calculation.
 * This nodes subscribes to the topic /haf_grasping/depth_registered/single_cloud/points_in_lcs (lcs: local coordinate system)
 * and receives (single) point clouds (in a local coordinate system where z is perpendicular to the floor).
 * The node sends the point cloud as an action server goal and receives the grasp result.
 *
 */


//ROS includes
#include <ros/ros.h>
#include <ros/package.h>
#include "sensor_msgs/PointCloud2.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
//actionlib
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
//grasp action message
#include <haf_grasping/CalcGraspPointsServerAction.h>
// service messages
#include <haf_grasping/GraspSearchCenter.h>
#include <haf_grasping/GraspSearchRectangleSize.h>
#include <haf_grasping/GraspCalculationTimeMax.h>
#include <haf_grasping/GraspApproachVector.h>
#include <haf_grasping/ShowOnlyBestGrasp.h>
#include <haf_grasping/GraspPreGripperOpeningWidth.h>
// for reading pcd file
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl_ros/transforms.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h> // to get the minimum and maximum cooridnates of a cloud
#include <pcl_ros/transforms.h> // transform from a frame to another

// tf listener & datatypes
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// service for the segmentation
#include "iri_tos_supervoxels/object_segmentation.h"

// my messages
#include <iri_tos_supervoxels/segmented_objects.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <haf_grasping/Grasps.h>

#include <visualization_msgs/MarkerArray.h>

// parsing function
#include <string>     // std::string, std::stof


ros::NodeHandle* nh_pointer = NULL;
tf::TransformListener* pListener = NULL;
ros::ServiceClient client;
ros::Publisher grasp_poses_pub, grasp_poses_closing_direction_pub,
               grasps_data_pub, segmented_objects_pub;

//-------------------------------------------------
const std::string INPUT_TOPIC = "/camera/depth/points";
const std::string BASE_FRAME = "/world";
const double OPENING_WIDTH_SCALE = 1;

std::string input_topic, base_frame;
double opening_width_scale;// scale of the visualization of the closing direction markers
//------------------------------------------------

/** \struct to retireve in a compact way the grasping parametrrs
*/
struct grasp_result{

	float val,roll;
	geometry_msgs::Point gp1,gp2,gcp;
	geometry_msgs::Vector3 av;
	bool time;
};

class CCalcGrasppointsClient
{
public:
	ros::Subscriber objs_sub;								//subscriber for Objects
	ros::Subscriber pcd_sub;							//subscriber for path for pcd-file (to read)
	ros::ServiceServer srv_set_grasp_center;			// service to set new grasp center (center of rectangles where grasps are searched for)
	ros::ServiceServer srv_set_grasp_search_area_size;	// service to set size of rectangle where grasps are searched
	ros::ServiceServer srv_set_grasp_calculation_time_max;	// service to set maximal grasp calculation time (sec) before result is returned
	ros::ServiceServer srv_set_approach_vector;			// service to set approach vector for grasping (only direction hand is approaching, not roll angle)
	ros::ServiceServer srv_set_show_only_best_grasp;	// service to set show_only_best_grasp bool variable
	ros::ServiceServer srv_set_gripper_width;			// service to set factor f that is used for scaling point cloud to imitate pre-gripper opening width of 1/f
	geometry_msgs::Point graspsearchcenter;				// center for searching for grasps
	geometry_msgs::Vector3 approach_vector;				// defines the direction from where a grasp should be executed
	int grasp_search_size_x;		// [cm]	// the size (x direction) where grasps are really calculated (in each direction 7cm more are needed for feature calculation!
	int grasp_search_size_y;			// the size (y direction) where grasps are really calculated (in each direction 7cm more are needed for feature calculation!
	int max_grasp_search_size_x;	// x-limit for grasp search area size
	int max_grasp_search_size_y;	// y-limit for grasp search area size
	ros::Duration grasp_calculation_time_max;	//max time used for grasp calculation (sec) before result is returned
	bool show_only_best_grasp;
	std::string base_frame_default;
	int gripper_opening_width; 			//defines pre-grasp gripper opening width
	ros::NodeHandle nh_;
	grasp_result grasp_;

	bool get_grasp_cb(const sensor_msgs::PointCloud2& pc_in);
	void open_pcd_and_trig_get_grasp_cb(std_msgs::String pcd_path);
	bool set_grasp_center(haf_grasping::GraspSearchCenter::Request &req, haf_grasping::GraspSearchCenter::Response &res);
	bool set_grasp_search_area_size(haf_grasping::GraspSearchRectangleSize::Request &req, haf_grasping::GraspSearchRectangleSize::Response &res);
	bool set_grasp_calculation_time_max(haf_grasping::GraspCalculationTimeMax::Request &req, haf_grasping::GraspCalculationTimeMax::Response &res);
	bool set_approach_vector(haf_grasping::GraspApproachVector::Request &req, haf_grasping::GraspApproachVector::Response &res);
	bool set_show_only_best_grasp(haf_grasping::ShowOnlyBestGrasp::Request &req, haf_grasping::ShowOnlyBestGrasp::Response &res);
	bool set_gripper_width(haf_grasping::GraspPreGripperOpeningWidth::Request &req,	haf_grasping::GraspPreGripperOpeningWidth::Response &res);

	CCalcGrasppointsClient(ros::NodeHandle nh_)
	{
     	this->nh_ = nh_;

		this->graspsearchcenter.x = this->graspsearchcenter.y = this->graspsearchcenter.z = 0.0;	//default values
		
		
		this->approach_vector.x = this->approach_vector.y = 0.0;
		this->approach_vector.z = 1.0;
		
		//define size of grasp search rectangle (respectively take from launch file)
		this->max_grasp_search_size_x = 18;				//max. limit 32-14=18
		this->max_grasp_search_size_y = 30;				//max. limit 44-14=30
		
		// define maximal time before grasp result is returned
		int max_calculation_time = 50;
		this->grasp_calculation_time_max = ros::Duration(max_calculation_time);

		// define if only the best grasp should be visualized (respectively take bool value from launch file)
		nh_.param("show_only_best_grasp", this->show_only_best_grasp, true);

		// set default cloud frame (if cloud is generated from pcd)
		this->base_frame_default = base_frame;
		// set gripper opening with factor => 1/gripper opening width is tested
		nh_.param("gripper_width", this->gripper_opening_width, 1);


		//services for setting parameters
		this->srv_set_grasp_center = nh_.advertiseService("/haf_grasping/set_grasp_center", &CCalcGrasppointsClient::set_grasp_center,this);
		this->srv_set_grasp_search_area_size = nh_.advertiseService("/haf_grasping/set_grasp_search_area_size", &CCalcGrasppointsClient::set_grasp_search_area_size,this);
		this->srv_set_grasp_calculation_time_max = nh_.advertiseService("/haf_grasping/set_grasp_calculation_time_max", &CCalcGrasppointsClient::set_grasp_calculation_time_max,this);
		this->srv_set_approach_vector = nh_.advertiseService("/haf_grasping/set_approach_vector", &CCalcGrasppointsClient::set_approach_vector, this);
		this->srv_set_show_only_best_grasp = nh_.advertiseService("/haf_grasping/set_show_only_best_grasp", &CCalcGrasppointsClient::set_show_only_best_grasp, this);
		this->srv_set_gripper_width = nh_.advertiseService("/haf_grasping/set_gripper_opening_width", &CCalcGrasppointsClient::set_gripper_width, this);
	}
};

// define goal (input) for grasp calculation, send it to grasp action server and receive result
bool CCalcGrasppointsClient::get_grasp_cb(const sensor_msgs::PointCloud2& pc_in)
{
 
  //ROS_INFO("Objects received");
  //sensor_msgs::PointCloud2 pc_in = objs->objects[0].points;

	//this->srv_set_grasp_center = this->nh_.advertiseService("/haf_grasping/set_grasp_center", &CCalcGrasppointsClient::set_grasp_center,this);

	ROS_INFO("\nFrom calc_grasppoints_action_client: point cloud received");

	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<haf_grasping::CalcGraspPointsServerAction> ac("calc_grasppoints_svm_action_server", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	haf_grasping::CalcGraspPointsServerGoal goal;
	goal.graspinput.input_pc = 	pc_in;

	goal.graspinput.grasp_area_center = this->graspsearchcenter;

	//set grasp approach vector
	goal.graspinput.approach_vector = this->approach_vector;

	// set size of grasp search area
	goal.graspinput.grasp_area_length_x = this->grasp_search_size_x+14;
	goal.graspinput.grasp_area_length_y = this->grasp_search_size_y+14;

	// set max grasp calculation time
	goal.graspinput.max_calculation_time = this->grasp_calculation_time_max;

	// set if only best grasp should be visualized
	goal.graspinput.show_only_best_grasp = this->show_only_best_grasp;

	// set pre-grasp gripper opening width (factor for scaling pc)
	goal.graspinput.gripper_opening_width = this->gripper_opening_width;

	//send goal
	ac.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(50.0));

	// parsing the output
	double val,gp1x,gp1y,gp1z,gp2x,gp2y,gp2z,avx,avy,avz,gcpx,gcpy,gcpz,roll;


	if (finished_before_timeout)
	{
	    actionlib::SimpleClientGoalState state = ac.getState();
	    boost::shared_ptr<const haf_grasping::CalcGraspPointsServerResult_<std::allocator<void> > > result = ac.getResult();
	    //ROS_INFO("Result: %s", (*(result)).result.data.c_str());
	    ROS_INFO("Action finished: %s",state.toString().c_str());

    	//Note: the haf service returns a string, we have to parse it to get the pose
    	std::stringstream data((*(result)).result.data.c_str());
    	data >> val >> gp1x >> gp1y >> gp1z >> gp2x >> gp2y >> gp2z >> avx >> avy >> avz >> gcpx >> gcpy >> gcpz >> roll;	

    	std::cout << "Result: " << val << " "
							<< gp1x << " "
							<< gp1y << " "
							<< gp1z << " "
							<< gp2x << " "
							<< gp2y << " "
							<< gp2z << " "
							<< avx << " "
							<< avy << " "
							<< avz << " "
							<< gcpx << " "
							<< gcpy << " "
							<< gcpz << " "
							<< roll << " "
							<< std::endl;
	
		grasp_.val = val;
		grasp_.gp1.x = gp1x;grasp_.gp1.y = gp1y;grasp_.gp1.z = gp1z;
		grasp_.gp2.x = gp2x;grasp_.gp2.y = gp2y;grasp_.gp2.z = gp2z;
		grasp_.av.x = avx;grasp_.av.y = avy;grasp_.av.z = avz;
		grasp_.gcp.x = gcpx;grasp_.gcp.y = gcpy;grasp_.gcp.z = gcpz;
    grasp_.roll = roll;
    grasp_.time = true; //true: finished before timeout - false not, so it will be not considered as valid grasp
    //----------------------------
	}
	else
	{
	    ROS_INFO("Action did not finish before the time out.");
   	    grasp_.time = false;

	}


	return finished_before_timeout;
}


//set grasp search center
bool CCalcGrasppointsClient::set_grasp_center(haf_grasping::GraspSearchCenter::Request &req,
		haf_grasping::GraspSearchCenter::Response &res)
{
	//set grasp search center
	this->graspsearchcenter.x = req.graspsearchcenter.x;
	this->graspsearchcenter.y = req.graspsearchcenter.y;
	this->graspsearchcenter.z = req.graspsearchcenter.z;
	ROS_INFO("Set grasp search center to: x=%f, y=%f, z=%f", req.graspsearchcenter.x, req.graspsearchcenter.y, req.graspsearchcenter.z);
	res.result = true;
	ROS_INFO("sending back response: [%ld]", (long int)res.result);
	return res.result;
}

//set size of rectangle where grasps are searched
bool CCalcGrasppointsClient::set_grasp_search_area_size(haf_grasping::GraspSearchRectangleSize::Request &req,
		haf_grasping::GraspSearchRectangleSize::Response &res)
{
	//set grasp search rectangle size
	if (req.grasp_search_size_x >= 0 and req.grasp_search_size_x <= this->max_grasp_search_size_x){
		this->grasp_search_size_x = req.grasp_search_size_x;
		ROS_INFO("Set grasp rectangle size to: x=%ld", (long int)req.grasp_search_size_x);
	} else {
		ROS_INFO("Could not set grasp rectangle size for x. Allowed values: [0, %ld ]. Received value was: x=%ld", (long int)this->max_grasp_search_size_x,(long int)req.grasp_search_size_x);
		res.result = false;
		return res.result;
	}
	if (req.grasp_search_size_y >= 0 and req.grasp_search_size_y <= this->max_grasp_search_size_y){
		this->grasp_search_size_y = req.grasp_search_size_y;
		ROS_INFO("Set grasp rectangle size to: y=%ld", (long int)req.grasp_search_size_y);
	} else {
		ROS_INFO("Could not set grasp rectangle size for y. Allowed values: [0, %ld ]. Received value was: y=%ld", (long int)this->max_grasp_search_size_y,(long int)req.grasp_search_size_y);
		res.result = false;
		return res.result;
	}

	res.result = true;
	ROS_INFO("sending back response: [%ld]", (long int)res.result);
	return res.result;
}


//set maximal grasp calculation time before result has to be returned
bool CCalcGrasppointsClient::set_grasp_calculation_time_max(haf_grasping::GraspCalculationTimeMax::Request &req,
		haf_grasping::GraspCalculationTimeMax::Response &res)
{
	//set max grasp calculation time
	this->grasp_calculation_time_max = req.max_calculation_time;
	ROS_INFO("Set max calculation time (sec) to: x=%d", (int)req.max_calculation_time.toSec());
	res.result = true;
	ROS_INFO("sending back response: [%d]", (int)res.result);
	return res.result;
}

//set approach vector for approaching the object with gripper
bool CCalcGrasppointsClient::set_approach_vector(haf_grasping::GraspApproachVector::Request &req, haf_grasping::GraspApproachVector::Response &res)
{
	//set grasp approach vector
	this->approach_vector = req.approach_vector;
	ROS_INFO("Set approach vector to: [%f,%f,%f]", this->approach_vector.x,this->approach_vector.y,this->approach_vector.z);
	res.result = true;
	ROS_INFO("sending back response: [%d]", (int)res.result);
	return res.result;
}

//set show_only_best_grasp (for visualization)
bool CCalcGrasppointsClient::set_show_only_best_grasp(haf_grasping::ShowOnlyBestGrasp::Request &req,
		haf_grasping::ShowOnlyBestGrasp::Response &res)
{
	//set show_only_best_grasp
	this->show_only_best_grasp = req.show_only_best_grasp;
	ROS_INFO("Set show_only_best_grasp to: [%d] ", (int)this->show_only_best_grasp);
	res.result = true;
	ROS_INFO("sending back response: [%d]", (int)res.result);
	return res.result;
}

//set maximal grasp calculation time before result has to be returned
bool CCalcGrasppointsClient::set_gripper_width(haf_grasping::GraspPreGripperOpeningWidth::Request &req,
		haf_grasping::GraspPreGripperOpeningWidth::Response &res)
{
	//set pre-grasp gripper opening width
	this->gripper_opening_width = req.gripper_opening_width;
	ROS_INFO("Set gripper_opening_width (factor for scaling pc) to: x=%d", (int)req.gripper_opening_width);
	res.result = true;
	ROS_INFO("sending back response: [%d]", (int)res.result);
	return res.result;
}


/** \brief COmpute the qyaternion needed to rotate the vector v1 (fixed: (1,0,0) for markers)
* up to vector v2
*/
tf::Quaternion quaternionFromVector(tf::Vector3 v2) 
{
  // all the normalizations are important! except the alst one (with this method!)
  tf::Vector3 v1 (1,0,0); // this to make it works with ros
  v2.normalize();

  tf::Vector3 cross_vector = v1.cross(v2);
  cross_vector.normalize();
  
  double angle = acos(v1.dot(v2))/2;
  //check for nans values
  // this occures only when v2 has the same direction of v1 (v2.normalize()=v1.normalize())
  if(cross_vector.x() != cross_vector.x()) //this will be true only for nans value
  {
    cross_vector.setX(0);
    cross_vector.setY(0);
    cross_vector.setZ(0);
    // check if the sense of v2 is positive or negative
    if(v2.x()<0)
    { // this means that the sense has to eb negative,
      // and it is equal to a rotation in the y axis of M_PI
      angle = angle + M_PI;
      cross_vector.setY(1);
    } 
  }
  // Build quaternion
  tf::Quaternion quatern; 
  quatern.setX(cross_vector.x() * sin(angle));
  quatern.setY(cross_vector.y() * sin(angle));
  quatern.setZ(cross_vector.z() * sin(angle));
  quatern.setW(cos(angle));  
  quatern.normalize(); 

  return quatern;
}

visualization_msgs::Marker grasp_closing_direction_marker(double length_line, int seq,
																			  geometry_msgs::Point gp1,
																			  geometry_msgs::Point gp2,
																			  geometry_msgs::Point gcp)
{
	visualization_msgs::Marker marker;

	marker.header.frame_id = base_frame;
	marker.header.stamp = ros::Time::now();
	marker.header.seq = seq;
	char str[20];
	sprintf(str,"closing_direction_%d",seq); 
	marker.ns = str;
	marker.action = visualization_msgs::Marker::ADD;
	marker.id = 0; 
	marker.type = visualization_msgs::Marker::CUBE;
	marker.color.b = 1.0;
    marker.color.a = 1.0; 

    double opening_width = sqrt(pow((gp1.x - gp2.x),2) + pow((gp1.z - gp2.z),2) + pow((gp1.z - gp2.z),2)); 

    marker.scale.x = opening_width * opening_width_scale;
    marker.scale.y = 0.002;
    marker.scale.z = 0.002;

    marker.lifetime = ros::Duration(10.0);

    marker.pose.position.x = gcp.x;
    marker.pose.position.y = gcp.y;
    marker.pose.position.z = gcp.z;

    geometry_msgs::Vector3 diff_vector;
    diff_vector.x = gp1.x-gcp.x;
    diff_vector.y = gp1.y-gcp.y;
    diff_vector.z = gp1.z-gcp.z;

    tf::Vector3 v2(diff_vector.x,diff_vector.y,diff_vector.z);
	tf::Quaternion quat_tf = quaternionFromVector(v2);
	marker.pose.orientation.x = quat_tf.x();
	marker.pose.orientation.y = quat_tf.y();
	marker.pose.orientation.z = quat_tf.z();
	marker.pose.orientation.w = quat_tf.w();

	return marker;
}

/** \brief function to construct the grasp message
*
*/
haf_grasping::Grasp buildGraspMsg(grasp_result& grasp_pose)
{
	haf_grasping::Grasp msg;

	msg.val = grasp_pose.val;
	msg.roll = grasp_pose.roll;
	msg.approach_vector = grasp_pose.av;
	msg.gcp = grasp_pose.gcp;
	msg.gp1 = grasp_pose.gp1;
	msg.gp2 = grasp_pose.gp2;

	return msg;
}

//void detectGrasp( const iri_tos_supervoxels::segmented_objectsConstPtr& msg)
void detectGrasp(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	
	//ROS_INFO("\n #### %d Objects received ####\n",(int)msg->objects.size());
	//now we pass all the objects and for each one create a haf_client
  ROS_INFO("Segmenting the point cloud");

  iri_tos_supervoxels::object_segmentation srv;
  srv.request.point_cloud = *cloud_msg; 

  iri_tos_supervoxels::segmented_objects msg;
  if(client.call(srv))
  {
    msg = srv.response.objects; 
    //segmented_objects_pub.publish(msg); 
  }
  else
  {
    ROS_ERROR("Impossible to segment the point cloud");
    return;
  }

	//---------- Messages  To publish --------------
	geometry_msgs::PoseArray grasp_poses;
	visualization_msgs::MarkerArray closing_direction_markers;
	haf_grasping::Grasps grasps_data_msg;
  //---------------------------------------------	

    CCalcGrasppointsClient grasp_client(*nh_pointer);
	for (int i = 0; i < msg.objects.size(); ++i)
	{
    //convert the pointcloud to the base frame
    sensor_msgs::PointCloud2 tmp_cloud_msg;
    tmp_cloud_msg = msg.objects[i];
    // we now fake the time stamp of the point cloud, if we don't do this the 
    // transformation will give "looking for past transform" error
    tmp_cloud_msg.header.stamp = ros::Time::now();
    if(!pcl_ros::transformPointCloud(base_frame,tmp_cloud_msg,tmp_cloud_msg,*pListener))
    {
      ROS_ERROR("Error transforming the point cloud.");
    }

	  // compute the limits for the object
    pcl::PointCloud<pcl::PointXYZRGBA> tmp;
		pcl::fromROSMsg(tmp_cloud_msg,tmp);
		pcl::PointXYZRGBA max,min;
		pcl::getMinMax3D(tmp,min,max);//compute the maximum and minimum coordinates

	  //defining the grasp center
	  geometry_msgs::Point graspsearchcenter;
	  graspsearchcenter.x = (max.x + min.x)/2;
	  graspsearchcenter.y = (max.y + min.y)/2;
	  graspsearchcenter.z = 0.0;

	  grasp_client.graspsearchcenter = graspsearchcenter;
	  grasp_client.grasp_search_size_x = (max.x - min.x) * 1.5 * 100 + 4;//[* 100] is to convert from meter to cm
	  grasp_client.grasp_search_size_y = (max.y - min.y) * 1.5 * 100 + 4;// 4 bias
	  
	  grasp_client.show_only_best_grasp = true;

	  ROS_INFO("Computing the grasp of object labeled as: %d",i);
	  bool success = grasp_client.get_grasp_cb(tmp_cloud_msg);

	  //bool success = grasp_client.get_grasp_cb(msg->objects[i].points);
	  if(success)
	  {
	  	ROS_INFO("Found grasp for object: %d",i);
	  }
	  else
	  	ROS_ERROR("Not possible grasp for object: %d",i);

	  std::cout << std::endl;

	  grasp_result grasp_pose = grasp_client.grasp_;
	  if(grasp_pose.time) //if it has finished before time out we consider it as valid
	  {
	  	tf::Vector3 v2(grasp_pose.av.x,grasp_pose.av.y,-grasp_pose.av.z);
	  	tf::Quaternion quat_tf = quaternionFromVector(v2);
	  	geometry_msgs::Quaternion quat;
	  	quat.x = quat_tf.x();quat.y = quat_tf.y();quat.z = quat_tf.z();quat.w = quat_tf.w();
	  	geometry_msgs::Pose pose;
	  	std::cout << quat_tf.x() << " " << quat_tf.y() << " " << quat_tf.z() << " " << quat_tf.w() << " "
	  			  << grasp_pose.gp2.x << " " << grasp_pose.gp2.y << " " << grasp_pose.gp2.x << std::endl;		
	   	pose.orientation = quat;
	   	pose.position.x = grasp_pose.gcp.x;
	   	pose.position.y = grasp_pose.gcp.y;
	   	pose.position.z = grasp_pose.gcp.z;

	   	grasp_poses.poses.push_back(pose);
	  	closing_direction_markers.markers.push_back(grasp_closing_direction_marker(0.4,i,grasp_pose.gp1,grasp_pose.gp2,grasp_pose.gcp));

	  	grasps_data_msg.grasps.push_back(buildGraspMsg(grasp_pose));

	  }
	}
  
	grasp_poses.header.seq = 0;
	grasp_poses.header.stamp = ros::Time::now();
	grasp_poses.header.frame_id = base_frame;
	grasp_poses_pub.publish(grasp_poses);

	grasp_poses_closing_direction_pub.publish(closing_direction_markers);
	grasps_data_pub.publish(grasps_data_msg);

}

int main (int argc, char **argv)
{
  ROS_INFO("ROS NODE calc_grasppoints_client_simulation started");
  ros::init(argc, argv, "calc_grasppoints_client_sim");
  nh_pointer = new(ros::NodeHandle);

  nh_pointer->param("input_topic_haf",input_topic,INPUT_TOPIC);
  std::cout << "\n\n input topic: " << input_topic << "\n\n";

  nh_pointer->param("opening_width_scale_marker",opening_width_scale,OPENING_WIDTH_SCALE);
  std::cout << "opening_width_scale: " << opening_width_scale << "\n";  

  nh_pointer->param("base_frame", base_frame, BASE_FRAME);

  ros::Subscriber sub = nh_pointer->subscribe<sensor_msgs::PointCloud2>(input_topic, 1,detectGrasp);
  //ros::Subscriber sub = nh_pointer->subscribe<iri_tos_supervoxels::segmented_objects>(input_topic, 1,detectGrasp);
  
  grasp_poses_pub = nh_pointer->advertise<geometry_msgs::PoseArray>("haf_grasping/grasp_poses",1);
  grasp_poses_closing_direction_pub = nh_pointer->advertise<visualization_msgs::MarkerArray>("haf_grasping/grasp_poses_closing_direction",1);
  grasps_data_pub = nh_pointer->advertise<haf_grasping::Grasps>("haf_grasping/grasps_data",1);
  segmented_objects_pub = nh_pointer->advertise<sensor_msgs::PointCloud2>("/segmented_objects",1);

  client = nh_pointer->serviceClient<iri_tos_supervoxels::object_segmentation>("/iri_tos_supervoxels_alg/object_segmentation");


  pListener = new (tf::TransformListener);

  ros::spin();
  return 0;
}


