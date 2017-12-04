#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>


#include <dynamic_reconfigure/server.h>
#include <cropping_pointcloud/croppingCloudConfig.h>


#include <pcl/io/ply_io.h>


typedef  pcl::PointXYZRGB PointType;


double x_min,x_max, y_min, y_max,z_min,z_max;



pcl::PointCloud<PointType>::Ptr cropped_cloud_ptr;
pcl::PointCloud<PointType>::Ptr received_cloud_ptr;
sensor_msgs::PointCloud2 cropped_cloud_msg, object_msg;
pcl::PointCloud<PointType>::Ptr object_cluster;
pcl::PointCloud<PointType>::Ptr no_plane_cloud;

void paramCallback(rmal::croppingCloudConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %f %f %f %f %f %f", config.x_min, config.x_max, config.y_min, config.y_max,config.z_min, config.z_max);
    x_min=config.x_min;
    x_max=config.x_max;
    y_min=config.y_min;
    y_max=config.y_max;
    z_min=config.z_min;
    z_max=config.z_max;
}



//sensor_msgs::PointCloud2 message to a sensor_msgs::PointCloud messa
//    sensor_msgs::convertPointCloud2ToPointCloud()

//Convert a sensor_msgs::PointCloud message to a sensor_msgs::PointCloud2 message.
//    sensor_msgs::convertPointCloudToPointCloud2.

void extractObject(pcl::PointCloud<PointType>::Ptr cropped_cloud_ptr)
{
    no_plane_cloud.reset(new pcl::PointCloud<PointType>);
    // Cloud indices representing planar components inliers
    pcl::PointIndices::Ptr planar_inliers (new pcl::PointIndices);
    // Cloud coefficients for planar components inliers
    pcl::ModelCoefficients::Ptr planar_coefficients (new pcl::ModelCoefficients);
    // Segmentation object
    pcl::SACSegmentation<PointType> SAC_filter;
    pcl::ExtractIndices<PointType> planar_inliers_extraction;
    // Euclidean Cluster Extraction object




    // Segmentation object initialization
    SAC_filter.setOptimizeCoefficients (true);
    SAC_filter.setModelType(pcl::SACMODEL_PLANE);
    SAC_filter.setMethodType (pcl::SAC_RANSAC);
    SAC_filter.setMaxIterations (100);
    SAC_filter.setDistanceThreshold (0.02);

    // Segment the dominant plane cluster
    SAC_filter.setInputCloud (cropped_cloud_ptr);
    SAC_filter.segment (*planar_inliers, *planar_coefficients);

    if (planar_inliers->indices.size () == 0)
    {
       return ;
    }

    // Remove the planar cluster from the input cloud
    planar_inliers_extraction.setInputCloud (cropped_cloud_ptr);
    planar_inliers_extraction.setIndices (planar_inliers);
    planar_inliers_extraction.setNegative (true);
    planar_inliers_extraction.filter (*no_plane_cloud);



    std::vector<int> no_Nan_vector;
    pcl::removeNaNFromPointCloud(*no_plane_cloud,*no_plane_cloud,no_Nan_vector);


/*
    // Segmentation object

    // KdTree object
    pcl::search::KdTree<PointType>::Ptr segmentation_tree (new pcl::search::KdTree<PointType>);
    // Euclidean Cluster Extraction object
    pcl::EuclideanClusterExtraction<PointType> cluster_exctraction;
    // Cluster indices vector
    std::vector<pcl::PointIndices> clusters_indices_vector;
    // Vector containing updated folders
    segmentation_tree->setInputCloud (no_plane_cloud);
    // Cluster extraction object initializations
    //    cluster_exctraction.setClusterTolerance (0.05);
    cluster_exctraction.setClusterTolerance (0.015  );

    cluster_exctraction.setMinClusterSize (50);
    cluster_exctraction.setSearchMethod (segmentation_tree);
    cluster_exctraction.setInputCloud (no_plane_cloud);

    // Cluster extraction
    // Each element of the vector <clusters_indices_vector> corresponds to a cluster of the cloud
    cluster_exctraction.extract (clusters_indices_vector);

    int number_of_clusters = clusters_indices_vector.size();

    //pcl::console::print_error ("\tdone\n");
//    std::cout << "\t" <<  number_of_clusters << " clusters found" << std::endl;



    pcl::PointIndices main_cluster=clusters_indices_vector.at(0);


    object_cluster.reset(new pcl::PointCloud<PointType>);

    // Loading of cluster informations and data from the original (segmented) cloud
    for (std::size_t i=0; i<main_cluster.indices.size();i++)
    {
        object_cluster->points.push_back (no_plane_cloud->points[main_cluster.indices.at(i)]);
    }
    object_cluster->width = object_cluster->points.size ();
    object_cluster->height = 1;
    object_cluster->is_dense = true;

*/

//    std::string file_name="spary.pcd";
//    pcl::io::savePCDFileASCII (file_name, *no_plane_cloud );




//    std::string file_name="spary.ply";
//    pcl::io::savePLYFile(file_name,*no_plane_cloud);



}


template
<typename PointT>
void croppingCloud(boost::shared_ptr<pcl::PointCloud<PointT> > input_cloud_ptr, boost::shared_ptr<pcl::PointCloud<PointT> > output_cloud_ptr,
double x_min, double x_max,double  y_min,double  y_max,double  z_min,double z_max)
{
    boost::shared_ptr<pcl::PointCloud<PointT> > boundingbox_ptr (new pcl::PointCloud<PointT> );



    PointT point;

     //x_min,y_min,z_min
    point.x = x_min;
    point.y = y_min;
    point.z = z_min;

    boundingbox_ptr->push_back(point);


    //x_min,y_min,z_max
    point.x = x_min;
    point.y = y_min;
    point.z = z_max;

    boundingbox_ptr->push_back(point);

    //x_min,y_max,z_min
    point.x = x_min;
    point.y = y_max;
    point.z = z_min;

    boundingbox_ptr->push_back(point);

    //x_min,y_max,z_max
    point.x = x_min;
    point.y = y_min;
    point.z = z_min;

    boundingbox_ptr->push_back(point);

    //x_max,y_min,z_min
    point.x = x_max;
    point.y = y_min;
    point.z = z_min;

    boundingbox_ptr->push_back(point);

    //x_max,y_min,z_max
    point.x = x_max;
    point.y = y_min;
    point.z = z_max;

    boundingbox_ptr->push_back(point);

    //x_max,y_max,z_min
    point.x = x_max;
    point.y = y_max;
    point.z = z_min;

    boundingbox_ptr->push_back(point);

    //x_max,y_max,z_max
    point.x = x_max;
    point.y = y_max;
    point.z = z_max;

    boundingbox_ptr->push_back(point);


    pcl::ConvexHull<PointT> hull;
    std::vector<pcl::Vertices> polygons;

    boost::shared_ptr<pcl::PointCloud<PointT> > surface_hull;

    try
    {
        hull.setInputCloud(boundingbox_ptr);
        hull.setDimension(3);

        surface_hull.reset(new pcl::PointCloud<PointT>);
        hull.reconstruct(*surface_hull, polygons);
        //    std::cout << surface_hull->size() << std::endl;

    }catch( std::exception e)
    {
        return;
    }



    try
    {
        pcl::CropHull<PointT> bb_filter;

        bb_filter.setDim(3);
        bb_filter.setNegative(false);
        bb_filter.setInputCloud(input_cloud_ptr);
        bb_filter.setHullIndices(polygons);
        bb_filter.setHullCloud(surface_hull);
        bb_filter.filter(*output_cloud_ptr.get());

    }
    catch( std::exception e )
    {
        return;
    }


}

template<>
void croppingCloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > input_cloud_ptr, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > output_cloud_ptr,
double x_min, double x_max,double  y_min,double  y_max,double  z_min,double z_max)
{
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > boundingbox_ptr (new pcl::PointCloud<pcl::PointXYZRGB> );
    pcl::PointXYZRGB point;
    uint8_t r = 255;
    uint8_t g = 0;
    uint8_t b = 0;
    int32_t rgb = (r << 16) | (g << 8) | b;
    point.rgb = *(float *)(&rgb); // makes the point red


    //x_min,y_min,z_min
    point.x = x_min;
    point.y = y_min;
    point.z = z_min;

    boundingbox_ptr->push_back(point);

    //x_min,y_min,z_max
    point.x = x_min;
    point.y = y_min;
    point.z = z_max;

    boundingbox_ptr->push_back(point);

    //x_min,y_max,z_min
    point.x = x_min;
    point.y = y_max;
    point.z = z_min;

    boundingbox_ptr->push_back(point);

    //x_min,y_max,z_max
    point.x = x_min;
    point.y = y_min;
    point.z = z_min;

    boundingbox_ptr->push_back(point);

    //x_max,y_min,z_min
    point.x = x_max;
    point.y = y_min;
    point.z = z_min;

    boundingbox_ptr->push_back(point);

    //x_max,y_min,z_max
    point.x = x_max;
    point.y = y_min;
    point.z = z_max;

    boundingbox_ptr->push_back(point);

    //x_max,y_max,z_min
    point.x = x_max;
    point.y = y_max;
    point.z = z_min;

    boundingbox_ptr->push_back(point);

    //x_max,y_max,z_max
    point.x = x_max;
    point.y = y_max;
    point.z = z_max;

    boundingbox_ptr->push_back(point);

    pcl::ConvexHull<pcl::PointXYZRGB> hull;
    hull.setInputCloud(boundingbox_ptr);
    hull.setDimension(3);
    std::vector<pcl::Vertices> polygons;

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > surface_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
    hull.reconstruct(*surface_hull, polygons);
    //    std::cout << surface_hull->size() << std::endl;



    pcl::CropHull<pcl::PointXYZRGB> bb_filter;

    bb_filter.setDim(3);
    bb_filter.setNegative(false);
    bb_filter.setInputCloud(input_cloud_ptr);
    bb_filter.setHullIndices(polygons);
    bb_filter.setHullCloud(surface_hull);
    bb_filter.filter(*output_cloud_ptr.get());

}




void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg)
{



    received_cloud_ptr.reset(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*pointcloud_msg.get(), *received_cloud_ptr.get());
//    std::cout<<"pointcloud_msg->header.frame_id: "<<pointcloud_msg->header.frame_id <<std::endl;




    cropped_cloud_ptr.reset(new pcl::PointCloud<PointType>);

    croppingCloud(received_cloud_ptr, cropped_cloud_ptr,x_max,x_min, y_max, y_min,z_max,z_min);

    pcl::toROSMsg(*cropped_cloud_ptr.get(),cropped_cloud_msg );
    extractObject(cropped_cloud_ptr);

    pcl::toROSMsg(*no_plane_cloud.get(),object_msg );

//    pcl::toROSMsg(*object_cluster.get(),object_msg );



}




int main(int argc, char** argv)
{
    ros::init(argc,argv,"croppingCloud");
    ros::NodeHandle nh;
//    std::string pointcloud_topic="/kinect2/depth_scaled/points";
//    std::string pointcloud_topic="/kinect2/depth_scaled_registered/points";
    //std::string pointcloud_topic="/kinect2/sd/points";
	//camera/rgb/points
	std::string pointcloud_topic="camera/rgb/points";
    ros::Subscriber sub_cloud= nh.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic,1 , pointcloudCallback);
    ros::Publisher marker_pub=nh.advertise<visualization_msgs::Marker>("visualization_marker",10);
    ros::Publisher pub_cropped_cloud=nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud",1);
    ros::Publisher pub_object_cloud=nh.advertise<sensor_msgs::PointCloud2>("object_cloud",1);




    dynamic_reconfigure::Server<rmal::croppingCloudConfig> server;
    dynamic_reconfigure::Server<rmal::croppingCloudConfig>::CallbackType f;

    f = boost::bind(&paramCallback, _1, _2);
    server.setCallback(f);

//    cropped_cloud_ptr->header.frame_id = "head_mount_kinect2_link";
    ros::Rate loop_rate(30);

//    x_min=-0.3;
//    x_max=0.3;
//    y_min=-1.0;
//    y_max=1.5;
//    z_min=-1;
//    z_max=1;

    while(ros::ok)
    {

        ros::spinOnce();
		std::string frame_id="openni_rgb_optical_frame";

//        object_msg.header.frame_id = "kinect2_ir_optical_frame";
//        cropped_cloud_msg.header.frame_id="kinect2_ir_optical_frame";

        object_msg.header.frame_id = frame_id;
        cropped_cloud_msg.header.frame_id=frame_id;

		
		
		
        pub_cropped_cloud.publish (cropped_cloud_msg);
        pub_object_cloud.publish (object_msg);

        loop_rate.sleep();
    }
    return 0;
}
