#ifndef HOLEDETECTOR_PCL_H
#define HOLEDETECTOR_PCL_H

#include "sensor_msgs/PointCloud2.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <boost/thread/thread.hpp>

#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>

//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

class HoleDetector
{
public:
    HoleDetector();
    typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr      CLOUDPTR;
    typedef pcl::PointXYZRGB                            PXYZRGB;
    typedef pcl::PointIndices                           PNTIND;
    typedef pcl::PointIndices::Ptr                      PNTINDPTR;
    typedef pcl::ModelCoefficients::Ptr                 MODELCOEFFSPTR;

    void asusCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void planeDetection(CLOUDPTR cloud_in, PNTINDPTR inliers, MODELCOEFFSPTR coefficients);
    void cuttingX(CLOUDPTR cloud_input, CLOUDPTR cloud_out, double x_min, double x_max);
    void cuttingY(CLOUDPTR cloud_input, CLOUDPTR cloud_out, double y_min, double y_max);
    void cuttingZ(CLOUDPTR cloud_input, CLOUDPTR cloud_out, double z_min, double z_max);
    void extractIndices(CLOUDPTR cloud_in, CLOUDPTR cloud_out, PNTINDPTR inliers, bool set_negative);
    void euclidean_Clustering(CLOUDPTR input,std::vector<pcl::PointIndices> &cluster_indices, int min_size, int max_size, double tolerance);


};

#endif // HOLEDETECTOR_H
