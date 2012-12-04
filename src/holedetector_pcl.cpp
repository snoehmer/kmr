#include "holedetector_pcl.h"
#include "ros/message.h"
#include <ros/ros.h>

// PCL specific includes



typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr      CLOUDPTR;
typedef pcl::PointXYZRGB                            PXYZRGB;
typedef pcl::PointIndices                           PNTIND;
typedef pcl::PointIndices::Ptr                      PNTINDPTR;
typedef pcl::ModelCoefficients::Ptr                 MODELCOEFFSPTR;

//void cuttingZ(CLOUDPTR cloud_input, CLOUDPTR cloud_out, double z_min, double z_max);
//void planeDetection(CLOUDPTR cloud_in, PNTINDPTR inliers, MODELCOEFFSPTR coefficents);
//pcl::visualization::CloudViewer* viewer;


ros::Publisher pub;

HoleDetector::HoleDetector()
{
    ros::NodeHandle n;
    // Create a ROS publisher for the output point cloud
    pub = n.advertise<sensor_msgs::PointCloud2> ("holes", 1);
}

void HoleDetector::asusCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

    pcl::PointCloud<pcl::PointXYZRGB> ::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);

    std::cout << "ASUS-Callback!!!" << std::endl;
//    int rows = msg->height;
//    int cols = msg->width;
//    signed char* data = (signed char*)&(msg->data.front());
//    ROS_INFO("This is HoleDetector (asusCallback). Saving PointCloud2 data... rows: [%d] cols: [%d]", rows, cols);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    planeDetection(cloud, inliers, coefficients);

//    if(coefficients->values[3] < 0) coefficients->values[3]*=(-1);

    Eigen::Vector3f board_coeff(coefficients->values[0],//*coefficients->values[3],
                                coefficients->values[1],//*coefficients->values[3],
                                coefficients->values[2]);//*coefficients->values[3]);
    Eigen::Vector3f camCoord(0,0,1);

    float theta_angle;
    theta_angle = acos(camCoord.dot(board_coeff));
    if(theta_angle >= (M_PI)/2)
    {
    theta_angle =  theta_angle - M_PI;
    }
    if(theta_angle <=-M_PI/2)
    {
    theta_angle = M_PI - theta_angle;
    }
    std::cout << "RotationAngle: " << theta_angle << std::endl;

    Eigen::Vector3f rotation_axis;
    rotation_axis = board_coeff.cross(camCoord);
    rotation_axis.normalize();
    Eigen::Matrix3f m;
    m = Eigen::AngleAxisf(theta_angle,rotation_axis).toRotationMatrix();

    Eigen::Matrix4f rotor;
    rotor.block<3,3>(0,0) = m;
    for(int i=0; i<4; i++)
    {
        rotor(3,i) = 0.0;
        rotor(i,3) = 0.0;
    }
    rotor(3,3)= 1.0;

    std::cout << "Rotationsmatrix: " << rotor << std::endl;

    std::cout << "RotationAxis: " << rotation_axis << std::endl;

    pcl::transformPointCloud(*cloud, *cloud, rotor);
    std::cerr << "Cloud now rotated." << std::endl;

    // Calculate mean Z value (depth) of board
       double board_z_mean = coefficients->values[3];
       if(board_z_mean <= 0)
         board_z_mean = -board_z_mean;

       pcl::ModelCoefficients::Ptr coeff_after_rot (new pcl::ModelCoefficients);
       pcl::PointIndices::Ptr inliers_after_rot (new pcl::PointIndices);
       planeDetection(cloud, inliers_after_rot, coeff_after_rot);
//       pcl::PointCloud<pcl::PointXYZRGB>::Ptr sky = (new pcl::PointCloud<pcl::PointXYZRGB>);
//       extractIndices(cloud, sky, inliers_after_rot, false);

       // Cutting out the wallpart, other stuff is not of interest
       Eigen::Vector4f min_pt, max_pt;
       pcl::getMinMax3D(*cloud,*inliers_after_rot, min_pt, max_pt);
       std::cout << "Max_pt board: " << max_pt(0) << " " << max_pt(1) << " " << max_pt(2) << std::endl;
       std::cout << "Min_pt board: " << min_pt << std::endl;
       cuttingX(cloud, cloud,(min_pt(0)+0.02), (max_pt(0)-0.03));
       cuttingY(cloud, cloud, (min_pt(1)+0.02), (max_pt(1)-0.03));

       // Cutting off board, holes should be the only remaining parts.
       cuttingZ(cloud, cloud, 0.0, (board_z_mean+0.02));


        // Clustering to avoid artefacts.
        std::vector<pcl::PointIndices> indices;
        euclidean_Clustering(cloud,indices, 500, 15000,0.025); // 500-15000 points 2.5 cm maximum distance
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr hole (new pcl::PointCloud<pcl::PointXYZRGB>);
        PNTINDPTR hole_ind (new pcl::PointIndices);
        *hole_ind = indices[0];
        extractIndices(cloud, hole, hole_ind, false);

        // TODO: rotation in original direction necessary? DONE

        theta_angle = acos(board_coeff.dot(camCoord));
        if(theta_angle >= (M_PI)/2)
        {
        theta_angle =  theta_angle - M_PI;
        }
        if(theta_angle <=-M_PI/2)
        {
        theta_angle = M_PI - theta_angle;
        }
        std::cout << "RotationAngle: " << theta_angle << std::endl;

        rotation_axis = camCoord.cross(board_coeff);
        rotation_axis.normalize();
        m = Eigen::AngleAxisf(theta_angle,rotation_axis).toRotationMatrix();

        rotor.block<3,3>(0,0) = m;
        for(int i=0; i<4; i++)
        {
            rotor(3,i) = 0.0;
            rotor(i,3) = 0.0;
        }
        rotor(3,3)= 1.0;

        std::cout << "Rotationsmatrix: " << rotor << std::endl;

        std::cout << "RotationAxis: " << rotation_axis << std::endl;


        pcl::transformPointCloud(*hole, *hole, rotor);
        std::cerr << "Hole now rotated." << std::endl;


        pcl::getMinMax3D(*hole, min_pt, max_pt);
        Eigen::Vector3f center ;
        std::cout << "Calculating Center" << std::endl;
        center(0) = ((min_pt(0) + max_pt(0)) / 2);      // x - coord
        center(1) = ((min_pt(1) + max_pt(1)) / 2);      // y - coord
        center(2) = (coefficients->values[2] - 0.05);   // z - coord 5cm infront of the wall

        std::cout << "Hole Position from cam: " << center << std::endl;

        // TODO: return the center position



    sensor_msgs::PointCloud2 cloud_filtered;
    // Create massage from pointcloud
    pcl::toROSMsg(*hole, cloud_filtered);

    // Publish the data
    pub.publish (cloud_filtered);

//    cv::imwrite("depth.png", image);
}

void HoleDetector::planeDetection(CLOUDPTR cloud_in, PNTINDPTR inliers, MODELCOEFFSPTR coefficients)
{

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;

  std::cerr << "Start Plane detection." << std::endl;
  //Fit plane for Board
      // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);

  seg.setAxis(Eigen::Vector3f (0.0, 0.0, 1.0));
  seg.setEpsAngle(0.0000); //in radiant
  //seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  //seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_in);
  seg.segment (*inliers, *coefficients);

  std::cerr << " Board model coefficients: " << coefficients->values[0] << " "
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " "
                                        << coefficients->values[3] << std::endl;
}

void HoleDetector::cuttingX(CLOUDPTR cloud_input, CLOUDPTR cloud_out, double x_min, double x_max)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass_x;
     pass_x.setInputCloud (cloud_input);
     pass_x.setFilterFieldName ("x");
     pass_x.setFilterLimits (x_min, x_max);
     pass_x.setFilterLimitsNegative (false);
     pass_x.filter (*cloud_out);
}

void HoleDetector::cuttingY(CLOUDPTR cloud_input, CLOUDPTR cloud_out, double y_min, double y_max)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass_y;
     pass_y.setInputCloud (cloud_input);
     pass_y.setFilterFieldName ("y");
     pass_y.setFilterLimits (y_min, y_max);
     pass_y.setFilterLimitsNegative (false);
     pass_y.filter (*cloud_out);
}

void HoleDetector::cuttingZ(CLOUDPTR cloud_input, CLOUDPTR cloud_out, double z_min, double z_max)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass_z;
     pass_z.setInputCloud (cloud_input);
     pass_z.setFilterFieldName ("z");
     pass_z.setFilterLimits (z_min, z_max);
     pass_z.setFilterLimitsNegative (true);
     pass_z.filter (*cloud_out);
}

void HoleDetector::extractIndices(CLOUDPTR cloud_in, CLOUDPTR cloud_out, PNTINDPTR inliers, bool set_negative)
{
    //ROS_INFO("start extractIndices()");
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_in);
    extract.setIndices (inliers);
    extract.setNegative (set_negative);
    extract.filter (*cloud_out);
    //ROS_INFO("end extractIndices()");

}

//Euclidean Clustering
void HoleDetector::euclidean_Clustering(CLOUDPTR input,std::vector<pcl::PointIndices> &cluster_indices, int min_size, int max_size, double tolerance)
{
  pcl::search::KdTree<PXYZRGB>::Ptr tree (new pcl::search::KdTree<PXYZRGB> ());
  pcl::EuclideanClusterExtraction<PXYZRGB> ec;
  ec.setClusterTolerance (tolerance); // 2cm
  ec.setMinClusterSize (min_size);
  ec.setMaxClusterSize (max_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input);
  ec.extract (cluster_indices);

}
