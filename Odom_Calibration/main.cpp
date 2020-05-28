#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <nav_core/recovery_behavior.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>

#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "sensor_msgs/LaserScan.h"
#include "stdio.h"
#include "boost/asio.hpp"   //include boost library function
#include "boost/bind.hpp"
#include "math.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//pcl

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/registration.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>

//Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "../include/calib_odom/Odom_Calib.hpp"

#include <csm/csm_all.h>

using namespace std;
using namespace boost::asio;// for IO operation
//using namespace karto;

// odom calibration instance
OdomCalib Odom_calib;


std::vector<geometry_msgs::PointStamped> mcu_path;

Eigen::Vector3d  cal_delta_distence(Eigen::Vector3d odom_pose);

class Scan2
{
public:
    Scan2();

    //variable for PI-ICP
    LDP m_prevLDP;
    sm_params m_PIICPParams;
    sm_result m_OutputResult;

    //odom & scan　for pose integration
    Eigen::Vector3d scan_pos_cal;
    Eigen::Vector3d odom_pos_cal;

    //store odom increment between two frame
    std::vector<Eigen::Vector3d> odom_increments;          

    std::string odom_frame_;
    std::string base_frame_;

    ros::NodeHandle node_;
    tf::TransformListener tf_;

    ros::Subscriber calib_flag_sub_;

    ros::Publisher odom_path_pub_,scan_path_pub_,calib_path_pub_;

    nav_msgs::Path path_odom,path_scan;
    ros::Time current_time;

    //time synchronization
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;

    void CalibFlagCallBack(const std_msgs::Empty &msg);

    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr&scan2);

    //tf tree check odom pose
    bool getOdomPose(Eigen::Vector3d& pose, const ros::Time& t);

    //pub odom & laser path
    void pub_msg( Eigen::Vector3d& pose,nav_msgs::Path &path,ros::Publisher &mcu_path_pub_);

    //for publishing correct path
    void publishPathEigen(std::vector<Eigen::Vector3d>& path_eigen,ros::Publisher& path_pub_);

    //functions for pl-icp
    void SetPIICPParams();
    void LaserScanToLDP(sensor_msgs::LaserScan *pScan,
                                 LDP& ldp);
    Eigen::Vector3d  PIICPBetweenTwoFrames(LDP& currentLDPScan,
                                           Eigen::Vector3d tmprPose);


};


Eigen::Vector3d now_pos,last_pos;

void Scan2::pub_msg( Eigen::Vector3d& pose,nav_msgs::Path &path,ros::Publisher &mcu_path_pub_)
{

    current_time = ros::Time::now();
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = pose(0);
    this_pose_stamped.pose.position.y = pose(1);

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(pose(2));
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id="odom";
    path.poses.push_back(this_pose_stamped);
    mcu_path_pub_.publish(path);

}

void Scan2::publishPathEigen(std::vector<Eigen::Vector3d>& path_eigen,ros::Publisher& path_pub_)
{
    nav_msgs::Path visual_path;

    current_time = ros::Time::now();

    visual_path.header.stamp = ros::Time::now();
    visual_path.header.frame_id="odom";

    geometry_msgs::PoseStamped tmpPose;
    tmpPose.header.stamp = current_time;
    tmpPose.header.frame_id="odom";

    for(int i = 0 ;i < path_eigen.size();i++)
    {
        Eigen::Vector3d poseEigen = path_eigen[i];

        tmpPose.pose.position.x = poseEigen(0);
        tmpPose.pose.position.y = poseEigen(1);

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(poseEigen(2));
        tmpPose.pose.orientation.x = goal_quat.x;
        tmpPose.pose.orientation.y = goal_quat.y;
        tmpPose.pose.orientation.z = goal_quat.z;
        tmpPose.pose.orientation.w = goal_quat.w;

        visual_path.poses.push_back(tmpPose);
    }

    path_pub_.publish(visual_path);
}

// get robot pose in odom frame at time t
bool Scan2::getOdomPose(Eigen::Vector3d& pose, const ros::Time& t)
{
    // Get the robot's pose
    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                               tf::Vector3(0,0,0)), t, base_frame_);
    tf::Stamped<tf::Transform> odom_pose;
    try
    {
        tf_.transformPose(odom_frame_, ident, odom_pose);
    }


    catch(tf::TransformException e)
    {
        ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
        return false;
    }
    double yaw = tf::getYaw(odom_pose.getRotation());



    pose << odom_pose.getOrigin().x(),
            odom_pose.getOrigin().y(),
            yaw;
    //pub_msg(pose,path_odom,odom_path_pub_);
    return true;
}

Scan2::Scan2()
{
    ros::NodeHandle private_nh_("~");

    m_prevLDP = NULL;
    SetPIICPParams();

    scan_pos_cal.setZero();
    odom_pos_cal.setZero();
    odom_increments.clear();

    if(!private_nh_.getParam("odom_frame", odom_frame_))
        odom_frame_ = "odom";
    if(!private_nh_.getParam("base_frame", base_frame_))
        base_frame_ = "base_link";

    // callback for calculating least square
    calib_flag_sub_ = node_.subscribe("calib_flag",5,&Scan2::CalibFlagCallBack,this);

    //pub path
    odom_path_pub_ = node_.advertise<nav_msgs::Path>("odom_path_pub_",1,true);
    scan_path_pub_ = node_.advertise<nav_msgs::Path>("scan_path_pub_",1,true);
    calib_path_pub_ = node_.advertise<nav_msgs::Path>("calib_path_pub_",1,true);
    current_time = ros::Time::now();

    path_odom.header.stamp=current_time;
    path_scan.header.stamp=current_time;
    path_odom.header.frame_id="odom";
    path_scan.header.frame_id="odom";

    //odom and scan data synchronization
    scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "/sick_scan", 10);
    scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 10);
    scan_filter_->registerCallback(boost::bind(&Scan2::scanCallBack, this, _1));

    std::cout <<"Calibration Online,Wait for Data!!!!!!!"<<std::endl;
}

void Scan2::CalibFlagCallBack(const std_msgs::Empty &msg)
{
    Eigen::Matrix3d correct_matrix = Odom_calib.Solve();

    Eigen::Matrix3d tmp_transform_matrix;

    std::cout<<"correct_matrix:"<<std::endl<<correct_matrix<<std::endl;

    //calculate path after calibration
    Eigen::Vector3d calib_pos(0,0,0);                 //pose after calibration
    std::vector<Eigen::Vector3d> calib_path_eigen;    //path after calibration
    for(int i = 0; i < odom_increments.size();i++)
    {
        Eigen::Vector3d odom_inc = odom_increments[i];
        Eigen::Vector3d correct_inc = correct_matrix * odom_inc;

        tmp_transform_matrix << cos(calib_pos(2)),-sin(calib_pos(2)),0,
                            sin(calib_pos(2)), cos(calib_pos(2)),0,
                                            0,                 0,1;

        calib_pos += tmp_transform_matrix * correct_inc;

        calib_path_eigen.push_back(calib_pos);
    }

    //pub calibrated path
    publishPathEigen(calib_path_eigen,calib_path_pub_);
    
    //finish calibration
    scan_filter_sub_->unsubscribe();

    std::cout <<"calibration over!!!!"<<std::endl;
}


void Scan2::scanCallBack(const sensor_msgs::LaserScan::ConstPtr &_laserScanMsg)
{
    static long int dataCnt = 0;
    sensor_msgs::LaserScan scan;
    Eigen::Vector3d odom_pose;              //odom pose corresponding to scan
    Eigen::Vector3d d_point_odom;           //dpose from odom
    Eigen::Vector3d d_point_scan;           //dpose from scanmatch
    Eigen::MatrixXd transform_matrix(3,3);  //

    double c,s;
    scan = *_laserScanMsg;

    //get odom pose corresponding to scan
    if(!getOdomPose(odom_pose, _laserScanMsg->header.stamp))
        return ;

    //pose increment between two frame of odom
    d_point_odom = cal_delta_distence(odom_pose);

    if(d_point_odom(0) < 0.05 &&
       d_point_odom(1) < 0.05 &&
       d_point_odom(2) < tfRadians(5.0))
    {
        return ;
    }
    last_pos = now_pos;

    odom_increments.push_back(d_point_odom);

    LDP currentLDP;
    if(m_prevLDP != NULL)
    {
        LaserScanToLDP(&scan,currentLDP);
        d_point_scan = PIICPBetweenTwoFrames(currentLDP,d_point_odom);
    }
    else
    {
        LaserScanToLDP(&scan,m_prevLDP);
    }

    // for laser_path visualization
    c = cos(scan_pos_cal(2));
    s = sin(scan_pos_cal(2));
    transform_matrix<<c,-s,0,
                      s, c,0,
                      0, 0,1;
    scan_pos_cal+=(transform_matrix*d_point_scan);

    // for odom_path visualization
    c = cos(odom_pos_cal(2));
    s = sin(odom_pos_cal(2));
    transform_matrix<<c,-s,0,
                      s, c,0,
                      0, 0,1;
    odom_pos_cal+=(transform_matrix*d_point_odom);

    //for visualization
    pub_msg(odom_pos_cal,path_odom,odom_path_pub_);
    pub_msg(scan_pos_cal,path_scan,scan_path_pub_);

    // construct overdetermined equation
    Odom_calib.Add_Data(d_point_odom,d_point_scan);
    dataCnt++;

    std::cout <<"Data Cnt:"<<dataCnt<<std::endl;
}

//get relative pose between two frame
Eigen::Vector3d  cal_delta_distence(Eigen::Vector3d odom_pose)
{

    Eigen::Vector3d d_pos;  //return value
    now_pos = odom_pose;

    Eigen::Matrix3d TOL;
    Eigen::Matrix3d TOC;
    TOC << cos(now_pos(2)), -sin(now_pos(2)), now_pos(0),
           sin(now_pos(2)),  cos(now_pos(2)), now_pos(1),
              0,          0,        1;
    TOL << cos(last_pos(2)), -sin(last_pos(2)), last_pos(0),
        sin(last_pos(2)),  cos(last_pos(2)), last_pos(1),
            0,          0,        1;
    Eigen::Matrix3d TLC = TOL.inverse() * TOC;
    d_pos << TLC(0, 2), TLC(1, 2), atan2(TLC(1, 0), TLC(0, 0));

    return d_pos;
}

//ser PI-ICP parameter
void Scan2::SetPIICPParams()
{
    m_PIICPParams.min_reading = 0.1;
    m_PIICPParams.max_reading = 20;

    m_PIICPParams.max_angular_correction_deg = 20.0;
    m_PIICPParams.max_linear_correction = 1;

    m_PIICPParams.max_iterations = 50;
    m_PIICPParams.epsilon_xy = 0.000001;
    m_PIICPParams.epsilon_theta = 0.0000001;

    m_PIICPParams.max_correspondence_dist = 1;
    m_PIICPParams.sigma = 0.01;
    m_PIICPParams.use_corr_tricks = 1;

    m_PIICPParams.restart = 0;
    m_PIICPParams.restart_threshold_mean_error = 0.01;
    m_PIICPParams.restart_dt = 1.0;
    m_PIICPParams.restart_dtheta = 0.1;

    m_PIICPParams.clustering_threshold = 0.2;

    m_PIICPParams.orientation_neighbourhood = 10;

    m_PIICPParams.use_point_to_line_distance = 1;

    m_PIICPParams.do_alpha_test = 0;
    m_PIICPParams.do_alpha_test_thresholdDeg = 5;

    m_PIICPParams.outliers_maxPerc = 0.9;
    m_PIICPParams.outliers_adaptive_order = 0.7;
    m_PIICPParams.outliers_adaptive_mult = 2.0;

    m_PIICPParams.do_visibility_test = 1;
    m_PIICPParams.outliers_remove_doubles = 1;
    m_PIICPParams.do_compute_covariance = 0;
    m_PIICPParams.debug_verify_tricks = 0;
    m_PIICPParams.use_ml_weights = 0;
    m_PIICPParams.use_sigma_weights = 0;
}



//convert scan to data for PI-ICP
void Scan2::LaserScanToLDP(sensor_msgs::LaserScan *pScan,
                           LDP& ldp)
{
    int nPts = pScan->intensities.size();
    ldp = ld_alloc_new(nPts);

    for(int i = 0;i < nPts;i++)
    {
        double dist = pScan->ranges[i];
        if(dist > 0.1 && dist < 20)
        {
            ldp->valid[i] = 1;
            ldp->readings[i] = dist;
        }
        else
        {
            ldp->valid[i] = 0;
            ldp->readings[i] = -1;
        }
        ldp->theta[i] = pScan->angle_min+pScan->angle_increment*i;
    }
    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[nPts-1];

    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = 0.0;

    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;
}


//get pose matching from icp
Eigen::Vector3d  Scan2::PIICPBetweenTwoFrames(LDP& currentLDPScan,
                                              Eigen::Vector3d tmprPose)
{
    m_prevLDP->odometry[0] = 0.0;
    m_prevLDP->odometry[1] = 0.0;
    m_prevLDP->odometry[2] = 0.0;

    m_prevLDP->estimate[0] = 0.0;
    m_prevLDP->estimate[1] = 0.0;
    m_prevLDP->estimate[2] = 0.0;

    m_prevLDP->true_pose[0] = 0.0;
    m_prevLDP->true_pose[1] = 0.0;
    m_prevLDP->true_pose[2] = 0.0;

    //set configure parameter
    m_PIICPParams.laser_ref = m_prevLDP;
    m_PIICPParams.laser_sens = currentLDPScan;

    m_PIICPParams.first_guess[0] = tmprPose(0);
    m_PIICPParams.first_guess[1] = tmprPose(1);
    m_PIICPParams.first_guess[2] = tmprPose(2);

    m_OutputResult.cov_x_m = 0;
    m_OutputResult.dx_dy1_m = 0;
    m_OutputResult.dx_dy2_m = 0;

    sm_icp(&m_PIICPParams,&m_OutputResult);

    //nowPose in the frame of lastPose
    Eigen::Vector3d  rPose;
    if(m_OutputResult.valid)
    {
        //relative pose from two frame
        rPose(0)=(m_OutputResult.x[0]);
        rPose(1)=(m_OutputResult.x[1]);
        rPose(2)=(m_OutputResult.x[2]);

//        std::cout <<"Iter:"<<m_OutputResult.iterations<<std::endl;
//        std::cout <<"Corr:"<<m_OutputResult.nvalid<<std::endl;
//        std::cout <<"Erro:"<<m_OutputResult.error<<std::endl;

//        std::cout <<"PI ICP GOOD"<<std::endl;
    }
    else
    {
        std::cout <<"PI ICP Failed!!!!!!!"<<std::endl;
        rPose = tmprPose;
    }

    //ld_free(m_prevLDP);

    m_prevLDP = currentLDPScan;

    return rPose;
}



int main(int argc,char** argv)
{

    ros::init(argc, argv, "message_filter_node");
    ros::Time::init();
    ros::NodeHandle n;
    Scan2 scan;


    Odom_calib.Set_data_len(12000);
    Odom_calib.set_data_zero();
    ros::spin();


    return 0;
}
