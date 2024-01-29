#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/registration/icp.h>
// #include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

struct PointBPearl
{
    PCL_ADD_POINT4D
    float intensity;
    uint16_t ring;                   ///< laser ring number
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointBPearl,
                                 (float, x, x) (float, y, y) (float, z, z)
                                 (float, intensity, intensity)
                                 (double, timestamp, timestamp)
                                 (uint16_t, ring, ring))

struct PointOuster
{
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t  ring;
    uint16_t ambient; // Available in NTU VIRAL and multicampus datasets
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointOuster,
                                 (float, x, x) (float, y, y) (float, z, z)
                                 (float, intensity, intensity)
                                 (uint32_t, t, t)
                                 (uint16_t, reflectivity, reflectivity)
                                 (uint8_t,  ring, ring)
                                 (uint16_t, ambient, ambient)
                                 (uint32_t, range, range))

typedef pcl::PointCloud<PointOuster> CloudOuster;

template <typename PointType>
sensor_msgs::PointCloud2 publishCloud(ros::Publisher &thisPub,
                                        pcl::PointCloud<PointType> &thisCloud,
                                        ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    // if (thisPub.getNumSubscribers() != 0)
        thisPub.publish(tempCloud);
    return tempCloud;
}

using namespace std;
using namespace Eigen;
using namespace pcl;

class BPearlToOuster
{
private:
    // Node handler
    ros::NodeHandlePtr nh_ptr;

    ros::Subscriber bpearlCloudSub;
    ros::Publisher ousterCloudPub;

    bool remove_human_body = true;

public:
    // Destructor
    ~BPearlToOuster() {}

    BPearlToOuster(ros::NodeHandlePtr &nh_ptr_) : nh_ptr(nh_ptr_)
    {
        bpearlCloudSub = nh_ptr->subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 50, &BPearlToOuster::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        ousterCloudPub = nh_ptr->advertise<sensor_msgs::PointCloud2>("/os_cloud_node/points", 50);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &msgIn)
    {
        pcl::PointCloud<PointBPearl> laserCloudBPearl;
        pcl::fromROSMsg(*msgIn, laserCloudBPearl);
        
        int cloudsize = laserCloudBPearl.size();

        CloudOuster laserCloudOuster;
        laserCloudOuster.points.resize(cloudsize);
        laserCloudOuster.is_dense = true;

        static double hsToOusterIntensity = 1500.0/255.0;

        #pragma omp parallel for num_threads(omp_get_max_threads())
        // double max_intensity = -1;
        for (size_t i = 0; i < cloudsize; i++)
        {
            auto &src = laserCloudBPearl.points[i];
            auto &dst = laserCloudOuster.points[i];
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = src.intensity * hsToOusterIntensity;
            dst.ring = src.ring;
            dst.t = (int)((src.timestamp- laserCloudBPearl.points[0].timestamp) * 1e9f);
            dst.range = sqrt(src.x*src.x + src.y*src.y + src.z*src.z)*1000.0;

            // Remove points on the carrier
            // if(remove_human_body)
            // {
            //     double yaw = Util::wrapTo360(atan2(dst.y, dst.x)*180/M_PI);
            //     if (yaw > 38 && yaw < 142 && dst.range < 0.5)
            //     {
            //         dst.range = 0.0;
            //         dst.x = dst.y = dst.z = 0;
            //     }
            // }

            // max_intensity = (dst.intensity > max_intensity)?dst.intensity:max_intensity;
        }

        publishCloud(ousterCloudPub, laserCloudOuster, msgIn->header.stamp, msgIn->header.frame_id);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bpearl_to_ouster");
    ros::NodeHandle nh("~");
    ros::NodeHandlePtr nh_ptr = boost::make_shared<ros::NodeHandle>(nh);

    ROS_INFO("----> BPearl to Ouster started");

    BPearlToOuster B2O(nh_ptr);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    return 0;
}