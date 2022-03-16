将LIOSAM修改为robosense激光雷达可用修改代码的范围：
1.修改utility.h文件在185-188行加入
        else if (sensorStr == "robosense")
        {
            sensor = SensorType::ROBOSENSE;//加入新成员robosense
        }
2.修改imageProjection.cpp文件
32-42行加入
struct RobosensePointXYZIRT {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (RobosensePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)
82行加入
    pcl::PointCloud<RobosensePointXYZIRT>::Ptr tmpRobosenseCloudIn;//新加入语句
121行加入
    tmpRobosenseCloudIn.reset(new pcl::PointCloud<RobosensePointXYZIRT>());//新加入语句
238-255行加入
	else if(sensor==SensorType::ROBOSENSE){
	// Convert to Velodyne format
	pcl::moveFromROSMsg(currentCloudMsg, *tmpRobosenseCloudIn);//修改此处
        laserCloudIn->points.resize(tmpRobosenseCloudIn->size());
        laserCloudIn->is_dense = tmpRobosenseCloudIn->is_dense;
        for (size_t i = 0; i < tmpRobosenseCloudIn->size(); i++)
            {
                auto &src = tmpRobosenseCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.time * 1e-9f;
            }

	}
