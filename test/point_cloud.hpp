#include <pcl/point_cloud.h>

#ifndef SWIG
template <typename T>
bool compareXYZ(T const& t1, T const& t2)
{
    return (t1.x == t2.x) && (t1.y == t2.y) && (t1.z == t2.z);
}
#endif

// -------------------------------------------------- PointCloudXYZ
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
PointCloudXYZ return_PointCloudXYZ()
{
    PointCloudXYZ p;
    p.push_back(pcl::PointXYZ(0.642, 0.054, 0.540));
    p.push_back(pcl::PointXYZ(0.287, 0.677, 0.195));
    p.push_back(pcl::PointXYZ(0.592, 0.165, 0.540));
    return p;
}

bool receive_PointCloudXYZ(PointCloudXYZ const& cloud)
{
    PointCloudXYZ r;
    r.push_back(pcl::PointXYZ(0.111, 0.303, 0.235));
    r.push_back(pcl::PointXYZ(0.990, 0.966, 0.592));
    r.push_back(pcl::PointXYZ(0.893, 0.030, 0.660));

    if (cloud.size() != r.size())
        return false;

    for (int i = 0; i < r.size(); ++i)
    {
        if (!compareXYZ(cloud.at(i), r.at(i)))
            return false;
    }

    return true;
}
