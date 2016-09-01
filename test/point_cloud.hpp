#include <pcl/point_cloud.h>

#ifndef SWIG
template <typename T>
bool compareXYZ(T const& t1, T const& t2)
{
    return (t1.x == t2.x) && (t1.y == t2.y) && (t1.z == t2.z);
}

template <typename T>
bool compareRGBA(T const& t1, T const& t2)
{
    return (t1.r == t2.r) && (t1.g == t2.g) && (t1.b == t2.b) && (t1.a == t2.a);
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

// -------------------------------------------------- PointCloudXYZRGB
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

PointCloudXYZRGB return_PointCloudXYZRGB()
{
    PointCloudXYZRGB cloud;
    pcl::PointXYZRGB p0, p1, p2;
    p0.x = 0.073; p0.y = 0.682; p0.z = 0.910; p0.r = 120; p0.g = 247; p0.b = 189;
    p1.x = 0.777; p1.y = 0.660; p1.z = 0.374; p1.r = 183; p1.g = 190; p1.b = 133;
    p2.x = 0.038; p2.y = 0.617; p2.z = 0.378; p2.r =  13; p2.g =   9; p2.b = 113;
    cloud.push_back(p0); cloud.push_back(p1); cloud.push_back(p2);

    return cloud;
}

PointCloudXYZRGB::Ptr return_PointCloudXYZRGB_Ptr()
{
    PointCloudXYZRGB::Ptr cloud(new PointCloudXYZRGB);
    pcl::PointXYZRGB p0, p1, p2;
    p0.x = 0.714; p0.y = 0.770; p0.z = 0.314; p0.r = 150; p0.g = 185; p0.b = 205;
    p1.x = 0.777; p1.y = 0.384; p1.z = 0.542; p1.r = 239; p1.g =  32; p1.b = 190;
    p2.x = 0.510; p2.y = 0.564; p2.z = 0.137; p2.r = 238; p2.g =  63; p2.b =  95;
    cloud->push_back(p0); cloud->push_back(p1); cloud->push_back(p2);

    return cloud;
}

bool receive_PointCloudXYZRGB(PointCloudXYZRGB const& cloud)
{
    PointCloudXYZRGB r;
    pcl::PointXYZRGB p0, p1, p2;
    p0.x = 0.551; p0.y = 0.059; p0.z = 0.984; p0.r =  50; p0.g =  17; p0.b =  54;
    p1.x = 0.049; p1.y = 0.963; p1.z = 0.433; p1.r =  44; p1.g = 149; p1.b =  43;
    p2.x = 0.854; p2.y = 0.542; p2.z = 0.935; p2.r = 209; p2.g = 163; p2.b = 135;
    r.push_back(p0); r.push_back(p1); r.push_back(p2);

    if (cloud.size() != r.size())
        return false;

    for (int i = 0; i < r.size(); ++i)
    {
        if (!compareRGBA(cloud.at(i), r.at(i)))
            return false;
    }

    return true;
}

bool receive_PointCloudXYZRGB_Ptr(PointCloudXYZRGB::Ptr cloud)
{
    PointCloudXYZRGB r;
    pcl::PointXYZRGB p0, p1, p2;
    p0.x = 0.176; p0.y = 0.023; p0.z = 0.972; p0.r =  48; p0.g = 242; p0.b = 184;
    p1.x = 0.421; p1.y = 0.759; p1.z = 0.389; p1.r =  38; p1.g =  92; p1.b =  51;
    p2.x = 0.518; p2.y = 0.898; p2.z = 0.609; p2.r = 253; p2.g =  13; p2.b = 150;
    r.push_back(p0); r.push_back(p1); r.push_back(p2);

    if (cloud->size() != r.size())
        return false;

    for (int i = 0; i < r.size(); ++i)
    {
        if (!compareRGBA(cloud->at(i), r.at(i)))
            return false;
    }

    return true;
}
