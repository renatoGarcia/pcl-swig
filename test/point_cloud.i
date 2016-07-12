%module point_cloud

%include <pcl/point_cloud.i>

%pcl_point_cloud_instantiate(PointXYZ, XYZ)

%header %{
#include "point_cloud.hpp"
%}

%include "point_cloud.hpp"
