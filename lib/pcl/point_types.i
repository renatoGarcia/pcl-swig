/* Copyright (c) 2016 The PCL-SWIG Library Developers. See the AUTHORS file at the
 * top-level directory of this distribution and at
 * https://github.com/renatoGarcia/pcl-swig/blob/master/AUTHORS.
 *
 * This file is part of PCL-SWIG Library. It is subject to the 3-clause BSD license terms
 * as in the LICENSE file found in the top-level directory of this distribution and at
 * https://github.com/renatoGarcia/pcl-swig/blob/master/LICENSE. No part of PCL-SWIG
 * Library, including this file, may be copied, modified, propagated, or distributed
 * except according to the terms contained in the LICENSE file.
 */

%include <stdint.i>
%include <std_string.i>

#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#define PCL_EXPORTS
#define EIGEN_ALIGN16

#define POINT_CLOUD_REGISTER_POINT_STRUCT(A, B)
#define POINT_CLOUD_REGISTER_POINT_WRAPPER(A, B)

%header
%{
    #include <pcl/point_types.h>
%}

%define %_extend__str__(class_name)
    %extend pcl::class_name {
        std::string __str__()
        {
            std::ostringstream s;
            s << *$self;
            return s.str();
        }
    }
%enddef

%define %_extend_attribute(class_name, attr_type, attr_name)
    %extend pcl::class_name {
        attr_type attr_name;
    }
    %header
    {
        inline
        attr_type pcl_##class_name##_##attr_name##_get(pcl::##class_name##* p)
        {
            return p->##attr_name##;
        }
        inline
        void pcl_##class_name##_##attr_name##_set(pcl::##class_name##* p, attr_type attr_name)
        {
            p->##attr_name = attr_name;
        }
    }
%enddef

%define %_extend_BGRA(class_name)
    %_extend_attribute(class_name, uint8_t, b)
    %_extend_attribute(class_name, uint8_t, g)
    %_extend_attribute(class_name, uint8_t, r)
    %_extend_attribute(class_name, uint8_t, a)
    %_extend_attribute(class_name, float, rgb)
    %_extend_attribute(class_name, uint32_t, rgba)
%enddef

%define %_extend_XYZ(class_name)
    %_extend_attribute(class_name, float, x)
    %_extend_attribute(class_name, float, y)
    %_extend_attribute(class_name, float, z)
%enddef

%define %_extend_L(class_name)
    %_extend_attribute(class_name, uint32_t, label)
%enddef

%define %_extend_NORMAL(class_name)
    %_extend_attribute(class_name, float, normal_x)
    %_extend_attribute(class_name, float, normal_y)
    %_extend_attribute(class_name, float, normal_z)
%enddef

//TODO: Extend the other attributes

%_extend__str__(PointXYZ)
%_extend__str__(RGB)
%_extend__str__(Intensity)
%_extend__str__(Intensity8u)
/* %_extend__str__(Intensity32u) */ // Intensity32u symbol not defined when loading library.
%_extend__str__(PointXYZI)
%_extend__str__(PointXYZL)
%_extend__str__(Label)
%_extend__str__(PointXYZRGBA)
%_extend__str__(PointXYZRGB)
%_extend__str__(PointXYZRGBL)
%_extend__str__(PointXYZHSV)
%_extend__str__(PointXY)
%_extend__str__(PointUV)
%_extend__str__(InterestPoint)
%_extend__str__(Normal)
%_extend__str__(Axis)
%_extend__str__(PointNormal)
%_extend__str__(PointXYZRGBNormal)
%_extend__str__(PointXYZINormal)
%_extend__str__(PointXYZLNormal)
%_extend__str__(PointWithRange)
%_extend__str__(PointWithViewpoint)
%_extend__str__(MomentInvariants)
%_extend__str__(PrincipalRadiiRSD)
%_extend__str__(Boundary)
%_extend__str__(PrincipalCurvatures)
%_extend__str__(SHOT352)
%_extend__str__(SHOT1344)
%_extend__str__(ReferenceFrame)
%_extend__str__(ShapeContext1980)
%_extend__str__(UniqueShapeContext1960)
%_extend__str__(PFHSignature125)
%_extend__str__(PFHRGBSignature250)
%_extend__str__(PPFSignature)
%_extend__str__(CPPFSignature)
%_extend__str__(PPFRGBSignature)
%_extend__str__(NormalBasedSignature12)
%_extend__str__(FPFHSignature33)
%_extend__str__(VFHSignature308)
/* %_extend__str__(GRSDSignature21) */ // GRSDSignature21symbol not defined when loading library.
%_extend__str__(ESFSignature640)
%_extend__str__(GFPFHSignature16)
%_extend__str__(BRISKSignature512)
%_extend__str__(Narf36)
%_extend__str__(BorderDescription)
%_extend__str__(IntensityGradient)
%_extend__str__(Histogram)
%_extend__str__(PointWithScale)
%_extend__str__(PointSurfel)
%_extend__str__(PointDEM)

%_extend_BGRA(RGB)
%_extend_BGRA(PointXYZRGBA)
%_extend_BGRA(PointXYZRGB)
%_extend_BGRA(PointXYZRGBL)
%_extend_BGRA(PointXYZRGBNormal)
%_extend_BGRA(PointSurfel)

%_extend_XYZ(PointXYZ)
%_extend_XYZ(PointXYZI)
%_extend_XYZ(PointXYZL)
%_extend_XYZ(PointXYZRGBA)
%_extend_XYZ(PointXYZRGB)
%_extend_XYZ(PointXYZRGBL)
%_extend_XYZ(PointXYZHSV)
%_extend_XYZ(InterestPoint)
%_extend_XYZ(PointNormal)
%_extend_XYZ(PointXYZRGBNormal)
%_extend_XYZ(PointXYZINormal)
%_extend_XYZ(PointXYZLNormal)
%_extend_XYZ(PointWithRange)
%_extend_XYZ(PointWithViewpoint)
%_extend_XYZ(PointWithScale)
%_extend_XYZ(PointSurfel)
%_extend_XYZ(PointDEM)

%_extend_L(PointXYZL)
%_extend_L(PointXYZRGBL)
%_extend_L(PointXYZLNormal)

%_extend_NORMAL(Normal)
%_extend_NORMAL(PointNormal)
%_extend_NORMAL(PointXYZRGBNormal)
%_extend_NORMAL(PointXYZINormal)
%_extend_NORMAL(PointXYZLNormal)
%_extend_NORMAL(PointSurfel)


%ignore getVector3fMap;
%ignore getArray3fMap;
%ignore getVector4fMap;
%ignore getArray4fMap;

%ignore getRGBVector3i;
%ignore getRGBVector4i;
%ignore getRGBAVector4i;
%ignore getBGRVector3cMap;
%ignore getBGRVector3cMap;
%ignore getBGRAVector4cMap;

%ignore getNormalVector3fMap;
%ignore getNormalVector4fMap;

%ignore getXAxisVector3fMap;
%ignore getYAxisVector3fMap;
%ignore getZAxisVector3fMap;
%ignore getMatrix3fMap;

%ignore pcl::operator<<;
%ignore operator<<;

%ignore pcl::FieldMatches;

#pragma SWIG nowarn=312
%include <pcl/point_types.h>
%include <pcl/impl/point_types.hpp>
