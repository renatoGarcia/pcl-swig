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

%include <pcl/_numpy.i>
%include <pcl/point_types.i>

%include <std_vector.i>
%include <std_string.i>


/* %pcl_point_cloud_instantiate(type, type_alias, np_basic_type)
 *
 *  Generete the wrapper code to a specific cv::Mat_<> template instantiation.
 *
 *  type - The cv::Mat_<> value type.
 *  type_alias - The value type alias used at the cv::Mat_<> typedefs.
 *  np_basic_type - The character code[0] describing the numpy array item type.
 *
 *  For instance, the C++ type cv::Mat_<cv::Vec3b> would be instantiated with:
 *
 *      %cv_mat__instantiate(Vec3b, 3b, f)
 *
 *  which would generate a Python wrapper class Mat3b.
 *
 *  [0]: http://docs.scipy.org/doc/numpy/reference/arrays.interface.html#__array_interface__
 */
%define %pcl_point_cloud_instantiate(type, type_alias)

    /* %cv_numpy_add_type(type, np_basic_type) */

    #if !_PCL_POINT_CLOUD__##type##_INSTANTIATED_
        namespace pcl
        {
            %template(_PointCloud_##type) PointCloud< type >;
        }
        %pythoncode
        %{
            PointCloud##type_alias = _PointCloud_##type
        %}
        #define _PCL_POINT_CLOUD__##type##_INSTANTIATED_
    #endif
%enddef

%header
%{
    #include <pcl/point_cloud.h>
%}

%ignore getMatrixXfMap;
%ignore getMapping;
%ignore pcl::PointCloud::operator[];

%include <pcl/point_cloud.h>

%extend pcl::PointCloud
{
    std::size_t _get_data_ptr()
    {
        return reinterpret_cast<std::size_t>(&$self->points[0]);
    }

    int _get_dim()
    {
        return sizeof($parentclassname::PointType)/sizeof(float);
    }

    std::string __str__()
    {
        std::ostringstream s;
        s << *$self;
        return s.str();
    }

    %pythoncode
    %{
        @staticmethod
        def _typestr():
            typestr = _cv_numpy_endianess + "f4"
            return typestr

        def __getattribute__(self, name):
            if name == "__array_interface__":
                shape = (self.size(), self._get_dim())
                return {"shape": shape,
                        "typestr": self._typestr(),
                        "data": (self._get_data_ptr(), False)}

            else:
                return object.__getattribute__(self, name)

        #@classmethod
        #def from_array(cls, array):
        #    import numpy as np
        #    array = np.asarray(array)

        #    dtype = array.__array_interface__['typestr']
        #    if dtype[1:] != "f4":
        #        raise ValueError("{} expects a float32 datatype.".format(cls))

        #    if len(array.shape) != 2:
        #        raise ValueError("{} expects a 2-dimensional numpy ndarray.".format(cls))

        #    new_cloud = cls()

        #    if array.shape[1] != new_cloud._get_dim():
        #        raise ValueError("{} expects the last ndarray dimension to have a size of {}".format(cls, cls._get_dim()))

        #    for l in array:
        #        new_cloud.push_back(PointXYZ(l[0], l[1], l[2]))

        #    return new_cloud
    %}
}
