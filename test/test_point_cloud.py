#! /usr/bin/python3

import sys
sys.path.insert(0, ".")

import point_cloud
import numpy as np


def compareXYZ(p1, p2):
    return p1.x == p2.x and p1.y == p2.y and p1.z == p2.z


def compareRGBA(p1, p2):
    return p1.r == p2.r and p1.g == p2.g and p1.b == p2.b and p1.a == p2.a


class TestPointCloudXYZ:
    def test_return(self):
        r = point_cloud.PointCloudXYZ()
        r.push_back(point_cloud.PointXYZ(0.642, 0.054, 0.540))
        r.push_back(point_cloud.PointXYZ(0.287, 0.677, 0.195))
        r.push_back(point_cloud.PointXYZ(0.592, 0.165, 0.540))

        p = point_cloud.return_PointCloudXYZ()
        assert isinstance(p, point_cloud.PointCloudXYZ)
        assert p.size() == r.size()
        for i in range(p.size()):
            assert compareXYZ(p.at(i), r.at(i))

    def test_receive(self):
        p = point_cloud.PointCloudXYZ()
        p.push_back(point_cloud.PointXYZ(0.111, 0.303, 0.235))
        p.push_back(point_cloud.PointXYZ(0.990, 0.966, 0.592))
        p.push_back(point_cloud.PointXYZ(0.893, 0.030, 0.660))

        assert point_cloud.receive_PointCloudXYZ(p)

    def test_asarray(self):
        p = point_cloud.PointCloudXYZ()
        p.push_back(point_cloud.PointXYZ(0.077, 0.002, 0.478))
        p.push_back(point_cloud.PointXYZ(0.521, 0.981, 0.660))

        r = np.array([[0.077, 0.002, 0.478, 1.0],
                      [0.521, 0.981, 0.660, 1.0]], dtype=np.float32)

        ar = np.asarray(p)
        assert(ar.shape == (2, 4))
        assert(np.all(ar == r))

    # def test_from_array(self):
    #     ar = np.array([[0.757, 0.913, 0.248, 1.0],
    #                    [0.780, 0.746, 0.891, 1.0]], dtype=np.float32)

    #     r = point_cloud.PointCloudXYZ()
    #     r.push_back(point_cloud.PointXYZ(0.757, 0.913, 0.248))
    #     r.push_back(point_cloud.PointXYZ(0.780, 0.746, 0.891))

    #     p = point_cloud.PointCloudXYZ.from_array(ar)
    #     assert isinstance(p, point_cloud.PointCloudXYZ)
    #     assert p.size() == r.size()
    #     for i in range(p.size()):
    #         assert compareXYZ(p.at(i), r.at(i))


class TestPointCloudXYZRGB:
    def test_return(self):
        r = point_cloud.PointCloudXYZRGB()
        p0 = point_cloud.PointXYZRGB()
        p1 = point_cloud.PointXYZRGB()
        p2 = point_cloud.PointXYZRGB()

        p0.x, p0.y, p0.z, p0.r, p0.g, p0.b = 0.073, 0.682, 0.910, 120, 247, 189
        p1.x, p1.y, p1.z, p1.r, p1.g, p1.b = 0.777, 0.660, 0.374, 183, 190, 133
        p2.x, p2.y, p2.z, p2.r, p2.g, p2.b = 0.038, 0.617, 0.378,  13,   9, 113

        r.push_back(p0)
        r.push_back(p1)
        r.push_back(p2)

        cloud = point_cloud.return_PointCloudXYZRGB()
        assert isinstance(cloud, point_cloud.PointCloudXYZRGB)
        assert cloud.size() == r.size()
        for i in range(cloud.size()):
            assert compareXYZ(cloud.at(i), r.at(i))
            assert compareRGBA(cloud.at(i), r.at(i))

    def test_return_ptr(self):
        r = point_cloud.PointCloudXYZRGB()
        p0 = point_cloud.PointXYZRGB()
        p1 = point_cloud.PointXYZRGB()
        p2 = point_cloud.PointXYZRGB()

        p0.x, p0.y, p0.z, p0.r, p0.g, p0.b = 0.714, 0.770, 0.314, 150, 185, 205
        p1.x, p1.y, p1.z, p1.r, p1.g, p1.b = 0.777, 0.384, 0.542, 239,  32, 190
        p2.x, p2.y, p2.z, p2.r, p2.g, p2.b = 0.510, 0.564, 0.137, 238,  63,  95

        r.push_back(p0)
        r.push_back(p1)
        r.push_back(p2)

        cloud = point_cloud.return_PointCloudXYZRGB_Ptr()
        assert isinstance(cloud, point_cloud.PointCloudXYZRGB)
        assert cloud.size() == r.size()
        for i in range(cloud.size()):
            assert compareXYZ(cloud.at(i), r.at(i))
            assert compareRGBA(cloud.at(i), r.at(i))

    def test_receive(self):
        cloud = point_cloud.PointCloudXYZRGB()
        p0 = point_cloud.PointXYZRGB()
        p1 = point_cloud.PointXYZRGB()
        p2 = point_cloud.PointXYZRGB()

        p0.x, p0.y, p0.z, p0.r, p0.g, p0.b = 0.551, 0.059, 0.984,  50,  17,  54
        p1.x, p1.y, p1.z, p1.r, p1.g, p1.b = 0.049, 0.963, 0.433,  44, 149,  43
        p2.x, p2.y, p2.z, p2.r, p2.g, p2.b = 0.854, 0.542, 0.935, 209, 163, 135

        cloud.push_back(p0)
        cloud.push_back(p1)
        cloud.push_back(p2)

        assert point_cloud.receive_PointCloudXYZRGB(cloud)

    def test_receive_ptr(self):
        cloud = point_cloud.PointCloudXYZRGB()
        p0 = point_cloud.PointXYZRGB()
        p1 = point_cloud.PointXYZRGB()
        p2 = point_cloud.PointXYZRGB()

        p0.x, p0.y, p0.z, p0.r, p0.g, p0.b = 0.176, 0.023, 0.972,  48, 242, 184
        p1.x, p1.y, p1.z, p1.r, p1.g, p1.b = 0.421, 0.759, 0.389,  38,  92,  51
        p2.x, p2.y, p2.z, p2.r, p2.g, p2.b = 0.518, 0.898, 0.609, 253,  13, 150

        cloud.push_back(p0)
        cloud.push_back(p1)
        cloud.push_back(p2)

        assert point_cloud.receive_PointCloudXYZRGB_Ptr(cloud)

    def test_asarray(self):
        cloud = point_cloud.PointCloudXYZRGB()
        p0 = point_cloud.PointXYZRGB()
        p1 = point_cloud.PointXYZRGB()

        p0.x, p0.y, p0.z, p0.r, p0.g, p0.b = 0.799, 0.813, 0.662,   9, 153, 133
        p1.x, p1.y, p1.z, p1.r, p1.g, p1.b = 0.309, 0.685, 0.597, 188,   0,   7

        cloud.push_back(p0)
        cloud.push_back(p1)

        r = np.array([[0.799, 0.813, 0.662, 1.0],
                      [0.309, 0.685, 0.597, 1.0]], dtype=np.float32)

        ar = np.asarray(cloud)
        assert(ar.shape == (2, 8))
        # TODO: check for all array
        assert(np.all(ar[:, 0:4] == r))
