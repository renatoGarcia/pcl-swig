#! /usr/bin/python3

import sys
sys.path.insert(0, ".")

import point_cloud
import numpy as np


def compareXYZ(p1, p2):
    return p1.x == p2.x and p1.y == p2.y and p1.z == p2.z


def fillXYZ(p, x, y, z):
    p.x = x
    p.y = y
    p.z = z


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
