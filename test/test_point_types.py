#! /usr/bin/python3

import sys
sys.path.insert(0, ".")

import point_types


def compareRGBA(p1, p2):
    return p1.r == p2.r and p1.g == p2.g and p1.b == p2.b and p1.a == p2.a


def fillRGBA(p, r, g, b, a):
    p.r = r
    p.g = g
    p.b = b
    p.a = a


def compareXYZ(p1, p2):
    return p1.x == p2.x and p1.y == p2.y and p1.z == p2.z


def fillXYZ(p, x, y, z):
    p.x = x
    p.y = y
    p.z = z


class TestPointXYZ:
    def test_return(self):
        r = point_types.PointXYZ()
        fillXYZ(r, 0.053, 0.854, 0.133)

        p = point_types.return_PointXYZ()
        assert isinstance(p, point_types.PointXYZ)
        assert compareXYZ(p, r)

    def test_receive(self):
        p = point_types.PointXYZ()
        fillXYZ(p, 0.753, 0.685, 0.385)
        assert point_types.receive_PointXYZ(p)


class TestRGB:
    def test_return(self):
        r = point_types.RGB()
        fillRGBA(r, 35, 100, 60, 86)

        p = point_types.return_RGB()
        assert isinstance(p, point_types.RGB)
        assert compareRGBA(p, r)

    def test_receive(self):
        p = point_types.RGB()
        fillRGBA(p, 238, 64, 29, 27)
        assert point_types.receive_RGB(p)


class TestIntensity:
    def test_return(self):
        p = point_types.return_Intensity()
        assert isinstance(p, point_types.Intensity)

    def test_receive(self):
        p = point_types.Intensity()
        point_types.receive_Intensity(p)


class TestIntensity8u:
    def test_return(self):
        p = point_types.return_Intensity8u()
        assert isinstance(p, point_types.Intensity8u)

    def test_receive(self):
        p = point_types.Intensity8u()
        point_types.receive_Intensity8u(p)


class TestIntensity32u:
    def test_return(self):
        p = point_types.return_Intensity32u()
        assert isinstance(p, point_types.Intensity32u)

    def test_receive(self):
        p = point_types.Intensity32u()
        point_types.receive_Intensity32u(p)


class TestPointXYZI:
    def test_return(self):
        p = point_types.return_PointXYZI()
        assert isinstance(p, point_types.PointXYZI)

    def test_receive(self):
        p = point_types.PointXYZI()
        point_types.receive_PointXYZI(p)


class TestPointXYZL:
    def test_return(self):
        p = point_types.return_PointXYZL()
        assert isinstance(p, point_types.PointXYZL)

    def test_receive(self):
        p = point_types.PointXYZL()
        point_types.receive_PointXYZL(p)


class TestLabel:
    def test_return(self):
        p = point_types.return_Label()
        assert isinstance(p, point_types.Label)

    def test_receive(self):
        p = point_types.Label()
        point_types.receive_Label(p)


class TestPointXYZRGBA:
    def test_return(self):
        p = point_types.return_PointXYZRGBA()
        assert isinstance(p, point_types.PointXYZRGBA)

    def test_receive(self):
        p = point_types.PointXYZRGBA()
        point_types.receive_PointXYZRGBA(p)


class TestPointXYZRGB:
    def test_return(self):
        p = point_types.return_PointXYZRGB()
        assert isinstance(p, point_types.PointXYZRGB)

    def test_receive(self):
        p = point_types.PointXYZRGB()
        point_types.receive_PointXYZRGB(p)


class TestPointXYZRGBL:
    def test_return(self):
        p = point_types.return_PointXYZRGBL()
        assert isinstance(p, point_types.PointXYZRGBL)

    def test_receive(self):
        p = point_types.PointXYZRGBL()
        point_types.receive_PointXYZRGBL(p)


class TestPointXYZHSV:
    def test_return(self):
        p = point_types.return_PointXYZHSV()
        assert isinstance(p, point_types.PointXYZHSV)

    def test_receive(self):
        p = point_types.PointXYZHSV()
        point_types.receive_PointXYZHSV(p)


class TestPointXY:
    def test_return(self):
        p = point_types.return_PointXY()
        assert isinstance(p, point_types.PointXY)

    def test_receive(self):
        p = point_types.PointXY()
        point_types.receive_PointXY(p)


class TestPointUV:
    def test_return(self):
        p = point_types.return_PointUV()
        assert isinstance(p, point_types.PointUV)

    def test_receive(self):
        p = point_types.PointUV()
        point_types.receive_PointUV(p)


class TestInterestPoint:
    def test_return(self):
        p = point_types.return_InterestPoint()
        assert isinstance(p, point_types.InterestPoint)

    def test_receive(self):
        p = point_types.InterestPoint()
        point_types.receive_InterestPoint(p)


class TestNormal:
    def test_return(self):
        p = point_types.return_Normal()
        assert isinstance(p, point_types.Normal)

    def test_receive(self):
        p = point_types.Normal()
        point_types.receive_Normal(p)


class TestAxis:
    def test_return(self):
        p = point_types.return_Axis()
        assert isinstance(p, point_types.Axis)

    def test_receive(self):
        p = point_types.Axis()
        point_types.receive_Axis(p)


class TestPointNormal:
    def test_return(self):
        p = point_types.return_PointNormal()
        assert isinstance(p, point_types.PointNormal)

    def test_receive(self):
        p = point_types.PointNormal()
        point_types.receive_PointNormal(p)


class TestPointXYZRGBNormal:
    def test_return(self):
        p = point_types.return_PointXYZRGBNormal()
        assert isinstance(p, point_types.PointXYZRGBNormal)

    def test_receive(self):
        p = point_types.PointXYZRGBNormal()
        point_types.receive_PointXYZRGBNormal(p)


class TestPointXYZINormal:
    def test_return(self):
        p = point_types.return_PointXYZINormal()
        assert isinstance(p, point_types.PointXYZINormal)

    def test_receive(self):
        p = point_types.PointXYZINormal()
        point_types.receive_PointXYZINormal(p)


class TestPointXYZLNormal:
    def test_return(self):
        p = point_types.return_PointXYZLNormal()
        assert isinstance(p, point_types.PointXYZLNormal)

    def test_receive(self):
        p = point_types.PointXYZLNormal()
        point_types.receive_PointXYZLNormal(p)


class TestPointWithRange:
    def test_return(self):
        p = point_types.return_PointWithRange()
        assert isinstance(p, point_types.PointWithRange)

    def test_receive(self):
        p = point_types.PointWithRange()
        point_types.receive_PointWithRange(p)


class TestPointWithViewpoint:
    def test_return(self):
        p = point_types.return_PointWithViewpoint()
        assert isinstance(p, point_types.PointWithViewpoint)

    def test_receive(self):
        p = point_types.PointWithViewpoint()
        point_types.receive_PointWithViewpoint(p)


class TestMomentInvariants:
    def test_return(self):
        p = point_types.return_MomentInvariants()
        assert isinstance(p, point_types.MomentInvariants)

    def test_receive(self):
        p = point_types.MomentInvariants()
        point_types.receive_MomentInvariants(p)


class TestPrincipalRadiiRSD:
    def test_return(self):
        p = point_types.return_PrincipalRadiiRSD()
        assert isinstance(p, point_types.PrincipalRadiiRSD)

    def test_receive(self):
        p = point_types.PrincipalRadiiRSD()
        point_types.receive_PrincipalRadiiRSD(p)


class TestBoundary:
    def test_return(self):
        p = point_types.return_Boundary()
        assert isinstance(p, point_types.Boundary)

    def test_receive(self):
        p = point_types.Boundary()
        point_types.receive_Boundary(p)


class TestPrincipalCurvatures:
    def test_return(self):
        p = point_types.return_PrincipalCurvatures()
        assert isinstance(p, point_types.PrincipalCurvatures)

    def test_receive(self):
        p = point_types.PrincipalCurvatures()
        point_types.receive_PrincipalCurvatures(p)


class TestSHOT352:
    def test_return(self):
        p = point_types.return_SHOT352()
        assert isinstance(p, point_types.SHOT352)

    def test_receive(self):
        p = point_types.SHOT352()
        point_types.receive_SHOT352(p)


class TestSHOT1344:
    def test_return(self):
        p = point_types.return_SHOT1344()
        assert isinstance(p, point_types.SHOT1344)

    def test_receive(self):
        p = point_types.SHOT1344()
        point_types.receive_SHOT1344(p)


class TestReferenceFrame:
    def test_return(self):
        p = point_types.return_ReferenceFrame()
        assert isinstance(p, point_types.ReferenceFrame)

    def test_receive(self):
        p = point_types.ReferenceFrame()
        point_types.receive_ReferenceFrame(p)


class TestShapeContext1980:
    def test_return(self):
        p = point_types.return_ShapeContext1980()
        assert isinstance(p, point_types.ShapeContext1980)

    def test_receive(self):
        p = point_types.ShapeContext1980()
        point_types.receive_ShapeContext1980(p)


class TestUniqueShapeContext1960:
    def test_return(self):
        p = point_types.return_UniqueShapeContext1960()
        assert isinstance(p, point_types.UniqueShapeContext1960)

    def test_receive(self):
        p = point_types.UniqueShapeContext1960()
        point_types.receive_UniqueShapeContext1960(p)


class TestPFHSignature125:
    def test_return(self):
        p = point_types.return_PFHSignature125()
        assert isinstance(p, point_types.PFHSignature125)

    def test_receive(self):
        p = point_types.PFHSignature125()
        point_types.receive_PFHSignature125(p)


class TestPFHRGBSignature250:
    def test_return(self):
        p = point_types.return_PFHRGBSignature250()
        assert isinstance(p, point_types.PFHRGBSignature250)

    def test_receive(self):
        p = point_types.PFHRGBSignature250()
        point_types.receive_PFHRGBSignature250(p)


class TestPPFSignature:
    def test_return(self):
        p = point_types.return_PPFSignature()
        assert isinstance(p, point_types.PPFSignature)

    def test_receive(self):
        p = point_types.PPFSignature()
        point_types.receive_PPFSignature(p)


class TestCPPFSignature:
    def test_return(self):
        p = point_types.return_CPPFSignature()
        assert isinstance(p, point_types.CPPFSignature)

    def test_receive(self):
        p = point_types.CPPFSignature()
        point_types.receive_CPPFSignature(p)


class TestPPFRGBSignature:
    def test_return(self):
        p = point_types.return_PPFRGBSignature()
        assert isinstance(p, point_types.PPFRGBSignature)

    def test_receive(self):
        p = point_types.PPFRGBSignature()
        point_types.receive_PPFRGBSignature(p)


class TestNormalBasedSignature12:
    def test_return(self):
        p = point_types.return_NormalBasedSignature12()
        assert isinstance(p, point_types.NormalBasedSignature12)

    def test_receive(self):
        p = point_types.NormalBasedSignature12()
        point_types.receive_NormalBasedSignature12(p)


class TestFPFHSignature33:
    def test_return(self):
        p = point_types.return_FPFHSignature33()
        assert isinstance(p, point_types.FPFHSignature33)

    def test_receive(self):
        p = point_types.FPFHSignature33()
        point_types.receive_FPFHSignature33(p)


class TestVFHSignature308:
    def test_return(self):
        p = point_types.return_VFHSignature308()
        assert isinstance(p, point_types.VFHSignature308)

    def test_receive(self):
        p = point_types.VFHSignature308()
        point_types.receive_VFHSignature308(p)


class TestGRSDSignature21:
    def test_return(self):
        p = point_types.return_GRSDSignature21()
        assert isinstance(p, point_types.GRSDSignature21)

    def test_receive(self):
        p = point_types.GRSDSignature21()
        point_types.receive_GRSDSignature21(p)


class TestESFSignature640:
    def test_return(self):
        p = point_types.return_ESFSignature640()
        assert isinstance(p, point_types.ESFSignature640)

    def test_receive(self):
        p = point_types.ESFSignature640()
        point_types.receive_ESFSignature640(p)


class TestGFPFHSignature16:
    def test_return(self):
        p = point_types.return_GFPFHSignature16()
        assert isinstance(p, point_types.GFPFHSignature16)

    def test_receive(self):
        p = point_types.GFPFHSignature16()
        point_types.receive_GFPFHSignature16(p)


class TestBRISKSignature512:
    def test_return(self):
        p = point_types.return_BRISKSignature512()
        assert isinstance(p, point_types.BRISKSignature512)

    def test_receive(self):
        p = point_types.BRISKSignature512()
        point_types.receive_BRISKSignature512(p)


class TestNarf36:
    def test_return(self):
        p = point_types.return_Narf36()
        assert isinstance(p, point_types.Narf36)

    def test_receive(self):
        p = point_types.Narf36()
        point_types.receive_Narf36(p)


class TestBorderDescription:
    def test_return(self):
        p = point_types.return_BorderDescription()
        assert isinstance(p, point_types.BorderDescription)

    def test_receive(self):
        p = point_types.BorderDescription()
        point_types.receive_BorderDescription(p)


class TestIntensityGradient:
    def test_return(self):
        p = point_types.return_IntensityGradient()
        assert isinstance(p, point_types.IntensityGradient)

    def test_receive(self):
        p = point_types.IntensityGradient()
        point_types.receive_IntensityGradient(p)


class TestPointWithScale:
    def test_return(self):
        p = point_types.return_PointWithScale()
        assert isinstance(p, point_types.PointWithScale)

    def test_receive(self):
        p = point_types.PointWithScale()
        point_types.receive_PointWithScale(p)


class TestPointSurfel:
    def test_return(self):
        p = point_types.return_PointSurfel()
        assert isinstance(p, point_types.PointSurfel)

    def test_receive(self):
        p = point_types.PointSurfel()
        point_types.receive_PointSurfel(p)


class TestPointDEM:
    def test_return(self):
        p = point_types.return_PointDEM()
        assert isinstance(p, point_types.PointDEM)

    def test_receive(self):
        p = point_types.PointDEM()
        point_types.receive_PointDEM(p)
