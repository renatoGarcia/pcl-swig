#include <pcl/point_types.h>

#include <cassert>

#ifndef SWIG
template <typename T>
bool compareXYZ(T const& t1, T const& t2)
{
    return (t1.x == t2.x) && (t1.y == t2.y) && (t1.z == t2.z);
}

template <typename T>
bool compareRGB(T const& t1, T const& t2)
{
    return (t1.r == t2.r) && (t1.g == t2.g) && (t1.b == t2.b)  && (t1.a == t2.a);
}

template <typename T>
bool fillRGBA(T& t1, uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    t1.r = r;
    t1.g = g;
    t1.b = b;
    t1.a = a;
}
#endif

// -------------------------------------------------- PointXYZ
pcl::PointXYZ return_PointXYZ()
{
    return pcl::PointXYZ(0.053, 0.854, 0.133);
}

bool receive_PointXYZ(pcl::PointXYZ const p)
{
    pcl::PointXYZ const r(0.753, 0.685, 0.385);
    return compareXYZ(p, r);
}

// -------------------------------------------------- RGB
pcl::RGB return_RGB()
{
    pcl::RGB r;
    fillRGBA(r, 35, 100, 60, 86);
    return r;
}

bool receive_RGB(pcl::RGB const p)
{
    pcl::RGB r;
    fillRGBA(r, 238, 64, 29, 27);
    return compareRGB(p, r);
}

// -------------------------------------------------- Intensity
pcl::Intensity return_Intensity()
{
    return pcl::Intensity();
}

void receive_Intensity(pcl::Intensity p)
{
    pcl::Intensity const r();
}

// -------------------------------------------------- Intensity8u
pcl::Intensity8u return_Intensity8u()
{
    return pcl::Intensity8u();
}

void receive_Intensity8u(pcl::Intensity8u p)
{
    pcl::Intensity8u const r();
}

// -------------------------------------------------- Intensity32u
pcl::Intensity32u return_Intensity32u()
{
    return pcl::Intensity32u();
}

void receive_Intensity32u(pcl::Intensity32u p)
{
    pcl::Intensity32u const r();
}

// -------------------------------------------------- PointXYZI
pcl::PointXYZI return_PointXYZI()
{
    return pcl::PointXYZI();
}

void receive_PointXYZI(pcl::PointXYZI p)
{
    pcl::PointXYZI const r();
}

// -------------------------------------------------- PointXYZL
pcl::PointXYZL return_PointXYZL()
{
    return pcl::PointXYZL();
}

void receive_PointXYZL(pcl::PointXYZL p)
{
    pcl::PointXYZL const r();
}

// -------------------------------------------------- Label
pcl::Label return_Label()
{
    return pcl::Label();
}

void receive_Label(pcl::Label p)
{
    pcl::Label const r();
}

// -------------------------------------------------- PointXYZRGBA
pcl::PointXYZRGBA return_PointXYZRGBA()
{
    return pcl::PointXYZRGBA();
}

void receive_PointXYZRGBA(pcl::PointXYZRGBA p)
{
    pcl::PointXYZRGBA const r();
}

// -------------------------------------------------- PointXYZRGB
pcl::PointXYZRGB return_PointXYZRGB()
{
    return pcl::PointXYZRGB();
}

void receive_PointXYZRGB(pcl::PointXYZRGB p)
{
    pcl::PointXYZRGB const r();
}

// -------------------------------------------------- PointXYZRGBL
pcl::PointXYZRGBL return_PointXYZRGBL()
{
    return pcl::PointXYZRGBL();
}

void receive_PointXYZRGBL(pcl::PointXYZRGBL p)
{
    pcl::PointXYZRGBL const r();
}

// -------------------------------------------------- PointXYZHSV
pcl::PointXYZHSV return_PointXYZHSV()
{
    return pcl::PointXYZHSV();
}

void receive_PointXYZHSV(pcl::PointXYZHSV p)
{
    pcl::PointXYZHSV const r();
}

// -------------------------------------------------- PointXY
pcl::PointXY return_PointXY()
{
    return pcl::PointXY();
}

void receive_PointXY(pcl::PointXY p)
{
    pcl::PointXY const r();
}

// -------------------------------------------------- PointUV
pcl::PointUV return_PointUV()
{
    return pcl::PointUV();
}

void receive_PointUV(pcl::PointUV p)
{
    pcl::PointUV const r();
}

// -------------------------------------------------- InterestPoint
pcl::InterestPoint return_InterestPoint()
{
    return pcl::InterestPoint();
}

void receive_InterestPoint(pcl::InterestPoint p)
{
    pcl::InterestPoint const r();
}

// -------------------------------------------------- Normal
pcl::Normal return_Normal()
{
    return pcl::Normal();
}

void receive_Normal(pcl::Normal p)
{
    pcl::Normal const r();
}

// -------------------------------------------------- Axis
pcl::Axis return_Axis()
{
    return pcl::Axis();
}

void receive_Axis(pcl::Axis p)
{
    pcl::Axis const r();
}

// -------------------------------------------------- PointNormal
pcl::PointNormal return_PointNormal()
{
    return pcl::PointNormal();
}

void receive_PointNormal(pcl::PointNormal p)
{
    pcl::PointNormal const r();
}

// -------------------------------------------------- PointXYZRGBNormal
pcl::PointXYZRGBNormal return_PointXYZRGBNormal()
{
    return pcl::PointXYZRGBNormal();
}

void receive_PointXYZRGBNormal(pcl::PointXYZRGBNormal p)
{
    pcl::PointXYZRGBNormal const r();
}

// -------------------------------------------------- PointXYZINormal
pcl::PointXYZINormal return_PointXYZINormal()
{
    return pcl::PointXYZINormal();
}

void receive_PointXYZINormal(pcl::PointXYZINormal p)
{
    pcl::PointXYZINormal const r();
}

// -------------------------------------------------- PointXYZLNormal
pcl::PointXYZLNormal return_PointXYZLNormal()
{
    return pcl::PointXYZLNormal();
}

void receive_PointXYZLNormal(pcl::PointXYZLNormal p)
{
    pcl::PointXYZLNormal const r();
}

// -------------------------------------------------- PointWithRange
pcl::PointWithRange return_PointWithRange()
{
    return pcl::PointWithRange();
}

void receive_PointWithRange(pcl::PointWithRange p)
{
    pcl::PointWithRange const r();
}

// -------------------------------------------------- PointWithViewpoint
pcl::PointWithViewpoint return_PointWithViewpoint()
{
    return pcl::PointWithViewpoint();
}

void receive_PointWithViewpoint(pcl::PointWithViewpoint p)
{
    pcl::PointWithViewpoint const r();
}

// -------------------------------------------------- MomentInvariants
pcl::MomentInvariants return_MomentInvariants()
{
    return pcl::MomentInvariants();
}

void receive_MomentInvariants(pcl::MomentInvariants p)
{
    pcl::MomentInvariants const r();
}

// -------------------------------------------------- PrincipalRadiiRSD
pcl::PrincipalRadiiRSD return_PrincipalRadiiRSD()
{
    return pcl::PrincipalRadiiRSD();
}

void receive_PrincipalRadiiRSD(pcl::PrincipalRadiiRSD p)
{
    pcl::PrincipalRadiiRSD const r();
}

// -------------------------------------------------- Boundary
pcl::Boundary return_Boundary()
{
    return pcl::Boundary();
}

void receive_Boundary(pcl::Boundary p)
{
    pcl::Boundary const r();
}

// -------------------------------------------------- PrincipalCurvatures
pcl::PrincipalCurvatures return_PrincipalCurvatures()
{
    return pcl::PrincipalCurvatures();
}

void receive_PrincipalCurvatures(pcl::PrincipalCurvatures p)
{
    pcl::PrincipalCurvatures const r();
}

// -------------------------------------------------- SHOT352
pcl::SHOT352 return_SHOT352()
{
    return pcl::SHOT352();
}

void receive_SHOT352(pcl::SHOT352 p)
{
    pcl::SHOT352 const r();
}

// -------------------------------------------------- SHOT1344
pcl::SHOT1344 return_SHOT1344()
{
    return pcl::SHOT1344();
}

void receive_SHOT1344(pcl::SHOT1344 p)
{
    pcl::SHOT1344 const r();
}

// -------------------------------------------------- ReferenceFrame
pcl::ReferenceFrame return_ReferenceFrame()
{
    return pcl::ReferenceFrame();
}

void receive_ReferenceFrame(pcl::ReferenceFrame p)
{
    pcl::ReferenceFrame const r();
}

// -------------------------------------------------- ShapeContext1980
pcl::ShapeContext1980 return_ShapeContext1980()
{
    return pcl::ShapeContext1980();
}

void receive_ShapeContext1980(pcl::ShapeContext1980 p)
{
    pcl::ShapeContext1980 const r();
}

// -------------------------------------------------- UniqueShapeContext1960
pcl::UniqueShapeContext1960 return_UniqueShapeContext1960()
{
    return pcl::UniqueShapeContext1960();
}

void receive_UniqueShapeContext1960(pcl::UniqueShapeContext1960 p)
{
    pcl::UniqueShapeContext1960 const r();
}

// -------------------------------------------------- PFHSignature125
pcl::PFHSignature125 return_PFHSignature125()
{
    return pcl::PFHSignature125();
}

void receive_PFHSignature125(pcl::PFHSignature125 p)
{
    pcl::PFHSignature125 const r();
}

// -------------------------------------------------- PFHRGBSignature250
pcl::PFHRGBSignature250 return_PFHRGBSignature250()
{
    return pcl::PFHRGBSignature250();
}

void receive_PFHRGBSignature250(pcl::PFHRGBSignature250 p)
{
    pcl::PFHRGBSignature250 const r();
}

// -------------------------------------------------- PPFSignature
pcl::PPFSignature return_PPFSignature()
{
    return pcl::PPFSignature();
}

void receive_PPFSignature(pcl::PPFSignature p)
{
    pcl::PPFSignature const r();
}

// -------------------------------------------------- CPPFSignature
pcl::CPPFSignature return_CPPFSignature()
{
    return pcl::CPPFSignature();
}

void receive_CPPFSignature(pcl::CPPFSignature p)
{
    pcl::CPPFSignature const r();
}

// -------------------------------------------------- PPFRGBSignature
pcl::PPFRGBSignature return_PPFRGBSignature()
{
    return pcl::PPFRGBSignature();
}

void receive_PPFRGBSignature(pcl::PPFRGBSignature p)
{
    pcl::PPFRGBSignature const r();
}

// -------------------------------------------------- NormalBasedSignature12
pcl::NormalBasedSignature12 return_NormalBasedSignature12()
{
    return pcl::NormalBasedSignature12();
}

void receive_NormalBasedSignature12(pcl::NormalBasedSignature12 p)
{
    pcl::NormalBasedSignature12 const r();
}

// -------------------------------------------------- FPFHSignature33
pcl::FPFHSignature33 return_FPFHSignature33()
{
    return pcl::FPFHSignature33();
}

void receive_FPFHSignature33(pcl::FPFHSignature33 p)
{
    pcl::FPFHSignature33 const r();
}

// -------------------------------------------------- VFHSignature308
pcl::VFHSignature308 return_VFHSignature308()
{
    return pcl::VFHSignature308();
}

void receive_VFHSignature308(pcl::VFHSignature308 p)
{
    pcl::VFHSignature308 const r();
}

// -------------------------------------------------- GRSDSignature21
pcl::GRSDSignature21 return_GRSDSignature21()
{
    return pcl::GRSDSignature21();
}

void receive_GRSDSignature21(pcl::GRSDSignature21 p)
{
    pcl::GRSDSignature21 const r();
}

// -------------------------------------------------- ESFSignature640
pcl::ESFSignature640 return_ESFSignature640()
{
    return pcl::ESFSignature640();
}

void receive_ESFSignature640(pcl::ESFSignature640 p)
{
    pcl::ESFSignature640 const r();
}

// -------------------------------------------------- GFPFHSignature16
pcl::GFPFHSignature16 return_GFPFHSignature16()
{
    return pcl::GFPFHSignature16();
}

void receive_GFPFHSignature16(pcl::GFPFHSignature16 p)
{
    pcl::GFPFHSignature16 const r();
}

// -------------------------------------------------- BRISKSignature512
pcl::BRISKSignature512 return_BRISKSignature512()
{
    return pcl::BRISKSignature512();
}

void receive_BRISKSignature512(pcl::BRISKSignature512 p)
{
    pcl::BRISKSignature512 const r();
}

// -------------------------------------------------- Narf36
pcl::Narf36 return_Narf36()
{
    return pcl::Narf36();
}

void receive_Narf36(pcl::Narf36 p)
{
    pcl::Narf36 const r();
}

// -------------------------------------------------- BorderDescription
pcl::BorderDescription return_BorderDescription()
{
    return pcl::BorderDescription();
}

void receive_BorderDescription(pcl::BorderDescription p)
{
    pcl::BorderDescription const r();
}

// -------------------------------------------------- IntensityGradient
pcl::IntensityGradient return_IntensityGradient()
{
    return pcl::IntensityGradient();
}

void receive_IntensityGradient(pcl::IntensityGradient p)
{
    pcl::IntensityGradient const r();
}

//TODO: pcl::Histogram<int N>

// -------------------------------------------------- PointWithScale
pcl::PointWithScale return_PointWithScale()
{
    return pcl::PointWithScale();
}

void receive_PointWithScale(pcl::PointWithScale p)
{
    pcl::PointWithScale const r();
}

// -------------------------------------------------- PointSurfel
pcl::PointSurfel return_PointSurfel()
{
    return pcl::PointSurfel();
}

void receive_PointSurfel(pcl::PointSurfel p)
{
    pcl::PointSurfel const r();
}

// -------------------------------------------------- PointDEM
pcl::PointDEM return_PointDEM()
{
    return pcl::PointDEM();
}

void receive_PointDEM(pcl::PointDEM p)
{
    pcl::PointDEM const r();
}
