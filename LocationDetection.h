/*
 * Author: Emoy Kim
 * E-mail: emoy.kim_AT_gmail.com
 * 
 * This code is a free software; it can be freely used, changed and redistributed.
 * If you use any version of the code, please reference the code.
 * 
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

#include "ProjectPath.h"

using uchar = unsigned char;
using uint = unsigned int;

const cv::Scalar RED_COLOR(0, 0, 255);
const cv::Scalar BLUE_COLOR(255, 0, 0);
const cv::Scalar GREEN_COLOR(0, 255, 0);
const cv::Scalar YELLOW_COLOR(0, 255, 255);
const cv::Scalar WHITE_COLOR(255, 255, 255);
const cv::Scalar BLACK_COLOR(0, 0, 0);

struct CustomizedZone
{
   float Altitude;
   std::vector<cv::Point> Zone;

   CustomizedZone() : Altitude( 0.0f ) {}
   CustomizedZone(float altitude, std::vector<cv::Point> zone) : Altitude( altitude ), Zone( std::move( zone ) ) {}
};

class LocationDetection
{
public:
   struct Camera
   {
      int Index;
      float FocalLength;
      float PanAngle;
      float TiltAngle;
      float CameraHeight;
      float Altitude;
      cv::Mat CameraView;
      cv::Matx33f Intrinsic;
      cv::Matx33f PanningToCamera;
      cv::Matx33f TiltingToCamera;
      cv::Matx33f ToWorldCoordinate;
      cv::Point3f Translation;

      Camera() : Index( 0 ), FocalLength( 0.0f ), PanAngle( 0.0f ), TiltAngle( 0.0f ), 
      CameraHeight( 0.0f ), Altitude( 0.0f ) {}
   };

   LocationDetection(
      float actual_width, 
      float actual_height, 
      const std::vector<CustomizedZone>& zones = std::vector<CustomizedZone>()
   );
   ~LocationDetection() = default;

   void customizeZones();
   void setCamera(
      int camera_index,
      int width, 
      int height, 
      float focal_length, 
      float pan_angle_in_degree, 
      float tilt_angle_in_degree,
      float camera_height_in_meter,
      const cv::Point2f& actual_position_in_meter
   );
   void generateEventOnWorldMap();
   void generateEventOnCamera(int camera_index);
   
   void detectLocation(cv::Point& camera_point, int camera_index, const cv::Point2f& actual_position_in_meter);
   void detectLocation(cv::Point2f& actual_position_in_meter, const cv::Point& camera_point, int camera_index);
   
private:
   inline static LocationDetection* Instance = nullptr;

   std::vector<cv::Point> ClickedPoints;

   cv::Mat FloorImage;
   float ActualFloorWidth;  // ActualFloorWidth(m) * MeterToPixel(pixel/m) = FloorImage.cols(pixel)
   float ActualFloorHeight; // ActualFloorHeight(m) * MeterToPixel(pixel/m) = FloorImage.rows(pixel)
   float MeterToPixel;
   float DefaultAltitude;
   std::vector<CustomizedZone> CustomizedZones;
   std::vector<Camera> LocalCameras;

   void renderZone(cv::Mat& image, const std::vector<cv::Point>& zone, const cv::Scalar& color = YELLOW_COLOR) const;
   
   bool isInsideZone(const cv::Point& point, const std::vector<cv::Point>& zone) const;
   
   void renderCameraPositionOnWorldMap(const Camera& camera);

   bool transformCameraToWorld(
      cv::Point2f& transformed, 
      const cv::Point& camera_point,
      float altitude_of_point,
      const Camera& camera
   ) const;
   bool canSeePointOnDefaultAltitude(cv::Point2f& world_point, const cv::Point& camera_point, const Camera& camera);
   bool getValidWorldPointFromCamera(cv::Point2f& valid_world_point, const cv::Point& camera_point, const Camera& camera);
   cv::Vec3b getPixelBilinearInterpolated(const cv::Point2f& image_point);
   void renderCameraView(Camera& camera);

   void transformWorldToCamera(
      cv::Point& transformed, 
      const cv::Point& world_point, 
      float altitude_of_point,
      const Camera& camera
   ) const;
   void renderZonesInCamera(Camera& camera);

   bool isEndPoint(int x, int y);
   void customizeZonesCallback(int evt, int x, int y, int flags, void* param);
   static void customizeZonesCallbackWrapper(int evt, int x, int y, int flags, void* param);
   
   void pickPointOnWorldMapCallback(int evt, int x, int y, int flags, void* param);
   void pickPointOnCameraCallback(int evt, int x, int y, int flags, void* param);
   static void pickPointOnWorldMapCallbackWrapper(int evt, int x, int y, int flags, void* param);
   static void pickPointOnCameraCallbackWrapper(int evt, int x, int y, int flags, void* param);
};