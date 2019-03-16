#pragma once

#include <OpenCVLinker.h>

using namespace std;
using namespace cv;

#define RED_COLOR Scalar(0, 0, 255)
#define BLUE_COLOR Scalar(255, 0, 0)
#define GREEN_COLOR Scalar(0, 255, 0)
#define YELLOW_COLOR Scalar(0, 255, 255)
#define WHITE_COLOR Scalar(255, 255, 255)
#define BLACK_COLOR Scalar(0, 0, 0)

struct CustomizedZone
{
   float Altitude;
   vector<Point> Zone;

   CustomizedZone() : Altitude( 0.0f ) {}
   CustomizedZone(const float& altitude, vector<Point> zone) : Altitude( altitude ), Zone( move( zone ) ) {}
};

class LocationDetection
{
   struct Camera
   {
      int Index;
      float FocalLength;
      float PanAngle;
      float TiltAngle;
      float CameraHeight;
      float Altitude;
      Mat CameraView;
      Matx33f Intrinsic;
      Matx33f PanningToCamera;
      Matx33f TiltingToCamera;
      Matx33f ToWorldCoordinate;
      Point3f Translation;

      Camera() : Index( 0 ), FocalLength( 0.0f ), PanAngle( 0.0f ), TiltAngle( 0.0f ), 
      CameraHeight( 0.0f ), Altitude( 0.0f ) {}
   };

   static LocationDetection* Instance;

   vector<Point> ClickedPoints;

   //Mat WorldMap;
   //Rect FloorOnWorldMap;

   Mat FloorImage;
   float ActualFloorWidth;  // ActualFloorWidth(m) * MeterToPixel(pixel/m) = FloorImage.cols(pixel)
   float ActualFloorHeight; // ActualFloorHeight(m) * MeterToPixel(pixel/m) = FloorImage.rows(pixel)
   float MeterToPixel;
   float DefaultAltitude;
   vector<CustomizedZone> CustomizedZones;
   vector<Camera> LocalCameras;

   void setWorldMap();

   void renderZone(Mat& image, const vector<Point>& zone, const Scalar& color = YELLOW_COLOR) const;
   
   bool isInsideZone(const Point& point, const vector<Point>& zone) const;
   
   void renderCameraPositionOnWorldMap(const Camera& camera);

   void transformWorldToCamera(
      Point& transformed, 
      const Point& world_point, 
      const float& altitude_of_point,
      const Camera& camera
   ) const;
   void renderZonesInCamera(Camera& camera);

   bool transformCameraToWorld(
      Point2f& transformed, 
      const Point& camera_point,
      const float& altitude_of_point,
      const Camera& camera
   ) const;
   Vec3b getPixelBilinearInterpolated(const Point2f& image_point);
   void renderCameraView(Camera& camera);

   bool isEndPoint(const int& x, const int& y);
   void customizeZonesCallback(int evt, int x, int y, int flags, void* param);
   static void customizeZonesCallbackWrapper(int evt, int x, int y, int flags, void* param);
   
   void pickPointCallback(int evt, int x, int y, int flags, void* param);
   static void pickPointCallbackWrapper(int evt, int x, int y, int flags, void* param);


public:
   LocationDetection(
      const float& actual_width, 
      const float& actual_height, 
      const vector<CustomizedZone>& zones = vector<CustomizedZone>()
   );
   ~LocationDetection() = default;

   void customizeZones();
   void setCamera(
      const int& camera_index,
      const int& width, 
      const int& height, 
      const float& focal_length, 
      const float& pan_angle_in_degree, 
      const float& tilt_angle_in_degree,
      const float& camera_height_in_meter,
      const Point2f& actual_position_in_meter
   );
   void detectLocation(Point& camera_point, const int& camera_index, const Point2f& actual_position_in_meter);
   void generateEvent();
};