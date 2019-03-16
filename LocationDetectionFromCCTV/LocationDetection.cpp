#include "LocationDetection.h"

LocationDetection* LocationDetection::Instance = nullptr;
LocationDetection::LocationDetection(const float& actual_width, const float& actual_height, const vector<CustomizedZone>& zones) :
   ActualFloorWidth( actual_width ), ActualFloorHeight( actual_height ), DefaultAltitude( 1.0f )
{
   Instance = this;

   FloorImage = imread( "floor.jpg" );
   if (!FloorImage.empty()) {
      MeterToPixel = static_cast<float>(FloorImage.cols) / ActualFloorWidth;
      setWorldMap();
      if (zones.empty()) customizeZones();
      else CustomizedZones = zones;
   }
   else cout << "Cannot Load the Image..." << endl;
}

void LocationDetection::setWorldMap()
{
   const int width = static_cast<int>(round( ActualFloorWidth * MeterToPixel ));
   const int height = static_cast<int>(round( ActualFloorHeight * MeterToPixel ));
   //FloorOnWorldMap = Rect(width / 10, height / 3, width, height);
   //WorldMap = Mat(height * 7 / 5, width * 6 / 5, CV_8UC3, Scalar::all(160));
   //putText( 
   //   WorldMap, 
   //   "World Map", 
   //   Point(WorldMap.cols * 4 / 13, WorldMap.rows / 7), 
   //   FONT_HERSHEY_SCRIPT_COMPLEX, 
   //   5.0, WHITE_COLOR, 5
   //);
   //FloorImage.copyTo( WorldMap(FloorOnWorldMap) );
}

bool LocationDetection::isEndPoint(const int& x, const int& y)
{
   if (ClickedPoints.size() <= 2) return false;

   const int squared_distance_to_first = 
      (ClickedPoints[0].x - x) * (ClickedPoints[0].x - x) +
      (ClickedPoints[0].y - y) * (ClickedPoints[0].y - y);
   return squared_distance_to_first <= 50;
}

void LocationDetection::renderZone(Mat& image, const vector<Point>& zone, const Scalar& color) const
{
   for (uint e1 = 0, e2 = zone.size() - 1; e1 < zone.size(); e2 = e1++) {
      line( image, zone[e1], zone[e2], color, 5 );
   }
}

void LocationDetection::customizeZonesCallback(int evt, int x, int y, int flags, void* param)
{
   static bool complete = false;
   if (ClickedPoints.empty()) complete = false;

   if (evt == CV_EVENT_LBUTTONDOWN) {
      Mat viewer = static_cast<Mat*>(param)->clone();
      if (complete || isEndPoint( x, y )) {
         if (!complete) {
            vector<Point> convex;
            convexHull( ClickedPoints, convex );
            ClickedPoints = move( convex );
            complete = true;
         }
         renderZone( viewer, ClickedPoints, GREEN_COLOR );
      }
      else {
         ClickedPoints.emplace_back( x, y );
         for (uint i = 1; i < ClickedPoints.size(); ++i) {
            line( viewer, ClickedPoints[i - 1], ClickedPoints[i], YELLOW_COLOR, 5 );
         }
         circle( viewer, ClickedPoints[0], 10, RED_COLOR, -1 );
      }
      imshow( "Customizing Zones", viewer );
   }
}

void LocationDetection::customizeZonesCallbackWrapper(int evt, int x, int y, int flags, void* param)
{
   Instance->customizeZonesCallback( evt, x, y, flags, param );
}

void LocationDetection::customizeZones()
{
   int key = -1;
   CustomizedZones.clear();
   while (key != 'q') {
      ClickedPoints.clear();
      Mat viewer = FloorImage.clone();
      namedWindow( "Customizing Zones", 0 );
      resizeWindow( "Customizing Zones", FloorImage.cols / 3, FloorImage.rows / 3 );
      imshow( "Customizing Zones", viewer );
      setMouseCallback( "Customizing Zones", customizeZonesCallbackWrapper, &viewer );
      key = waitKey();
      destroyWindow( "Customizing Zones" );

      if (key == 'r' || ClickedPoints.empty()) continue;
      if (key == 13 /* Enter */) {
         float altitude;
         cout << ">> Enter the Altitude in Meter." << endl;
         cin >> altitude;
         CustomizedZones.emplace_back( altitude, ClickedPoints );

         renderZone( FloorImage, ClickedPoints );
      }
   }
}

bool LocationDetection::isInsideZone(const Point& point, const vector<Point>& zone) const
{
   if (zone.size() <= 2) return false;
   
   bool is_inside = false;
   for (uint i = 0, j = zone.size() - 1; i < zone.size(); j = i++) {
      if (((zone[i].y <= point.y && point.y < zone[j].y) || (zone[j].y <= point.y && point.y < zone[i].y)) && 
          (point.x < (zone[j].x - zone[i].x) * (point.y - zone[i].y) / (zone[j].y - zone[i].y) + zone[i].x)) 
         is_inside = !is_inside;
   }
   return is_inside;
}

void LocationDetection::renderCameraPositionOnWorldMap(const Camera& camera)
{
   const Point3f origin_vector(0.0f, 0.0f, 35.0f);
   const Scalar color(51, 153, 255);

   const float half_fov = atan( camera.CameraView.cols * 0.5f / camera.FocalLength );
   const float cos_fov = cos( half_fov );
   const float sin_fov = sin( half_fov );
   const float cos_fov_neg = cos( -half_fov );
   const float sin_fov_neg = sin( -half_fov );
   const Matx33f fov_pos = Matx33f(
      cos_fov, 0.0f, -sin_fov,
      0.0f, 1.0f, 0.0f,
      sin_fov, 0.0f, cos_fov
   );
   const Matx33f fov_neg = Matx33f(
      cos_fov_neg, 0.0f, -sin_fov_neg,
      0.0f, 1.0f, 0.0f,
      sin_fov_neg, 0.0f,  cos_fov_neg
   );
   const Matx33f pan_inv = camera.PanningToCamera.inv();
   const Matx33f tilt_inv = camera.TiltingToCamera.inv();
   const Point3f view_vector = tilt_inv * pan_inv * origin_vector;
   const Point camera_in_world(
      static_cast<int>(round( camera.Translation.z * MeterToPixel )), 
      static_cast<int>(round( camera.Translation.x * MeterToPixel ))
   );
   const Point camera_direction = camera_in_world + Point(
      static_cast<int>(round( view_vector.z )), static_cast<int>(round( view_vector.x ))
   );
   arrowedLine( FloorImage, camera_in_world, camera_direction, color, 2 );

   const Point3f left_fov = tilt_inv * fov_pos * pan_inv * origin_vector;
   const Point3f right_fov = tilt_inv * fov_neg * pan_inv * origin_vector;
   const Point left_direction = camera_in_world + Point(
      static_cast<int>(round( left_fov.z )), static_cast<int>(round( left_fov.x ))
   );
   const Point right_direction = camera_in_world + Point(
      static_cast<int>(round( right_fov.z )), static_cast<int>(round( right_fov.x ))
   );
   line( FloorImage, camera_in_world, left_direction, color );
   line( FloorImage, camera_in_world, right_direction, color );
}

void LocationDetection::transformWorldToCamera(
   Point& transformed, 
   const Point& world_point,
   const float& altitude_of_point,
   const Camera& camera
) const
// camera's view direction is z-axis, down direction is y-axis, and right direction is x-axis.
{
   const Point2f actual_point_in_meter(
      static_cast<float>(world_point.x) / MeterToPixel, 
      static_cast<float>(world_point.y) / MeterToPixel
   );
   const float h = camera.CameraHeight + camera.Altitude - altitude_of_point;
   Point3f world = camera.Intrinsic * camera.TiltingToCamera * camera.PanningToCamera * Point3f(
      actual_point_in_meter.y - camera.Translation.x, 
      h, 
      actual_point_in_meter.x - camera.Translation.z
   );
   if (world.z == 0.0f) world.z = 1e-7f;
   world.x /= world.z;
   world.y /= world.z;
   transformed.x = static_cast<int>(round( world.x ));
   transformed.y = static_cast<int>(round( world.y ));
}

void LocationDetection::renderZonesInCamera(Camera& camera)
{
   for (const auto& zone : CustomizedZones) {
      vector<Point> points_in_camera(zone.Zone.size());
      for (uint i = 0; i < zone.Zone.size(); ++i) {
         transformWorldToCamera( points_in_camera[i], zone.Zone[i], zone.Altitude, camera );
      }
      renderZone( camera.CameraView, points_in_camera, YELLOW_COLOR );
   }
}

void LocationDetection::setCamera(
   const int& camera_index,
   const int& width, 
   const int& height, 
   const float& focal_length, 
   const float& pan_angle_in_degree, 
   const float& tilt_angle_in_degree,
   const float& camera_height_in_meter,
   const Point2f& actual_position_in_meter
)
{
   Camera camera;
   camera.Index = camera_index;
   camera.FocalLength = focal_length;
   camera.PanAngle = pan_angle_in_degree * static_cast<float>(CV_PI) / 180.0f;
   camera.TiltAngle = tilt_angle_in_degree * static_cast<float>(CV_PI) / 180.0f;
   camera.Translation = { actual_position_in_meter.y, 0.0f, actual_position_in_meter.x };
   camera.CameraHeight = camera_height_in_meter;
   camera.Altitude = DefaultAltitude;
   camera.CameraView = Mat(height, width, CV_8UC3, WHITE_COLOR);
   camera.Intrinsic = Matx33f(
      focal_length, 0.0f, static_cast<float>(width) * 0.5f,
      0.0f, focal_length, static_cast<float>(height) * 0.5f,
      0.0f, 0.0f, 1.0f
   );

   const float sin_pan = sin( camera.PanAngle );
   const float cos_pan = cos( camera.PanAngle );
   camera.PanningToCamera = Matx33f(
      cos_pan, 0.0f, -sin_pan,
      0.0f, 1.0f, 0.0f,
      sin_pan, 0.0f, cos_pan
   );
   const float sin_tilt = sin( camera.TiltAngle );
   const float cos_tilt = cos( camera.TiltAngle );
   camera.TiltingToCamera = Matx33f(
      1.0f, 0.0f, 0.0f,
      0.0f, cos_tilt, -sin_tilt,
      0.0f, sin_tilt, cos_tilt
   );
   camera.ToWorldCoordinate = camera.PanningToCamera.inv() * camera.TiltingToCamera.inv();

   const Point camera_in_world = Point(
      static_cast<int>(round( actual_position_in_meter.x * MeterToPixel )), 
      static_cast<int>(round( actual_position_in_meter.y * MeterToPixel ))
   );
   for (const auto& zone : CustomizedZones) {
      if (isInsideZone( camera_in_world, zone.Zone )) {
         camera.Altitude = zone.Altitude;
         break;
      }
   }

   renderCameraPositionOnWorldMap( camera );
   renderZonesInCamera( camera );
   LocalCameras.emplace_back( camera );
}

void LocationDetection::detectLocation(Point& camera_point, const int& camera_index, const Point2f& actual_position_in_meter)
{
   if (static_cast<int>(LocalCameras.size()) <= camera_index) {
      camera_point = { -1, -1 };
      return;
   }

   const Point world_point(
      static_cast<int>(round( actual_position_in_meter.x * MeterToPixel )), 
      static_cast<int>(round( actual_position_in_meter.y * MeterToPixel ))
   );

   float altitude = DefaultAltitude;
   for (const auto& zone : CustomizedZones) {
      if (isInsideZone( world_point, zone.Zone )) {
         altitude = zone.Altitude;
         break;
      }
   }
   transformWorldToCamera( camera_point, world_point, altitude, LocalCameras[camera_index] );
}

bool LocationDetection::transformCameraToWorld(
   Point2f& transformed, 
   const Point& camera_point,
   const float& altitude_of_point,
   const Camera& camera
) const
{
   const auto half_width = static_cast<float>(camera.CameraView.cols) * 0.5f;
   const auto half_height = static_cast<float>(camera.CameraView.rows) * 0.5f;
   const float sin_tilt = sin( camera.TiltAngle );
   const float cos_tilt = cos( camera.TiltAngle );
   const float f_mul_sin_tilt = camera.FocalLength * sin_tilt;

   Point3f ground_point;
   ground_point.z = f_mul_sin_tilt + (static_cast<float>(camera_point.y) - half_height) * cos_tilt;

   const float h = camera.CameraHeight + camera.Altitude - altitude_of_point;
   if (ground_point.z <= 0.0f || h < 0.0f) return false;

   ground_point.z = h / ground_point.z;
   ground_point.x = (static_cast<float>(camera_point.x) - half_width) * ground_point.z;
   ground_point.y = (static_cast<float>(camera_point.y) - half_height) * ground_point.z;
   ground_point.z = camera.FocalLength * ground_point.z;

   const Point3f world_point = camera.ToWorldCoordinate * ground_point + camera.Translation;
   const Point2f image_point(world_point.z * MeterToPixel, world_point.x * MeterToPixel);
   transformed = image_point;

   return 
      0.0f <= image_point.x && image_point.x < static_cast<float>(FloorImage.cols) && 
      0.0f <= image_point.y && image_point.y < static_cast<float>(FloorImage.rows);
}

Vec3b LocationDetection::getPixelBilinearInterpolated(const Point2f& image_point)
{
   const auto x0 = static_cast<int>(floor( image_point.x ));
   const auto y0 = static_cast<int>(floor( image_point.y ));
   const float tx = image_point.x - static_cast<float>(x0);
   const float ty = image_point.y - static_cast<float>(y0);
   const int x1 = min( x0 + 1, FloorImage.cols - 1 );
   const int y1 = min( y0 + 1, FloorImage.rows - 1 );

   const Vec3b* curr = FloorImage.ptr<Vec3b>(y0);
   const Vec3b* next = FloorImage.ptr<Vec3b>(y1);

   return Vec3b{
      static_cast<uchar>(
         static_cast<float>(curr[x0](0)) * (1.0f - tx) * (1.0f - ty) + static_cast<float>(curr[x1](0)) * tx * (1.0f - ty)
         + static_cast<float>(next[x0](0)) * (1.0f - tx) * ty + static_cast<float>(next[x1](0)) * tx * ty
      ),
      static_cast<uchar>(
         static_cast<float>(curr[x0](1)) * (1.0f - tx) * (1.0f - ty) + static_cast<float>(curr[x1](1)) * tx * (1.0f - ty)
         + static_cast<float>(next[x0](1)) * (1.0f - tx) * ty + static_cast<float>(next[x1](1)) * tx * ty
      ),
      static_cast<uchar>(
         static_cast<float>(curr[x0](2)) * (1.0f - tx) * (1.0f - ty) + static_cast<float>(curr[x1](2)) * tx * (1.0f - ty)
         + static_cast<float>(next[x0](2)) * (1.0f - tx) * ty + static_cast<float>(next[x1](2)) * tx * ty
      )
   };
}

void LocationDetection::renderCameraView(Camera& camera)
{
   for (int j = 0; j < camera.CameraView.rows; ++j) {
      auto* view_ptr = camera.CameraView.ptr<Vec3b>(j);
      Point2f valid_world_point;
      for (int i = 0; i < camera.CameraView.cols; ++i) {
         Point2f world_point;
         Point camera_point(i, j);
         float max_altitude = -numeric_limits<float>::infinity();
         for (const auto& zone : CustomizedZones) {
            if (transformCameraToWorld( world_point, camera_point, zone.Altitude, camera )) {
               if (isInsideZone( static_cast<Point>(world_point), zone.Zone )) {
                  if (max_altitude < zone.Altitude) {
                     max_altitude = zone.Altitude;
                     valid_world_point = world_point;
                  }
               }
            }
         }
         //if (transformCameraToWorld( world_point, camera_point, DefaultAltitude, camera )) {
         //   if (max_altitude < DefaultAltitude) {
         //      max_altitude = DefaultAltitude;
         //      valid_world_point = world_point;
         //   }
         //}
         if (isinf( max_altitude )) continue;

         view_ptr[i] = getPixelBilinearInterpolated( valid_world_point );
      }
   }
}

void LocationDetection::pickPointCallback(int evt, int x, int y, int flags, void* param)
{
   static bool update = false;

   if (evt == CV_EVENT_LBUTTONDOWN) {
      update = true;
      //ClickedPoint = Point(x, y);
   }
   else if (evt == CV_EVENT_MOUSEWHEEL) {
      //if (ClickedPoint.x >= 0) {
      //   update = true;
      //   if (flags & CV_EVENT_FLAG_CTRLKEY) updateFenceHeight( getMouseWheelDelta( flags ) );
      //   else updateFenceRadius( getMouseWheelDelta( flags ) );
      //}
   }

   if (update) {
      update = false;
      
      Mat viewer = static_cast<Mat*>(param)->clone();
      //render( viewer );
   }
}

void LocationDetection::pickPointCallbackWrapper(int evt, int x, int y, int flags, void* param)
{
   Instance->pickPointCallback( evt, x, y, flags, param );
}

void LocationDetection::generateEvent()
{
   for (auto& camera : LocalCameras) {
      renderCameraView( camera );
      imshow("cam" + to_string( camera.Index ), camera.CameraView );
   }
   waitKey();
   //
   //Mat viewer = WorldMap.clone();
   //namedWindow( "Location Detection", 1 );
   //imshow( "Location Detection", viewer );
   //setMouseCallback( "Location Detection", pickPointCallbackWrapper, &viewer );
   //waitKey();
   //destroyWindow( "Location Detection" );
}