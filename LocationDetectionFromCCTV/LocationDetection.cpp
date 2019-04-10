#include "LocationDetection.h"

LocationDetection* LocationDetection::Instance = nullptr;
LocationDetection::LocationDetection(const float& actual_width, const float& actual_height, const vector<CustomizedZone>& zones) :
   ActualFloorWidth( actual_width ), ActualFloorHeight( actual_height ), DefaultAltitude( 1.0f )
{
   Instance = this;

   FloorImage = imread( "floor.jpg" );
   if (!FloorImage.empty()) {
      MeterToPixel = static_cast<float>(FloorImage.cols) / ActualFloorWidth;
      if (zones.empty()) customizeZones();
      else CustomizedZones = zones;
   }
   else cout << "Cannot Load the Image...\n";
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

      if (key == 13 /* Enter */) {
         float altitude;
         cout << ">> Enter the Altitude in Meter.\n";
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
   const Point3f origin_vector(0.0f, 0.0f, 135.0f);
   const Scalar color(87, 7, 228);

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
   arrowedLine( FloorImage, camera_in_world, camera_direction, color, 4 );

   const Point3f left_fov = tilt_inv * fov_pos * pan_inv * origin_vector;
   const Point3f right_fov = tilt_inv * fov_neg * pan_inv * origin_vector;
   const Point left_direction = camera_in_world + Point(
      static_cast<int>(round( left_fov.z )), static_cast<int>(round( left_fov.x ))
   );
   const Point right_direction = camera_in_world + Point(
      static_cast<int>(round( right_fov.z )), static_cast<int>(round( right_fov.x ))
   );
   line( FloorImage, camera_in_world, left_direction, color, 3 );
   line( FloorImage, camera_in_world, right_direction, color, 3 );
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
   LocalCameras.emplace_back( camera );
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

bool LocationDetection::canSeePointOnDefaultAltitude(Point2f& world_point, const Point& camera_point, const Camera& camera)
{
   if (transformCameraToWorld( world_point, camera_point, DefaultAltitude, camera )) {
      for (const auto& zone : CustomizedZones) {
         if (isInsideZone( static_cast<Point>(world_point), zone.Zone )) {
            return false;            
         }
      }
      return true;
   }
   return false;
}

bool LocationDetection::getValidWorldPointFromCamera(Point2f& valid_world_point, const Point& camera_point, const Camera& camera)
{
   float max_altitude = canSeePointOnDefaultAltitude( valid_world_point, camera_point, camera ) ?
         DefaultAltitude : -numeric_limits<float>::infinity();
   
   Point2f world_point;
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
   return !isinf( max_altitude );
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
         static_cast<float>(curr[x0](0)) * (1.0f - tx) * (1.0f - ty) + static_cast<float>(curr[x1](0)) * tx * (1.0f - ty) +
         static_cast<float>(next[x0](0)) * (1.0f - tx) * ty + static_cast<float>(next[x1](0)) * tx * ty
      ),
      static_cast<uchar>(
         static_cast<float>(curr[x0](1)) * (1.0f - tx) * (1.0f - ty) + static_cast<float>(curr[x1](1)) * tx * (1.0f - ty) +
         static_cast<float>(next[x0](1)) * (1.0f - tx) * ty + static_cast<float>(next[x1](1)) * tx * ty
      ),
      static_cast<uchar>(
         static_cast<float>(curr[x0](2)) * (1.0f - tx) * (1.0f - ty) + static_cast<float>(curr[x1](2)) * tx * (1.0f - ty) +
         static_cast<float>(next[x0](2)) * (1.0f - tx) * ty + static_cast<float>(next[x1](2)) * tx * ty
      )
   };
}

void LocationDetection::renderCameraView(Camera& camera)
{
   Point2f valid_world_point;
   for (int j = 0; j < camera.CameraView.rows; ++j) {
      auto* view_ptr = camera.CameraView.ptr<Vec3b>(j);
      for (int i = 0; i < camera.CameraView.cols; ++i) {
         const Point camera_point(i, j);
         if (getValidWorldPointFromCamera( valid_world_point, camera_point, camera )) {
            view_ptr[i] = getPixelBilinearInterpolated( valid_world_point );
         }
      }
   }
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

void LocationDetection::pickPointOnWorldMapCallback(int evt, int x, int y, int flags, void* param)
{
   if (evt == CV_EVENT_LBUTTONDOWN) {
      Mat viewer = static_cast<Mat*>(param)->clone();
      const Point world_point(x, y);
      circle( viewer, world_point, 10, RED_COLOR, -1 );

      const Point2f actual_point_in_meter(static_cast<float>(x) / MeterToPixel, static_cast<float>(y) / MeterToPixel );
      cout << "\n>> Event Location Generated on World Map: " << actual_point_in_meter << " (in meter)\n";

      float max_altitude = DefaultAltitude;
      for (const auto& zone : CustomizedZones) {
         if (isInsideZone( world_point, zone.Zone )) {
            if (max_altitude < zone.Altitude) {
               max_altitude = zone.Altitude;
            }
         }
      }

      cout << ">> Event Location Information in Each Camera:\n";
      for (auto& camera : LocalCameras) {
         Point camera_point;
         transformWorldToCamera( camera_point, world_point, max_altitude, camera );

         Mat camera_view = camera.CameraView.clone();
         circle( camera_view, camera_point, 5, RED_COLOR, -1 );
         imshow( "Camera#" + to_string( camera.Index ), camera_view );
         cout << ">> \t- Camera#" << to_string( camera.Index ) << " " << camera_point << endl;
      }
      imshow( "Event Generation", viewer );
   }
}

void LocationDetection::pickPointOnWorldMapCallbackWrapper(int evt, int x, int y, int flags, void* param)
{
   Instance->pickPointOnWorldMapCallback( evt, x, y, flags, param );
}

void LocationDetection::generateEventOnWorldMap()
{
   for (auto& camera : LocalCameras) {
      renderCameraView( camera );
      renderZonesInCamera( camera );
   }

   Mat viewer = FloorImage.clone();
   namedWindow( "Event Generation", 0 );
   resizeWindow( "Event Generation", FloorImage.cols / 3, FloorImage.rows / 3 );
   imshow( "Event Generation", viewer );
   setMouseCallback( "Event Generation", pickPointOnWorldMapCallbackWrapper, &viewer );
   waitKey();
   destroyAllWindows();
}

void LocationDetection::pickPointOnCameraCallback(int evt, int x, int y, int flags, void* param)
{
   if (evt == CV_EVENT_LBUTTONDOWN) {
      auto* camera = static_cast<Camera*>(param);
      Mat viewer = camera->CameraView.clone();
      
      Point2f valid_world_point;
      const Point camera_point(x, y);
      if (getValidWorldPointFromCamera( valid_world_point, camera_point, *camera )) {
         cout << "\n>> Event Location Generated on Camera#" << camera->Index << ": " << camera_point << "\n";
         circle( viewer, camera_point, 5, RED_COLOR, -1 );

         const Point2f actual_point_in_meter( valid_world_point.x / MeterToPixel, valid_world_point.y / MeterToPixel );
         cout << ">> Event Location on World Map: " << actual_point_in_meter << " (in meter)\n";

         Mat world_map = FloorImage.clone();
         circle( world_map, valid_world_point, 10, RED_COLOR, -1 );
         namedWindow( "World Map", 0 );
         resizeWindow( "World Map", world_map.cols / 3, world_map.rows / 3 );
         imshow( "World Map", world_map );
      }
      imshow( "Event Generation on Camera#" + to_string( camera->Index ), viewer );
   }
}

void LocationDetection::pickPointOnCameraCallbackWrapper(int evt, int x, int y, int flags, void* param)
{
   Instance->pickPointOnCameraCallback( evt, x, y, flags, param );
}

void LocationDetection::generateEventOnCamera(const int& camera_index)
{
   const auto camera = find_if( 
      LocalCameras.begin(), LocalCameras.end(), 
      [camera_index](const Camera& cam)
      {
         return cam.Index == camera_index;
      }
   );
   if (camera == LocalCameras.end()) return;

   renderCameraView( *camera );
   renderZonesInCamera( *camera );
   
   imshow( "Event Generation on Camera#" + to_string( camera->Index ), camera->CameraView );
   setMouseCallback( "Event Generation on Camera#" + to_string( camera->Index ), pickPointOnCameraCallbackWrapper, &*camera );
   waitKey();
   destroyAllWindows();
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

void LocationDetection::detectLocation(Point2f& actual_position_in_meter, const Point& camera_point, const int& camera_index)
{
   actual_position_in_meter = { -1.0f, -1.0f };
   if (static_cast<int>(LocalCameras.size()) <= camera_index) return;
   
   Point2f valid_world_point;
   Camera& camera = LocalCameras[camera_index];
   if (getValidWorldPointFromCamera( valid_world_point, camera_point, camera )) {
      actual_position_in_meter.x = valid_world_point.x / MeterToPixel;
      actual_position_in_meter.y = valid_world_point.y / MeterToPixel;
   }
}