#include "LocationDetection.h"

void setCCTV1(LocationDetection& location_detector)
{
   const int camera_index = 1;
   const int width = 640;
   const int height = 480;
   const float focal_length = 500.0f;
   const float pan_angle_in_degree = 45.0f;
   const float tilt_angle_in_degree = 30.0f;
   const float camera_height_in_meter = 30.0f;
   const Point2f actual_position_in_meter(0.0f, 0.0f);
   location_detector.setCamera(
      camera_index,
      width, height,
      focal_length,
      pan_angle_in_degree,
      tilt_angle_in_degree,
      camera_height_in_meter,
      actual_position_in_meter
   );
}

void setCCTV2(LocationDetection& location_detector)
{
   const int camera_index = 2;
   const int width = 640;
   const int height = 480;
   const float focal_length = 500.0f;
   const float pan_angle_in_degree = -45.0f;
   const float tilt_angle_in_degree = 30.0f;
   const float camera_height_in_meter = 30.0f;
   const Point2f actual_position_in_meter(0.0f, 93.0f);
   location_detector.setCamera(
      camera_index,
      width, height,
      focal_length,
      pan_angle_in_degree,
      tilt_angle_in_degree,
      camera_height_in_meter,
      actual_position_in_meter
   );
}

void setCCTV3(LocationDetection& location_detector)
{
   const int camera_index = 3;
   const int width = 640;
   const int height = 480;
   const float focal_length = 500.0f;
   const float pan_angle_in_degree = -135.0f;
   const float tilt_angle_in_degree = 30.0f;
   const float camera_height_in_meter = 30.0f;
   const Point2f actual_position_in_meter(160.0f, 93.0f);
   location_detector.setCamera(
      camera_index,
      width, height,
      focal_length,
      pan_angle_in_degree,
      tilt_angle_in_degree,
      camera_height_in_meter,
      actual_position_in_meter
   );
}

void setCCTV4(LocationDetection& location_detector)
{
   const int camera_index = 4;
   const int width = 640;
   const int height = 480;
   const float focal_length = 500.0f;
   const float pan_angle_in_degree = 135.0f;
   const float tilt_angle_in_degree = 30.0f;
   const float camera_height_in_meter = 30.0f;
   const Point2f actual_position_in_meter(160.0f, 0.0f);
   location_detector.setCamera(
      camera_index,
      width, height,
      focal_length,
      pan_angle_in_degree,
      tilt_angle_in_degree,
      camera_height_in_meter,
      actual_position_in_meter
   );
}

int main()
{
   const float floor_width_in_meter = 160.0f;
   const float floor_height_in_meter = 93.0f;
   LocationDetection location_detector(floor_width_in_meter, floor_height_in_meter);

   setCCTV1( location_detector );
   setCCTV2( location_detector );
   setCCTV3( location_detector );
   setCCTV4( location_detector );

   location_detector.generateEvent();

   Point camera_point;
   const Point2f actual_point_in_meter(1.0f, 1.0f);
   location_detector.detectLocation( camera_point, 1, actual_point_in_meter );
   cout << camera_point << endl;

   return 0;
}