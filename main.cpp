#include "LocationDetection.h"

void setCCTV1(LocationDetection& location_detector)
{
   constexpr int camera_index = 1;
   constexpr int width = 640;
   constexpr int height = 480;
   constexpr float focal_length = 500.0f;
   constexpr float pan_angle_in_degree = 45.0f;
   constexpr float tilt_angle_in_degree = 30.0f;
   constexpr float camera_height_in_meter = 50.0f;
   const cv::Point2f actual_position_in_meter(0.0f, 0.0f);
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
   constexpr int camera_index = 2;
   constexpr int width = 640;
   constexpr int height = 480;
   constexpr float focal_length = 500.0f;
   constexpr float pan_angle_in_degree = -45.0f;
   constexpr float tilt_angle_in_degree = 30.0f;
   constexpr float camera_height_in_meter = 50.0f;
   const cv::Point2f actual_position_in_meter(0.0f, 93.0f);
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
   constexpr int camera_index = 3;
   constexpr int width = 640;
   constexpr int height = 480;
   constexpr float focal_length = 500.0f;
   constexpr float pan_angle_in_degree = -135.0f;
   constexpr float tilt_angle_in_degree = 30.0f;
   constexpr float camera_height_in_meter = 50.0f;
   const cv::Point2f actual_position_in_meter(160.0f, 93.0f);
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
   constexpr int camera_index = 4;
   constexpr int width = 640;
   constexpr int height = 480;
   constexpr float focal_length = 500.0f;
   constexpr float pan_angle_in_degree = 135.0f;
   constexpr float tilt_angle_in_degree = 30.0f;
   constexpr float camera_height_in_meter = 50.0f;
   const cv::Point2f actual_position_in_meter(160.0f, 0.0f);
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

void testReprojection(LocationDetection& location_detector)
{
   cv::Point camera_point;
   const cv::Point2f actual_point_in_meter(10.0f, 10.0f);
   location_detector.detectLocation( camera_point, 1, actual_point_in_meter );
   std::cout << "\nActual Point " << actual_point_in_meter << "(in meter) is projected on " << camera_point << " in Camera#1\n";

   cv::Point2f reprojected;
   location_detector.detectLocation( reprojected, camera_point, 1 );
   std::cout << "The projected point " << camera_point << " is reprojected on " << reprojected << "(in meter)\n";
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

   location_detector.generateEventOnWorldMap();

   location_detector.generateEventOnCamera( 1 );

   testReprojection( location_detector );
   return 0;
}