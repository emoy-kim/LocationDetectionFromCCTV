if(${CMAKE_BUILD_TYPE} MATCHES Debug)
   target_link_libraries(LocationDetectionFromCCTV opencv_cored opencv_imgprocd opencv_imgcodecsd opencv_highguid)
else()
   target_link_libraries(LocationDetectionFromCCTV opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui)
endif()