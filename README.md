# LocationDetectionFromCCTV

  This code is about estimating a position of an world map from a local position of a camera and vice versa.
  
  Please refer to [this](emoy.net) to see the details.
  
  
  
## Test Environment
  * Windows 10
  * Visual Studio 2017
  * Surface Book 2
  
## Library Dependencies
  * OpenCV

## How to Set Event Zone
  1. Construct an instance of *LocationDetection class*.
  2. Set a polygon by clicking points on the 'Customizing Zones' window.
     The convex polygon is automatically set from points you clicked.
     * **Enter key**: complete a zone setting when it turns green, and input the altitude of this zone
     * **q key**: exit the setting zones
     
## How to Convert The World Map to The Camera
  * Call *generateEventOnWorldMap()*.
  * Click anywhere inside the 'Event Generation' window.
  * After clicking, the views of cameras which are already set are popped up.  
  
## How to Convert The Camera to The World Map
  * Call *generateEventOnCamera()*.
  * Click anywhere inside the 'Event Generation on Camera#<index>' window.
  * After clicking, the world map is popped up.  
