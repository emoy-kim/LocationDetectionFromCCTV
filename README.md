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
  1. call *generateEvent()*.
  2. set a polygon by clicking points on the 'Customizing Zones' window.
     The convex polygon is automatically set from points you clicked.
     * **Enter key**: complete a zone setting and input the altitude of this zone
     * **q key**: exit the setting zones
     
## How to Convert The World Map to The Camera
  * after setting event zones, the 'Event Generation' window is created.
  * click anywhere inside the 'Event Generation' window.
  * after clicking, the views of cameras which are already set are popped up.  
