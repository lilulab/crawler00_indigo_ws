Overview
========

This package provides a [ROS][] driver for [PMD[vision]® CamBoard nano][PMD]
depth sensor.

The driver is packaged as a nodelet, therefore it may be directly merged inside
another ROS node to avoid unnecessary data copying. On the same time, it may be
started standalone or withing a nodelet manager. The distance, depth (see
"Distance vs. depth images" section), amplitude, and point cloud data are
retrieved from the device and processed only if there are subscribers on the
corresponding topics.

The `pmd_camboard_nano.launch` script (inspired by the [openni_launch][] stack
in ROS) starts the driver nodelet along with the image rectification nodelets.

Installation
============

PMD SDK installation
--------------------

This package requires PMD SDK to be installed in the system. It will search
for:

* header files in `/usr/local/pmd/include`
* shared library in `/usr/local/pmd/bin`
* plugins in `/usr/local/pmd/plugins`

Alternatively, you could put the `include`, `bin`, and `plugins` folders
elsewhere in your file system and set an environment variable `${PMDDIR}` to
point to their location.

You also need to copy the file `10-pmd.rules` provided with the SDK to
`/etc/udev/rules.d` to allow normal users to open the camera.

Package installation
--------------------

Clone this repository into a folder that is in your `$ROS_PACKAGE_PATH` and run
`rosmake pmd_camboard_nano`.

ROS API
=======

pmd_camboard_nano::DriverNodelet
--------------------------------

### Published topics

* `distance/camera_info` (*sensor_msgs/CameraInfo*)  
  camera calibration and metadata (see "Camera calibration" section)

* `distance/image` (*sensor_msgs/Image*)  
  raw distances from the optical center of the device to scene points, contains
  `float` distances (mm)

* `depth/camera_info` (*sensor_msgs/CameraInfo*)  
  camera calibration and metadata (see "Camera calibration" section)

* `depth/image` (*sensor_msgs/Image*)  
  depths of scene points (distances along the camera optical axis) from the
  device, contains `float` depths in mm

* `amplitude/camera_info` (*sensor_msgs/CameraInfo*)  
  camera calibration and metadata (see "Camera calibration" section)

* `amplitude/image` (*sensor_msgs/Image*)  
  signal strengths of active illumination

* `points_unrectified` (*sensor_msgs/PointCloud2*)  
  3D point cloud generated by the camera driver

### Parameters

* `~device_serial` (default: "")  
  specifies which device to open, empty means any

* `~calibration_file`  
  path to the file with camera calibration data

* `~frame_id` (default: "/camera_optical_frame")  
  the tf frame of the camera

* `~open_camera_retry_period` (default: 3)  
  how often (seconds) to try to open camera during the startup

* `~update_rate` (default: 30)  
  how often (Hz) to download and publish new data from the camera

* `~flip_vertical` (default: true)  
  flip the output images/point clouds vertically, so that the first row is
  swapped with the last and so on

### Dynamically reconfigurable parameters

Use the [dynamic_reconfigure][] package to update these parameters in runtime:

* `~remove_invalid_pixels` (default: true)  
  replace invalid pixels in depth and amplitude images with NaNs

* `~integration_time` (default: 333)  
  integration time of the camera in us

* `~averaging_frames` (default: 0)  
  number of frames in sliding averaging window for distance data

* `~signal_strength_check` (default: true)  
  activate signal strength check

* `~signal_strength_threshold` (default: 200)  
  if the signal strength is below this threshold, the pixel is marked as invalid

* `~consistency_check` (default: true)  
  activate consistency check

* `~consistency_check_threshold` (default: 0.98)  
  if the consistency value of a pixel is below this threshold, it is marked as
  invalid

* `~bilateral_filter` (default: true)  
  enable/disable bilateral filtering of the depth images

* `~sigma_spatial` (default: 2.5)  
  spatial sigma parameter of the bilateral filter

* `~sigma_range` (default: 25)  
  range sigma parameter of the bilateral filter

* `~kernel_size` (default: 5)  
  kernel size parameter of the bilateral filter

* `~bilateral_filter_enhance_image` (default: false)  
  activate enhanced bilateral filtering (increases robustness against motion
  blur)

Misc
====

Camera calibration
------------------

By default the PMD plugin loads the calibration data from a file (provided with
the camera), which must be located within the working directory of the
application and have a name composed of the device serial number and *".dat"*
extension. If you are using the `pmd_camboard_nano.launch` file, the working
directory of the driver nodelet will be `~/.ros`. You therefore have to have a
copy of the calibration file there.

Alternatively, you can specify the location of the calibration data file as a
parameter of the nodelet (`~calibration_file`).

If the PMD plugin failed to load the calibration data, then the camera info
messages produced by the driver nodelet will be filled with the values that
*seem* to be "default" (see [this forum topic][calibration_forum_topic]).


Distance vs. depth images
-------------------------

The distance data provided by the PMD SDK driver is actually the distances from
the optical center of the camera to the scene points. Other cameras (e.g.
Microsoft Kinect) output depth maps that are composed of distances from the
cameras principal plane to the scene points along the optical axis. In other
words, their depth image consists of z-coordinates of the scene points in the
cameras coordinate frame.

This driver publishes both distance images (as output by the PMD SDK driver),
and "Kinect-style" depth images, computed by multiplying the distances with the
corresponding direction vectors.

Compatibility
-------------

This package was tested under Ubuntu Precise x64 with ROS Fuerte and under
Ubuntu Oneiric x64 with ROS Electric. The version of PMD SDK is 1.3.2.

Known issues
------------

This package was tested on multiple PCs and generally worked fine, however on
one Lenovo laptop the following problems were observed:

* RViz crashed when trying to display the messages in the `/camera/points`
  topic.
  Workaround: set display style **NOT** to Points, e.g. to BillboardSpheres.

* While adjusting the parameters with dynamic reconfigure GUI the driver nodelet
  freezed and sometimes even died.

[ROS]: http://www.ros.org
[PMD]: http://www.pmdtec.com/products_services/reference_design.php
[openni_launch]: http://ros.org/wiki/openni_launch
[dynamic_reconfigure]: http://ros.org/wiki/dynamic_reconfigure
[calibration_forum_topic]: https://www.cayim.com/forum/index.php?/topic/33-intrinsics-and-calibration/#entry125