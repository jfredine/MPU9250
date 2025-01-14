# MPU9250
This is example code for use of an MPU9250 9-DoF accelerometer, gyro,
and magnetometer.  It follows the steps layed out in the [Adafruit AHRS Motion
Sensor Fusion Overview](https://learn.adafruit.com/how-to-fuse-motion-sensor-data-into-ahrs-orientation-euler-quaternions/overview)
with the code for both calibration and AHRS steps included.  Failure to do the
magnetic calibration will result in very poor performance so do not skip this
step.

The macro CALIBRATION should be set during compilation to complete the
calibration step and once calibration is complete the code should be updated
with the adjustment data (look for sensor_adj_t) and build again without this
macro to output orientation data usable with the
[Adafruit 3D Model Viewer](https://adafruit.github.io/Adafruit_WebSerial_3DModelViewer/)

It does not rely on any Adafruit libraries making it potentially easier to port
to other platforms where that does not exist.  There are two libraries self
contained in the repository.  One is for the Madgwick and Mahony AHRS filters.
The second is for interfacing with the actual MPU9250.
