MultiWii
========

- Gyro
- Accelerometer
- Barometer
- Sonar (in I2C GPS NAV)
- Optical flow sensor (in I2C GPS NAV)
- GPS (in I2C GPS NAV)
- iPhone controlled (Connect to Bluetooth to serial module and then 3DR Radio)
- MSP_SET_RAW_RC changed to only 5 bytes to save bandwidth
- One byte MSP_SET_RAW_RC_REPEAT is sent when RC channels are not changed
