# Sensor Fusion
*17 November 2017*

This project concerns utilizing a gryoscope and an accelerometer in combination to obtain a more accurate representation of orientation.  

Three unit vectors representing "true up" relative to the current orientation of the IMU were created from the accelerometer readings, gyroscope readings, and a combination of the two. The gyroscope vector drifts while the accelerometer vector is accurate in the long term but more sensitive and subject to innacuracy in high frequency, short term readings. By utilizing both the gyroscope and the accelerometer, an accurate reading in both the short and long term is achievable.  

The spec for this project can be found in [spec.pdf](https://github.com/rwgriffithv/IEEE-Advanced-Projects/blob/master/Sensor-Fusion/spec.pdf) and the final arduino code is within the folder entitled [normalVectorFusion](https://github.com/rwgriffithv/IEEE-Advanced-Projects/tree/master/Sensor-Fusion/normalVectorFusion).  
The written file [normalVectorFusion.ino](https://github.com/rwgriffithv/IEEE-Advanced-Projects/blob/master/Sensor-Fusion/normalVectorFusion/normalVectorFusion.ino) will print the components of each vector on one line, which can be processed and visualized using the application visualizer.exe which was provided by the IEEE AP project leads.  
The executable has been uploaded to this [drive](https://drive.google.com/open?id=1DWbbnri0LaV_kIbrMANC7wqFKkcLSzZK).

**Collaborators:**  
[Brian Dionigi](https://github.com/il-dionigi) | [Kevin Ke-En Sun](https://github.com/inherentlyMalicious) | [Koji Kusumi](https://github.com/kojiboji) | [Rob Griffith](https://github.com/rwgriffithv)