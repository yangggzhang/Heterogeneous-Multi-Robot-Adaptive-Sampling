# Temp
This package is a ROS wrapper for ```temper.py``` (https://github.com/urwen/temper). The package reads and publish the data given by the "TEMPer" temperature sensor. 

## Read and Publish Data
The following command launch a rosnode named ```node_temper```, which receive temperature reading from the sensor and publish to the topic ```/temp``` with the corresponding timestamp.
```
roslaunch temp temp_launch.launch
```
The file ```temp_sub.py``` shows an example subscriber. To see received messages, with the ```node_temper``` node running, run
```
cd src/
python temp_sub.py
```

## Published Topic
```
/temp (Temperature.msg)
```

## Troubleshooting
In case of permission denied, run:
```
chmod 0666 /dev/hidraw*
```
