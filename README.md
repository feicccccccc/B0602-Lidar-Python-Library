# B0602-Lidar-Python-Library
A simple Python library for low cost B0602 lidar
Not optimize and so many problem with the hardware but feel free to have a try 😄

To use the library, you need the following library:
- PySerial

# Install PySerial with pip

pip install pyserial 

# Use the library 
Example:
```
from B0602Lidar import B0602Lidar

lidar = B0602Lidar()
while 1:
    print(lidar.run())
```
run() method return A frame represent by a dictionary <br />
- First key is angle , value is 20 data point [angle , distance in mm] in that scan
- Second key is 'r/s' , value is r/s
