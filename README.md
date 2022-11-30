# DC Motor 
DC motor driver source code and schematics used in ECE 3709-L. Built using Megunolink, Arduino, and includes MATLAB files. 

## Materials
- L-298N Motor Drive
- DC Motor w/ Encoder
- Arduino Uno 

## Setup
- First set up physical circuit according to schematic. 
- Download and run Arduino code 
- Download and run meguno file with MegunoLink 
- Upload code to Arduino Uno 
- Begin serializing data in Meguno Link

## Notes
- When editing Arduino code, you must pause serializing data in MegunoLink, and then update the code with Arduino IDE, upload the code, then restart serializing data with MegunoLink. Only 1 device can be writing to a COM port at a time. 
- Whilst in MegunoLink, you may need to edit the viewpoint and play with the amount of zoom used. MegunoLink has a way of messing with views that can cause issues 
 
