
![](Ultrasonic\_Sensor\_SS2018.001.png)

Scientific Project Summer Semester ![](Ultrasonic\_Sensor\_SS2018.002.png)2018 

Ultrasonic Sensor Measurement Setup 

Under the guidance 

of 

Prof. Dr. Robert Hönl & Prof. Dr. katrin Skerl 

Report  

By  

Amol Hari Jadhav (258342) Phanindra Kumar Yellapu (258344) 

Contents 



|S.no |Topic |Page |
| - | - | - |
|1 |Introduction |3 |
|2 |Ultrasonic Sensor |3 |
|3 |Mechanical Experimental Setup |4 |
|4 |Electronic Components |8 |
|5 |Calculations |13 |
|6 |Measurements |14 |
|7 |Observations |16 |
|8 |Future scope |17 |
|9 |Appendix |18 |
Table of figures 



|Figure Number |Topic |
| - | - |
|1 |Ultrasonic Sensor |
|2 |Initial Design |
|3 |Realised Ultrasonic Design |
|4 |Vertical Stands on both ends |
|5 |Centre Equipment Holder |
|6 |Motor shaft and belt |
|7 |Mechanical Design for assembly |
|8 |Top View of Setup |
|9 |Side View |
|10 |Electrical setup made in Proteus 8.0 software |
|11 |HC-SR04 ultrasonic sensor |
|12 |Working principle of HC-SR04 |
|13 |Arduino Board |
|14 |Adafruit Motor shield v2.3 |
|15 |stepper Wiring Diagram |
|16 |Max range of sensor |
|17 |Object at 30cm distance |
|18 |Data mapped in Matlab |
|19 |Object at 70 cm |
|20 |Result with wood at 70 cm |
|21 |Class experiment |
|22 |Measurement range of 3cm to 400 cm |
|23 |Measurement range of 3 cm to 100 cm |
Introduction: ![](Ultrasonic\_Sensor\_SS2018.003.png)

Our aim for the scientific project was to develop a measurement setup that emits and receives ultrasound. The received data should be processed to a 1D ultrasound image. In addition, the influence of the control of the ultrasonic transmitter on the measured data is to be investigated. 

Hence first we started with the basics of ultrasonic sensor its working principle and then we implemented the basic application of ultrasonic sensor for sensing distance with the help of Arduino and serial monitor. We displayed the distance real time in the serial monitor. 

As per the problem statement, we need to now develop a measurement setup which can map sensor data. For this, we finalized on a linear guide system with Arduino as a slave and MATLAB as the main controller. 

Ultrasonic Sensor: 

Ultrasonic sensors emit short, high-frequency sound pulses at regular intervals. These propagate in the air at the velocity of sound. If they strike an object, then they are reflected as echo signals to the sensor, which itself computes the distance to the target based on the time- span between emitting the signal and receiving the echo. [4] 

As the distance to an object is determined by measuring the time of flight and not by the  intensity  of  the  sound,  ultrasonic  sensors  are  excellent  at  suppressing  background interference. Virtually all materials which reflect sound can be detected, regardless of their colour. Even transparent materials or thin foils represent no problem for an ultrasonic sensor. Ultrasonic sensors can see through dust-laden air and ink mists. [4]  

Even thin deposits on the sensor membrane do not impair its function. Sensors with a blind zone of only 20  mm and an extremely thin beam  spread are making entirely new applications possible today: Fill level measurement in wells of microtiter plates and test tubes, as well as the detection of small bottles in the packaging industry, can be implemented with ease. Even thin wires are reliably detected. [4]

![](Ultrasonic\_Sensor\_SS2018.004.png)

*Figure 1 : Ultrasonic Sensor [4]* 

The sensor was studied through the project by a combined effort of mechanical, electronics and software as explained further. ![](Ultrasonic\_Sensor\_SS2018.005.png)

Mechanical Experimental Setup: 

We selected a linear axis movement system so that the system can move automatically without any human errors or disturbance and the sensor can be studied more precisely and accurately. The idea of this system is to move it linearly in a single direction over a distance and to study the objects placed in its vicinity. The objects distance from this system and width of the object can be recorded with the help of ultrasonic sensor placed on the movable body. 

Another advantage of this design is that we can study other sensors also by replacing the sensor and the respective program in Matlab. This design also gives a liberty to extend the observable area or study area by unbolting the ends and extending the linear guides. 

In the initial phase of the project we have designed a single axis linear motion guide with the idea of using lead screw as shown below. The detachable holder with the ultrasonic sensor and Arduino setup can be placed over the moving unit in the above shown setup.  

![](Ultrasonic\_Sensor\_SS2018.006.png)

*Figure 2: Initial Design* 

To have smooth movement such that the vibrations can be minimised and does not affect the study of sensor, the lead screw system was replaced with a pulley and belt system. 

In the below figure, it can be noticed that the centre body is elevated to 5mm with respect to the corner holding stands. This is to allow the centre ![](Ultrasonic\_Sensor\_SS2018.007.png)body to move freely. The idea of this setup is to pull the centre body from one end to another end with the help of a belt and a pulley system. Belt is hinged to the centre body. 

![](Ultrasonic\_Sensor\_SS2018.008.png)

*Figure 3 : Realised Ultrasonic Design* 

The setup consists of three main mechanical parts.

1. Vertical Holding stands and linear guides 

These stands are on both the corners of the setup and are heavy to give enough strength to the system such that the vibrations are absorbed, and the entire system stays stable on a surface. These stands are further drilled on 4 places so that the linear guides can be inserted into them and be bolted on the outer corners. There is also a flat plate on both the ends to mount the pulley and motor.  

The pulley is mounted and free to rotate with the help of a ball bearing. The dimensions of these stands and bearings are provided in the design files that are attached in the appendix. 

![](Ultrasonic\_Sensor\_SS2018.009.png)

*Figure 4 : Vertical Stands on both ends*

2. Centre Holding body ![](Ultrasonic\_Sensor\_SS2018.010.png)

The centre body is to facilitate free movement for the sensor. This body lies on top of 4 linear guides. It has linear bearings to provide frictionless movement and is connected to the belt on both the ends which provide a pull force allowing for a movement in both the directions. The sensor along with Arduino and bread board for ultrasonic sensor are mounted on top of this setup. The entire metal frame holding the sensor and Arduino setup can be detached from the centre body by unbolting. It is feasible to use the sensor setup alone without the entire system. Just by unbolting two screws the sensor setup can be unmounted from the body. 

![](Ultrasonic\_Sensor\_SS2018.011.png)

*Figure 5 : Centre Equipment Holder*

3. Belt and pulley system 

The belt is connected to both the ends of the centre of the body and the belt passes through the pulley and motor shaft thereby forming a closed-circuit system.  By pulling one of the ends of the body, it enables the body to travel in that direction. Nema 17 Stepper motor is used as it can provide a torque of 1.6 Kg / cm. The centre body weight around 700 grams and the torque provided by the motor is enough to move the body linearly. 

![](Ultrasonic\_Sensor\_SS2018.012.png)

*Figure 6 : Motor shaft and belt [5]*

The system was designed in SolidWorks and realised in workshop. All the files for individual tiles and the detail list of parts are shared in appendix. ![](Ultrasonic\_Sensor\_SS2018.013.png)

Solid Works Design: 

![](Ultrasonic\_Sensor\_SS2018.014.png)

*Figure 7 : Mechanical Design for assembly* Realised Setup: 

Top View  Side View 

![](Ultrasonic\_Sensor\_SS2018.015.png)![](Ultrasonic\_Sensor\_SS2018.016.png)

*Figure 8 : Top View of Setup  Figure 9 : Side View* 

Electronic Components: ![](Ultrasonic\_Sensor\_SS2018.017.png)

We have used MATLAB platform to run our setup. Hence the Arduino Uno is used as a passive board. The electrical setup is as shown below. 

![](Ultrasonic\_Sensor\_SS2018.018.png)

*Figure 10 : Electrical setup made in Proteus 8.0 software.* 

The Electronic components used are: Ultrasonic sensor: 

Here, we are using the HC-SR04 ultrasonic sensor. The big advantage of this sensor its available easily and very cheap compared to other ultrasonic sensors. 

![](Ultrasonic\_Sensor\_SS2018.019.png)

*Figure 11 : HC-SR04 ultrasonic sensor* 

Specifications:



|Power Supply |+5V DC |
| - | - |
|Quiescent Current |<2mA |
|Working Current |15mA |
|Effectual Angle |<15 Degree |
|Ranging Distance |2-400 cm |
|Resolution |0.3 cm |
|Measuring Angle |30 Degree |
|Trigger Input Pulse width |10 s |
|Dimension |45mm x 20mm x 15mm |
|Weight approx |10 g |
Ultrasonic sensor has a transmitter and a receiver in one module. The transmitter has a piezo which vibrates at a frequency of 45 kHz. The working principle of this module is as shown in figure 1. The ultrasonic sensor is trigger by making Trigger pin high for 10us. Then 8 pulses of 40kHz is send and the echo is received by the receiver part of sensor. 

![](Ultrasonic\_Sensor\_SS2018.020.png)![](Ultrasonic\_Sensor\_SS2018.021.png)

*Figure 12 : Working principle of HC-SR04 [1]* 

As discussed earlier, Ultrasonic sensor is used to detect the distance of an object. This is done by time of flight concept. Here we have a proportional relationship between time and distance.  

The calculation of distance is done by the formula shown below. 

[  ∗ ( . . )]![](Ultrasonic\_Sensor\_SS2018.022.png)

\=

2

Where, 

\=

- (343 ) 20℃ [2] ![](Ultrasonic\_Sensor\_SS2018.023.png)

. . =

Arduino Uno: 

We established a communication between Arduino Uno and MATLAB. We first need to install Hardware support packages for Arduino Uno, HC-SR04 sensor. Once the Hardware packages are installed we can establish MATLAB and Arduino communication. Please see the appendix for the MATLAB code. 

Arduino Uno is a microcontroller board based on the ATmega328P. It has 14 digital input/output pins (of which 6 can be used as PWM outputs), 6 analogue inputs, a 16 MHz quartz crystal, a USB connection, a power jack, an ICSP header and a reset button. It contains everything needed to support the microcontroller; simply connect it to a computer with a USB cable or power it with an AC-to-DC adapter or battery to get started. [3] 

![](Ultrasonic\_Sensor\_SS2018.024.png)![](Ultrasonic\_Sensor\_SS2018.025.png)

*Figure 13 : Aurdino Board [6]*

Specifications [4]: 



|Microcontroller |ATmega328P |
| - | - |
|Operating voltage |5V |
|Number of Analog input pins |6 |
|Flash memory |32KB |
|Clock speed |16Mhz |
|DC current for 3.3v pin |50 mA |
Adafruit Motor shield v2.3: ![](Ultrasonic\_Sensor\_SS2018.026.png)

The Motor shield is used as a driver for stepper motor. The stepper motor needs 0.33A of current to run at 12V DC. The Motor shield provides a 1.2A peak current per phase. We can connect 2 stepper motors with this shield at the same time. We have connected our Stepper motor at M1 and M2 as it’s a bipolar stepper motor. 

![](Ultrasonic\_Sensor\_SS2018.027.png)

*Figure 14 : Adafruit Motor shield v2.3 [3]* 

Specifications [3] : 

- 2 connections for 5V 'hobby' servos connected to the Arduino's high-resolution dedicated timer - no jitter! 
- 4 H-Bridges: TB6612 chipset provides 1.2A per bridge (3A for brief 20ms peaks) with thermal shutdown protection, internal kickback protection diodes. Can run motors on 4.5VDC to 13.5VDC. 
- Up to 4 bi-directional DC motors with individual 8-bit speed selection (so, about 0.5% resolution) 
- Up to 2 stepper motors (unipolar or bipolar) with single coil, double coil, interleaved or micro-stepping. 
- Motors automatically disabled on power-up 
- Big terminal block connectors to easily hook up wires (18-26AWG) and power 
- Arduino reset button brought up top 
- Polarity protected 2-pin terminal block and jumper to connect external power, for separate logic/motor supplies 
- Tested compatible with Arduino UNO, Leonardo, ADK/Mega R3, Diecimila & Duemilanove. Works with Due with 3.3v logic jumper. Works with Mega/ADK R2 and earlier with 2 wire jumpers. 
- 5v or 3.3v compatible logic levels - jumper configurable. 

Stepper motor: 

We have used a NEMA 17 stepper motor with 1.8-degree angle per step. The electrical specification of stepper motor is as follows: 



|Step angle |1.8 degree |
| - | - |
|Phases |2 |
|Insulation Resistance |100Mohms (500V DC) |
|Class of Insulation |B |
|Weight |0.20 Kg |
|Voltage |12V |
|Current |0.33A |
|Holding torque |0.23 Nm |
The stepper motor has 2 coils and it’s a 4-wire motor. It is wired to Motor shield as shown below. 

![](Ultrasonic\_Sensor\_SS2018.028.png)![](Ultrasonic\_Sensor\_SS2018.029.png)

*Figure 15 : stepper Wiring Diagram* 

Power Supply: 

We need power to run the entire setup. 5v DC power to run the Arduino and 12V to power the shield which provides the required voltage and current for the motors. 

The Arduino is powered with the help of a USB cable and as we track live data, it can be controlled with Matlab. The Arduino acts as a slave to Matlab. The Shield is provided with external power source with the help of an adapter. These both power sources are enough to run the entire setup and collect data from the sensor.

Calculations: ![](Ultrasonic\_Sensor\_SS2018.030.png)

Before executing the program to run the motor, we need to do following calculations. 

- Steps Per mm: 

This will help us estimate how many steps our motor must travel to achieve 1mm practically. 

For Belt and Pulley Setup: 

Steps / mm  =  ( ) ∗( ) 

( − ℎ) ∗( ℎ)

We are using GT2 belt with a pulley having 20 teeth. Motor Steps per rev = 200 

Driver micro stepping = 1 

Belt pitch = 2mm 

Pulley Teeth = 20  

Steps / mm =   200 ∗1   =5 

(2) ∗(20)

For loops in Matlab = (Steps/mm) \* (distance) = 5 \* 450 = 2250 

- Maximum speed of motor: 

Max Speed = ( ∗ 60)/( ∗ 2 ∗ ∗ ) 

- 12 \* 60 / (1.2 \* 2 \* 0.33 \* 200) 
- 4.5 mm / s 

Note: In Matlab code, number of loops is 450 and the linear displacement given is 5 which results in 2250 as per our calculation above. 

Range of sensor: 

![](Ultrasonic\_Sensor\_SS2018.031.png)

*Figure 16 : Max range of sensor* 

Measurements: ![](Ultrasonic\_Sensor\_SS2018.032.png)

1. Experiment with a metal stand (30 cm distance): 

![](Ultrasonic\_Sensor\_SS2018.033.png)

*Figure 17: Object at 30cm distance.* Result: 

![](Ultrasonic\_Sensor\_SS2018.034.png)

*Figure 18 : Data mapped in Matlab* 

The above plot was mapped in Matlab and it can be seen the distance to be approximately 30 cm.

2. Experiment with a wooden stand (70 cm distance): ![](Ultrasonic\_Sensor\_SS2018.035.png)

![](Ultrasonic\_Sensor\_SS2018.036.png)

*Figure 19 : Object at 70 cm* Results: 

![](Ultrasonic\_Sensor\_SS2018.037.png)

*Figure 20 : Result with wood at 70 cm*

It can be seen in the above figure that wood is placed exactly at 70 cm and the output graph is also nearby to 70 cm. 

3. Experiment in class during presentation with professor Hoenl: 

![](Ultrasonic\_Sensor\_SS2018.038.png)![](Ultrasonic\_Sensor\_SS2018.039.png)

*Figure 21 : Class experiment* Observations: 

- Measurement with different ranges: 



|![](Ultrasonic\_Sensor\_SS2018.040.png)||![](Ultrasonic\_Sensor\_SS2018.041.png)|
| - | :- | - |
*Figure 22 : Measurement range of 3cm to  Figure 23  : Measurement range of 3 cm to 400 cm  100 cm* 

- The ultrasonic sensor detects all the objects in this range at a beam angle of 15 degrees. Hence the object width calculation is a tricky job with this ultrasonic sensor. 
- As per the time of flight concept the measured distance is directly proportional to the time. Time is inversely proportional to frequency. Hence by varying the frequency of the piezo crystal the range of ultrasonic sensor will be varied. 
- Ultrasonic sensor can give false echoes of object places at different distance. This can be solved ![](Ultrasonic\_Sensor\_SS2018.002.png)by using the ultrasonic sensor with low frequency by which the false echoes will diminished before reaching the receiver. 
- Objects which are place at an angle will cause the deflection of echoes. Hence the objects should always be present perpendicular. 
- There is also an effect of temperature. As temperature increases the detecting range decreases. Hence in some of the commercial products they have temperature compensation circuit available. 

Future scope: 

- The communication can be made wireless. 
- With the existing setup we can plug other sensor for distance measurement and mapping environment. Sensors with high accuracy and less beam angle. 

Software used: 

- MATLAB R2017b 
- SolidWorks 2018 
- Proteus 8.0 
- Arduino IDE 1.8.5 

Appendix: ![](Ultrasonic\_Sensor\_SS2018.002.png)

MATLAB Code: 

clc; clear all; 

%% Create a stepper motor connection to an Adafruit Motor Shield attached to Arduino® hardware. 

%we create an add-on connection to Arduino for Adafruit Motor shieldV2 

a = arduino('COM6','Uno','Libraries',{'Adafruit/MotorShieldV2','JRodrigoTech/HCSR04'}); shield = addon(a,'Adafruit/MotorShieldV2'); 

sensor = addon(a, 'JRodrigoTech/HCSR04', 'D12', 'D13'); % Create an ultrasonic sensor object with trigger pin D12 and echo pin D13. 

%% Create a stepper motor connection to motor number 1 on the shield, with steps per revolution and an RPM. 

sm = stepper(shield,1,200,'RPM',200,'stepType','Microstep'); %% step type is important. It is the Coil activation type. 

table = zeros(450,2); 

coloum1 = table(:,2)'; 

for SPR = 450:-1:1  

`    `move(sm, 5);%% move motor in -ve direction & with desired linear displacement. 

`    `val = readDistance(sensor); % Sensor detects the object and measures the distance at which it is placed. 

`    `valcm = round(val\*100); % convert meter to centimeter. 

`    `table (SPR,1)= (valcm); % fill 1st coloumn with sensor value 

`    `table (SPR,2)= (SPR); % fill the x axis. 

end 

release(sm); % very important to release the motor if not in use or else the over current heats up the motor. 

%% Mapping sensor Data 

figure; 

grid on; 

plot (table(:,2),table(:,1)); % ploting the output 

xlim([0 450]); % Linear movement range of sensor in mm. 

ylim([0 400]); % The range of ultrasonic sensor is from 3cm to 4m. title('Mapping object'); 

xlabel('Linear distance in (cm)'); 

ylabel('Distance in cm'); 

disp(table);

HC-SR04 Cpp file for sensor range setting: 

// Ultrasonic - Library for HR-SC04 Ultrasonic Ranging Module. // GitHub: https://github.com/JRodrigoTech/Ultrasonic-HC-SR04 // #### LICENSE #### 

// This code is licensed under Creative Commons Share alike  // and Attribution by J.Rodrigo ( http://www.jrodrigo.net ). #if ARDUINO >= 100 

`  `#include "Arduino.h" 

#else 

`  `#include "WProgram.h" 

#endif 

#include "Ultrasonic.h" 

Ultrasonic::Ultrasonic(int TP, int EP) 

{ 

`   `pinMode(TP,OUTPUT); 

`   `pinMode(EP,INPUT); 

`   `Trig\_pin=TP; 

`   `Echo\_pin=EP; 

`   `Time\_out=6000;// 3000 Âµs = 50cm // 30000 Âµs = 5 m // Range setting } 

Ultrasonic::Ultrasonic(int TP, int EP, long TO) 

{ 

`   `pinMode(TP,OUTPUT); 

`   `pinMode(EP,INPUT); 

`   `Trig\_pin=TP;    Echo\_pin=EP;    Time\_out=TO; ![](Ultrasonic\_Sensor\_SS2018.002.png)} 

long Ultrasonic::Timing() 

{ 

`  `digitalWrite(Trig\_pin, LOW); 

`  `delayMicroseconds(2); 

`  `digitalWrite(Trig\_pin, HIGH); 

`  `delayMicroseconds(10); 

`  `digitalWrite(Trig\_pin, LOW); 

`  `duration = pulseIn(Echo\_pin,HIGH,Time\_out);   if ( duration == 0 ) { 

duration = Time\_out; } 

`  `return duration; 

} 

long Ultrasonic::Ranging(int sys) 

{ 

`  `Timing(); 

`  `if (sys) { 

distance\_cm = duration /29 / 2 ; return distance\_cm; 

`  `} else { 

distance\_inc = duration / 74 / 2; return distance\_inc; } 

}

HC-SR04 Hex file: 

// Ultrasonic - Library for HR-SC04 Ultrasonic Ranging Module. // GitHub: https://github.com/JRodrigoTech/Ultrasonic-HC-SR04 // #### LICENSE #### 

// This code is licensed under Creative Commons Share alike  // and Attribution by J.Rodrigo ( http://www.jrodrigo.net ). 

#ifndef Ultrasonic\_h #define Ultrasonic\_h 

#if ARDUINO >= 100 

`  `#include "Arduino.h" #else 

`  `#include "WProgram.h" #endif 

#define CM 1 #define INC 0 

class Ultrasonic 

{ 

`  `public: 

`    `Ultrasonic(int TP, int EP); 

Ultrasonic(int TP, int EP, long TO);     long Timing(); 

`    `long Ranging(int sys); 

`  `private: 

`    `int Trig\_pin; 

`    `int Echo\_pin; 

long Time\_out; 

`    `long duration,distance\_cm,distance\_inc; }; 

#endif

Design files: 

All design files are attached as PDF files. 

References ![](Ultrasonic\_Sensor\_SS2018.002.png)

1. *Ultrasonic Sensor Breakout SRF04.* [Online] Available: http://linksprite.com/wiki/index.php5?title=Ultrasonic\_Sensor\_Breakout\_SRF04#Specification. 
1. Wikipedia, *Speed of sound.* [Online] Available: https://en.wikipedia.org/wiki/Speed\_of\_sound. 
1. Adafruit, *adafruit-motor-shield-v2-for-arduino*. 
1. [*https://www.microsonic.de/en/support/ultrasonic-technology/principle.htm* ](https://www.microsonic.de/en/support/ultrasonic-technology/principle.htm)
1. *https://d17kynu4zpq5hy.cloudfront.net/igi/prusa3d/yKdibhMbDytLWxCj.medium*  
1. https://store.arduino.cc/usa/arduino-uno-rev3 
pg. PAGE22
