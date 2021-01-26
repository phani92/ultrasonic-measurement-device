# ultrasonic-measurement-device
A measurement setup using ultrasonic sensor and single axis linear system

Introduction:

Our aim for the scientific project was to develop a measurement setup that emits and receives ultrasound. The received data should be processed to a 1D ultrasound image. In addition, the influence of the control of the ultrasonic transmitter on the measured data is to be investigated.

Hence first we started with the basics of ultrasonic sensor its working principle and then we implemented the basic application of ultrasonic sensor for sensing distance with the help of Arduino and serial monitor. We displayed the distance real time in the serial monitor.

As per the problem statement, we need to now develop a measurement setup which can map sensor data. For this, we finalized on a linear guide system with Arduino as a slave and MATLAB as the main controller.

Ultrasonic Sensor:

Ultrasonic sensors emit short, high-frequency sound pulses at regular intervals. These propagate in the air at the velocity of sound. If they strike an object, then they are reflected as echo signals to the sensor, which itself computes the distance to the target based on the time- span between emitting the signal and receiving the echo.

As the distance to an object is determined by measuring the time of flight and not by the  intensity  of  the  sound,  ultrasonic  sensors  are  excellent  at  suppressing  background interference. Virtually all materials which reflect sound can be detected, regardless of their colour. Even transparent materials or thin foils represent no problem for an ultrasonic sensor. Ultrasonic sensors can see through dust-laden air and ink mists.

Even thin deposits on the sensor membrane do not impair its function. Sensors with a blind zone of only 20  mm and an extremely thin beam  spread are making entirely new applications possible today: Fill level measurement in wells of microtiter plates and test tubes, as well as the detection of small bottles in the packaging industry, can be implemented with ease. Even thin wires are reliably detected.

![](\images\Ultrasonic\_Sensor\_SS2018.004.png)

Mechanical Experimental Setup:

We selected a linear axis movement system so that the system can move automatically without any human errors or disturbance and the sensor can be studied more precisely and accurately. The idea of this system is to move it linearly in a single direction over a distance and to study the objects placed in its vicinity. The objects distance from this system and width of the object can be recorded with the help of ultrasonic sensor placed on the movable body.

Another advantage of this design is that we can study other sensors also by replacing the sensor and the respective program in Matlab. This design also gives a liberty to extend the observable area or study area by unbolting the ends and extending the linear guides.

In the initial phase of the project we have designed a single axis linear motion guide with the idea of using lead screw as shown below. The detachable holder with the ultrasonic sensor and Arduino setup can be placed over the moving unit in the above shown setup.

![](images\Ultrasonic\_Sensor\_SS2018.006.png)

To have smooth movement such that the vibrations can be minimised and does not affect the study of sensor, the lead screw system was replaced with a pulley and belt system.

In the below figure, it can be noticed that the centre body is elevated to 5mm with respect to the corner holding stands. This is to allow the centre body to move freely. The idea of this setup is to pull the centre body from one end to another end with the help of a belt and a pulley system. Belt is hinged to the centre body.

![](images\Ultrasonic\_Sensor\_SS2018.008.png)

The setup consists of three main mechanical parts.

1. Vertical Holding stands and linear guides

These stands are on both the corners of the setup and are heavy to give enough strength to the system such that the vibrations are absorbed, and the entire system stays stable on a surface. These stands are further drilled on 4 places so that the linear guides can be inserted into them and be bolted on the outer corners. There is also a flat plate on both the ends to mount the pulley and motor.

The pulley is mounted and free to rotate with the help of a ball bearing. The dimensions of these stands and bearings are provided in the design files that are attached in the appendix.

![](images\Ultrasonic\_Sensor\_SS2018.009.png)

2. Centre Holding body

The centre body is to facilitate free movement for the sensor. This body lies on top of 4 linear guides. It has linear bearings to provide frictionless movement and is connected to the belt on both the ends which provide a pull force allowing for a movement in both the directions. The sensor along with Arduino and bread board for ultrasonic sensor are mounted on top of this setup. The entire metal frame holding the sensor and Arduino setup can be detached from the centre body by unbolting. It is feasible to use the sensor setup alone without the entire system. Just by unbolting two screws the sensor setup can be unmounted from the body.

![](images\Ultrasonic\_Sensor\_SS2018.011.png)

3. Belt and pulley system

The belt is connected to both the ends of the centre of the body and the belt passes through the pulley and motor shaft thereby forming a closed-circuit system.  By pulling one of the ends of the body, it enables the body to travel in that direction. Nema 17 Stepper motor is used as it can provide a torque of 1.6 Kg / cm. The centre body weight around 700 grams and the torque provided by the motor is enough to move the body linearly.

![](images\Ultrasonic\_Sensor\_SS2018.012.png)

The system was designed in SolidWorks and realised in workshop. All the files for individual tiles and the detail list of parts are shared in appendix. 

Solid Works Design:

![](images\Ultrasonic\_Sensor\_SS2018.014.png)

Realised Setup:

Top View

![](images\Ultrasonic\_Sensor\_SS2018.015.png)

Side View

![](images\Ultrasonic\_Sensor\_SS2018.016.png)

Electronic Components:

We have used MATLAB platform to run our setup. Hence the Arduino Uno is used as a passive board. The electrical setup is as shown below.

![](images\Ultrasonic\_Sensor\_SS2018.018.png)

The Electronic components used are: Ultrasonic sensor:

Here, we are using the HC-SR04 ultrasonic sensor. The big advantage of this sensor its available easily and very cheap compared to other ultrasonic sensors.

![](images\Ultrasonic\_Sensor\_SS2018.019.png)

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

![](images\Ultrasonic\_Sensor\_SS2018.021.png)

As discussed earlier, Ultrasonic sensor is used to detect the distance of an object. This is done by time of flight concept. Here we have a proportional relationship between time and distance.

Arduino Uno:

We established a communication between Arduino Uno and MATLAB. We first need to install Hardware support packages for Arduino Uno, HC-SR04 sensor. Once the Hardware packages are installed we can establish MATLAB and Arduino communication. Please see the appendix for the MATLAB code.

Arduino Uno is a microcontroller board based on the ATmega328P. It has 14 digital input/output pins (of which 6 can be used as PWM outputs), 6 analogue inputs, a 16 MHz quartz crystal, a USB connection, a power jack, an ICSP header and a reset button. It contains everything needed to support the microcontroller; simply connect it to a computer with a USB cable or power it with an AC-to-DC adapter or battery to get started.

![](images\Ultrasonic\_Sensor\_SS2018.025.png)

Specifications:

|Microcontroller |ATmega328P |
| - | - |
|Operating voltage |5V |
|Number of Analog input pins |6 |
|Flash memory |32KB |
|Clock speed |16Mhz |
|DC current for 3.3v pin |50 mA |

Adafruit Motor shield v2.3:

The Motor shield is used as a driver for stepper motor. The stepper motor needs 0.33A of current to run at 12V DC. The Motor shield provides a 1.2A peak current per phase. We can connect 2 stepper motors with this shield at the same time. We have connected our Stepper motor at M1 and M2 as it’s a bipolar stepper motor.

![](images\Ultrasonic\_Sensor\_SS2018.027.png)

Specifications :

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

![](images\Ultrasonic\_Sensor\_SS2018.029.png)

Power Supply:

We need power to run the entire setup. 5v DC power to run the Arduino and 12V to power the shield which provides the required voltage and current for the motors.

The Arduino is powered with the help of a USB cable and as we track live data, it can be controlled with Matlab. The Arduino acts as a slave to Matlab. The Shield is provided with external power source with the help of an adapter. These both power sources are enough to run the entire setup and collect data from the sensor.

Calculations:

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

Steps / mm = 200 ∗1 = 5

(2) ∗(20)

For loops in Matlab = (Steps/mm) \* (distance) = 5 \* 450 = 2250

- Maximum speed of motor:

Max Speed = ( ∗ 60)/( ∗ 2 ∗ ∗ )

- 12 \* 60 / (1.2 \* 2 \* 0.33 \* 200)
- 4.5 mm / s

Note: In Matlab code, number of loops is 450 and the linear displacement given is 5 which results in 2250 as per our calculation above.

Range of sensor:

![](images\Ultrasonic\_Sensor\_SS2018.031.png)

Measurements:

1. Experiment with a metal stand (30 cm distance):

![](images\Ultrasonic\_Sensor\_SS2018.033.png)

Result:

![](images\Ultrasonic\_Sensor\_SS2018.034.png)

The above plot was mapped in Matlab and it can be seen the distance to be approximately 30 cm.

2. Experiment with a wooden stand (70 cm distance):

![](images\Ultrasonic\_Sensor\_SS2018.036.png)

Results:

![](images\Ultrasonic\_Sensor\_SS2018.037.png)

It can be seen in the above figure that wood is placed exactly at 70 cm and the output graph is also nearby to 70 cm.

Observations:

- The ultrasonic sensor detects all the objects in this range at a beam angle of 15 degrees. Hence the object width calculation is a tricky job with this ultrasonic sensor.
- As per the time of flight concept the measured distance is directly proportional to the time. Time is inversely proportional to frequency. Hence by varying the frequency of the piezo crystal the range of ultrasonic sensor will be varied.
- Ultrasonic sensor can give false echoes of object places at different distance. This can be solved by using the ultrasonic sensor with low frequency by which the false echoes will diminished before reaching the receiver.
- Objects which are place at an angle will cause the deflection of echoes. Hence the objects should always be present perpendicular.
- There is also an effect of temperature. As temperature increases the detecting range decreases. Hence in some of the commercial products they have temperature compensation circuit available.

Future scope:

- The communication can be made wireless.
- With the existing setup we can plug other sensor for distance measurement and mapping environment. Sensors with high accuracy and less beam angle.
