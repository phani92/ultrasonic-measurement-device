clc;
clear all;

%% Create a stepper motor connection to an Adafruit Motor Shield attached to Arduino® hardware.
%we create an add-on connection to Arduino for Adafruit Motor shieldV2
a = arduino('COM6','Uno','Libraries',{'Adafruit/MotorShieldV2'})
shield = addon(a,'Adafruit/MotorShieldV2')
%sensor = addon(a, 'JRodrigoTech/HCSR04', 'D12', 'D13');

%% The I2CAddress of a shield is set to 0x60 by default if not specified. Search for available I2C addresses on bus 0 to specify a different address.
addrs = scanI2CBus(a,0);

%% Create a stepper motor connection to motor number 1 on the shield, with steps per revolution and an RPM.
sm = stepper(shield,1,500,'stepType','Double'); %% step type is important. It is the Coil activation type.
sm.RPM = 100; %% To the desired speed of motor
%ssPrintf("RPM set to - 100");
move(sm, 1000);%% move motor in +ve direction(right direction) & with desired linear displacement

%ssPrintf("Clockwise Rotation");
pause(2);
move(sm, -1000); %% move motor in -ve direction(left direction) & with desired linear displacement
%ssPrintf("Anticlockwise Rotation");
release(sm); %% stop stepper motor