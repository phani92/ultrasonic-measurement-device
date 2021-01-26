clc;
clear all;

%% Create a stepper motor connection to an Adafruit Motor Shield attached to Arduino® hardware.
%we create an add-on connection to Arduino for Adafruit Motor shieldV2
a = arduino('COM8','Uno','Libraries',{'Adafruit/MotorShieldV2','JRodrigoTech/HCSR04'});
shield = addon(a,'Adafruit/MotorShieldV2');
sensor = addon(a, 'JRodrigoTech/HCSR04', 'D12', 'D13'); % Create an ultrasonic sensor object with trigger pin D12 and echo pin D13.

%% The I2CAddress of a shield is set to 0x60 by default if not specified. Search for available I2C addresses on bus 0 to specify a different address.
addrs = scanI2CBus(a,0);

%% Create a stepper motor connection to motor number 1 on the shield, with steps per revolution and an RPM.
sm = stepper(shield,1,10,'stepType','Microstep'); %% step type is important. It is the Coil activation type.
sm.RPM = 1000; %% To the desired speed of motor
table = zeros(1700,4);
for SPR = 1:1:1700
    move(sm, 1);%% move motor in +ve direction(right direction) & with desired linear displacement.
    val = readDistance(sensor); % Sensor detects the object and measures the distance at which it is placed.
    valcm = round(val*100); % convert meter to centimeter.
    table (SPR,1)= (valcm); % fill 1st coloumn with sensor value
    table (SPR,3)= (SPR); % fill 
end
%revolutions = SPR

%for SPR = 170:-1:1
    %move(sm, -10);%% move motor in +ve direction(right direction) & with desired linear displacement
    %val = readDistance(sensor);
    %valcm = round(val*100);
    %table (SPR,2)= (valcm);
%end
release(sm);
%table(:,4)= (table(:,1) + table (:,2))/2;
figure;
grid on;
plot (table(:,3),table(:,1));
title('Mapping object');
xlabel('X');
ylabel('Distance in cm');
disp(table)


