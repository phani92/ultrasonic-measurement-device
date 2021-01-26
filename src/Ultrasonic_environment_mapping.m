


%% Mapping sensor Data
figure;
grid on;
plot (table(:,2),table(:,1)); % ploting the output
xlim([0 450]); % Linear movement range of sensor in mm.
ylim([0 400]); % The range of ultrasonic sensor is from 3cm to 4m.
title('Mapping object');
xlabel('Linear distance in (cm)');
ylabel('Distance in cm');
disp(table);