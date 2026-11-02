% Simulated Sensor Data for ACB Fault Detection (Updated for 3 Sets of Springs)

% Time settings
fs = 100; % Sampling frequency (Hz)
t = 0:1/fs:10; % 10 seconds of data

% Normal Condition (All 3 sets of springs working)
sound_normal = 50 + 1.5*randn(size(t)); % Simulated sound level (dB)
accel_normal = 0.1 + 0.01*randn(size(t)); % Simulated vibration (g)

% 1 Set Faulty (2 working, 1 faulty)
sound_1faulty = 52 + 1.5*randn(size(t));
accel_1faulty = 0.15 + 0.012*randn(size(t));

% 2 Sets Faulty (1 working, 2 faulty)
sound_2faulty = 55 + 1.5*randn(size(t));
accel_2faulty = 0.2 + 0.015*randn(size(t));

% 3 Sets Faulty (All faulty)
sound_3faulty = 58 + 1.5*randn(size(t));
accel_3faulty = 0.25 + 0.02*randn(size(t));

% Plot results
figure;
subplot(2,1,1);
plot(t, sound_normal, 'g', t, sound_1faulty, 'r', t, sound_2faulty, 'm', t, sound_3faulty, 'b');
title('Simulated Sound Sensor Data');
legend('All Springs Working', '1 Set Faulty', '2 Sets Faulty', '3 Sets Faulty');

subplot(2,1,2);
plot(t, accel_normal, 'g', t, accel_1faulty, 'r', t, accel_2faulty, 'm', t, accel_3faulty, 'b');
title('Simulated Accelerometer Data');
legend('All Springs Working', '1 Set Faulty', '2 Sets Faulty', '3 Sets Faulty');
xlabel('Time (s)');
ylabel('Vibration (g)');

% Save data for machine learning
data_table = table(t', sound_normal', sound_1faulty', sound_2faulty', sound_3faulty', ...
                    accel_normal', accel_1faulty', accel_2faulty', accel_3faulty', ...
                    'VariableNames', {'Time', 'Sound_Normal', 'Sound_1Faulty', 'Sound_2Faulty', 'Sound_3Faulty', ...
                                      'Accel_Normal', 'Accel_1Faulty', 'Accel_2Faulty', 'Accel_3Faulty'});
writetable(data_table, 'simulated_spring_fault_data.csv');
