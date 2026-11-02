obstacle_positions = [5, 7, 8, 9, 9.5, 9.7, 9.9, 10, 10.2, 10.5, 11, 12, 15, 18];
car_velocities = [1, 2, 3, 4]; 

data = []; % Initialize an empty matrix to store results

for i = 1:length(obstacle_positions)
    for j = 1:length(car_velocities)
        % Set obstacle position and car velocity
        X_obs_value = obstacle_positions(i);
        V_car_value = car_velocities(j);
        
        % Assign values in Simulink
        assignin('base', 'X_obs_value', X_obs_value);
        assignin('base', 'V_car_value', V_car_value);
        
        % Run the simulation
        simOut = sim('Obstacle_avoidance_bot');  
        % Extract results
        X_car = simOut.X_car.signals.values;
        X_obs = simOut.X_obs.signals.values;
        D = simOut.D.signals.values;
        V_car = simOut.V_car.signals.values;
        Collision = simOut.Collision.signals.values;

        num_samples = length(X_car);

        % Repeat single values to match time-series length
        X_obs = repmat(X_obs, num_samples, 1); 
        V_car = repmat(V_car, num_samples, 1);

        % Append new data
        data = [data; X_car, X_obs, D, V_car, Collision];
    end
end

% Convert data to table and save as CSV
data_table = array2table(data, 'VariableNames', {'X_car', 'X_obs', 'Distance', 'Velocity', 'Collision'});
writetable(data_table, 'obstacle_data.csv');

disp(" Data collection complete. CSV file saved!");
