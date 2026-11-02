% Set Python Environment
pyenv('Version', 'C:\Users\Grishma\AppData\Local\Programs\Python\Python312\python.exe');
script_path = "C:\Users\Grishma\Documents\MATLAB\obstaclepythonpyfile.py";

% Execute Python script
try
    command = "python """ + script_path + """";  % Run script to train model
    [status, cmdout] = system(command);
    if status ~= 0
        disp("Python script error:"); disp(cmdout); return;
    end
    py.importlib.import_module('obstaclepythonpyfile');  % Import for prediction
catch ME
    disp("Error executing Python script:"); disp(ME.message); return;
end

% Initialize figure
figure; hold on; grid on;
xlim([0 50]); ylim([0 3]);
xlabel('X Position'); ylabel('Track');
title('Obstacle Avoidance Simulation');

% Car and obstacle setup
X_car = 0; Y_car = 2; speed = 0.25;
num_obstacles = 13;
max_distance = 50;
obstacles = [randi([5, 50], num_obstacles, 1), randi([1, 3], num_obstacles, 1)];

while X_car < max_distance
    % Detect nearest obstacle in current lane
    lane_obstacles = obstacles(obstacles(:,2) == Y_car, :);
    
    if isempty(lane_obstacles)
        min_dist = Inf; X_obs = Inf; Y_obs = Y_car;
    else
        [min_dist, idx] = min(abs(lane_obstacles(:,1) - X_car));
        X_obs = lane_obstacles(idx, 1);
        Y_obs = lane_obstacles(idx, 2);
    end
    
    % Predict using Python
    result = py.obstaclepythonpyfile.predict_lane_change([X_car, X_obs, min_dist, 2, Y_car]);
    prediction = double(result{1}); lane_change = double(result{2});
    
    % Lane change with diagonal motion
    if prediction == 1 && min_dist <= 2
        steps = 10;
        lane_target = min(max(Y_car + lane_change, 1), 3);
        dx = speed * 0.3;
        dy = (lane_target - Y_car) / steps;

        for i = 1:steps
            X_car = X_car + dx;
            Y_car = Y_car + dy;

            % Move obstacles left as car moves forward
            obstacles(:,1) = obstacles(:,1) - dx;
            for j = 1:num_obstacles
                if obstacles(j,1) < X_car + 1
                    obstacles(j,1) = X_car + randi([20, 30]);
                    obstacles(j,2) = randi([1,3]);
                end
            end

            % === DRAW SCENE ===
            clf;

            % Grass on left and right
            fill([X_car-5, X_car+15, X_car+15, X_car-5], [3.5, 3.5, 4.5, 4.5], [0.3 0.7 0.3]); % Top grass
            fill([X_car-5, X_car+15, X_car+15, X_car-5], [-1, 0, 0, -1], [0.3 0.7 0.3]);         % Bottom grass

            % Road
            fill([X_car-5, X_car+15, X_car+15, X_car-5], [0, 0, 3.5, 3.5], [0.6 0.6 0.6]); % Grey road

            hold on; grid on;

            % Lane lines
            for y_line = [0.5, 1.5, 2.5, 3.5]
                plot([X_car-5, X_car+15], [y_line, y_line], 'w--', 'LineWidth', 1.5);
            end

            xlim([X_car-5, X_car+15]); ylim([-0.5, 4.5]);

            % Draw obstacles
            for j = 1:size(obstacles,1)
                text(obstacles(j,1), obstacles(j,2), "🚧", 'FontSize', 20, 'HorizontalAlignment', 'center');
            end
            % Draw car
            text(X_car, Y_car, "🚗", 'FontSize', 20, 'HorizontalAlignment', 'center');
            title("Changing Lane", 'FontSize', 14);

            pause(0.05);
        end

        Y_car = round(lane_target);
    else
        X_car = X_car + speed;

        % Move obstacles left as car moves forward
        obstacles(:,1) = obstacles(:,1) - speed;
        for j = 1:num_obstacles
            if obstacles(j,1) < X_car + 1
                obstacles(j,1) = X_car + randi([20, 30]);
                obstacles(j,2) = randi([1,3]);
            end
        end
    end

    % === DRAW SCENE ===
    clf;

    % Grass on both sides
    fill([X_car-5, X_car+15, X_car+15, X_car-5], [3.5, 3.5, 4.5, 4.5], [0.3 0.7 0.3]); % Top grass
    fill([X_car-5, X_car+15, X_car+15, X_car-5], [-1, 0, 0, -1], [0.3 0.7 0.3]);         % Bottom grass

    % Road
    fill([X_car-5, X_car+15, X_car+15, X_car-5], [0, 0, 3.5, 3.5], [0.6 0.6 0.6]);

    hold on; grid on;

    % Lane lines
    for y_line = [0.5, 1.5, 2.5, 3.5]
        plot([X_car-5, X_car+15], [y_line, y_line], 'w--', 'LineWidth', 1.5);
    end

    xlim([X_car-5, X_car+15]); ylim([-0.5, 4.5]);

    % Draw obstacles
    for j = 1:size(obstacles,1)
        text(obstacles(j,1), obstacles(j,2), "🚧", 'FontSize', 20, 'HorizontalAlignment', 'center');
    end

    % Draw car
    text(X_car, Y_car, "🚗", 'FontSize', 20, 'HorizontalAlignment', 'center');

    % Status message
    if prediction == 1 && min_dist <= 2
        title("Lane Change Complete", 'FontSize', 14); 
    else
        title("Safe", 'FontSize', 14); 
    end

    pause(0.1);
end
