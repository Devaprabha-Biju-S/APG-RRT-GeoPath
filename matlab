function APG_RRT()
    % APG-RRT Algorithm Implementation in MATLAB

    % Parameters
    maxIterations = 500; % Maximum number of iterations
    stepSize = 5; % Step size for tree expansion
    mapSize = [100, 100]; % Map dimensions
    obstacleExpansion = 5; % Safe distance for obstacle expansion
    vehicleRadius = 2; % Vehicle size for collision safety

    % Goal and start points
    startPoint = [10, 10];
    goalPoint = [90, 90];

    % Initialize Environment
    [map, obstacles] = createEnvironment(mapSize, obstacleExpansion);

    % APG-RRT Initialization
    tree = struct('nodes', startPoint, 'parent', -1); % Tree structure
    guidePaths = initializeGuidePaths(mapSize); % Predefined paths
    pathWeights = ones(1, length(guidePaths)) / length(guidePaths);

    % Main Algorithm
    for iter = 1:maxIterations
        % Sampling using guidance paths and randomness
        targetPoint = samplePoint(map, guidePaths, pathWeights);
        nearestIdx = findNearestNode(tree, targetPoint);
        newPoint = steer(tree.nodes(nearestIdx, :), targetPoint, stepSize);

        % Collision detection
        if ~isCollision(newPoint, obstacles, vehicleRadius)
            tree.nodes = [tree.nodes; newPoint];
            tree.parent = [tree.parent; nearestIdx];

            % Update path weights based on collisions
            pathWeights = updatePathWeights(pathWeights, guidePaths, newPoint, obstacles);

            % Check if goal is reached
            if norm(newPoint - goalPoint) < stepSize
                disp('Goal Reached!');
                break;
            end
        end
    end

    % Extract and Smooth Path
    path = extractPath(tree, goalPoint);
    smoothPath = smoothPathCubicSpline(path, obstacles, vehicleRadius);

    % Visualization
    visualizeEnvironment(map, obstacles, tree, path, smoothPath);
    visualizeTreeGrowth(tree);
    visualizeSamplingDistribution(guidePaths, pathWeights, mapSize);
end

function [map, obstacles] = createEnvironment(mapSize, obstacleExpansion)
    % Create a map with obstacles and expand boundaries for safety
    map = zeros(mapSize);
    obstacles = [
        30, 30, 10; % Obstacle 1: [x, y, radius]
        60, 60, 15; % Obstacle 2
        40, 80, 8   % Obstacle 3
    ];

    % Expand obstacles
    for i = 1:size(obstacles, 1)
        obstacles(i, 3) = obstacles(i, 3) + obstacleExpansion;
    end
end

function guidePaths = initializeGuidePaths(mapSize)
    % Initialize predefined guidance paths
    guidePaths = {
        [0, 0; mapSize(1), 0], ... % Long straight path
        [0, 0; mapSize(1), mapSize(2)], ... % Diagonal path
        [0, 0; mapSize(1)/2, mapSize(2); mapSize(1), mapSize(2)/2] % U-turn
    };
end

function targetPoint = samplePoint(map, guidePaths, pathWeights)
    % Sample a point using guidance paths and random exploration
    if rand < 0.8 % 80% probability to follow guide paths
        guideIdx = randsample(1:length(guidePaths), 1, true, pathWeights);
        path = guidePaths{guideIdx};
        targetPoint = path(randi(size(path, 1)), :);
    else
        targetPoint = [rand * size(map, 1), rand * size(map, 2)];
    end
end

function nearestIdx = findNearestNode(tree, targetPoint)
    % Find the nearest node in the tree to the target point
    distances = vecnorm(tree.nodes - targetPoint, 2, 2);
    [~, nearestIdx] = min(distances);
end

function newPoint = steer(nearestPoint, targetPoint, stepSize)
    % Steer towards the target point with a fixed step size
    direction = (targetPoint - nearestPoint) / norm(targetPoint - nearestPoint);
    newPoint = nearestPoint + direction * stepSize;
end

function collision = isCollision(point, obstacles, radius)
    % Check if the point collides with any obstacles
    collision = false;
    for i = 1:size(obstacles, 1)
        if norm(point - obstacles(i, 1:2)) < (obstacles(i, 3) + radius)
            collision = true;
            return;
        end
    end
end

function pathWeights = updatePathWeights(pathWeights, guidePaths, point, obstacles)
    % Update weights for guidance paths based on collision information
    for i = 1:length(guidePaths)
        if isCollision(point, obstacles, 0)
            pathWeights(i) = pathWeights(i) * 0.9; % Reduce weight on collision
        else
            pathWeights(i) = pathWeights(i) * 1.1; % Increase weight if successful
        end
    end
    pathWeights = pathWeights / sum(pathWeights); % Normalize weights
end

function path = extractPath(tree, goalPoint)
    % Extract path from tree to the goal
    path = goalPoint;
    currentIdx = size(tree.nodes, 1);
    while currentIdx > 0
        path = [tree.nodes(currentIdx, :); path];
        currentIdx = tree.parent(currentIdx);
    end
end

function smoothPath = smoothPathCubicSpline(path, obstacles, vehicleRadius)
    % Smooth path using cubic spline interpolation with collision checking
    t = 1:size(path, 1); % Original waypoints indices
    tt = linspace(1, size(path, 1), 10 * size(path, 1)); % Interpolated points

    % Perform cubic spline smoothing
    rawSmoothPath = [spline(t, path(:, 1), tt)', spline(t, path(:, 2), tt)'];
    
    % Collision checking for smoothed path
    smoothPath = rawSmoothPath(1, :); % Start with the first point
    for i = 2:size(rawSmoothPath, 1)
        if ~isCollision(rawSmoothPath(i, :), obstacles, vehicleRadius)
            smoothPath = [smoothPath; rawSmoothPath(i, :)];
        else
            % If a point is in collision, skip it
            disp(['Collision detected at: ', mat2str(rawSmoothPath(i, :))]);
        end
    end
end

function visualizeEnvironment(map, obstacles, tree, path, smoothPath)
    % Visualize the environment, tree, and paths
    figure;
    hold on;
    axis([0, size(map, 1), 0, size(map, 2)]);
    
    % Draw obstacles
    for i = 1:size(obstacles, 1)
        viscircles(obstacles(i, 1:2), obstacles(i, 3), 'Color', 'r');
    end

    % Draw tree
    for i = 2:size(tree.nodes, 1)
        line([tree.nodes(tree.parent(i), 1), tree.nodes(i, 1)], ...
             [tree.nodes(tree.parent(i), 2), tree.nodes(i, 2)], 'Color', 'b');
    end

    % Draw paths
    plot(path(:, 1), path(:, 2), 'g', 'LineWidth', 2);
    plot(smoothPath(:, 1), smoothPath(:, 2), 'm', 'LineWidth', 2);
    
    hold off;
    title('APG-RRT Path Planning');
    legend('Obstacles', 'Tree', 'Initial Path', 'Smoothed Path');
end

function visualizeTreeGrowth(tree)
    % Visualize the growth of the tree
    figure;
    hold on;
    title('Tree Growth');
    xlabel('X');
    ylabel('Y');
    for i = 2:size(tree.nodes, 1)
        line([tree.nodes(tree.parent(i), 1), tree.nodes(i, 1)], ...
             [tree.nodes(tree.parent(i), 2), tree.nodes(i, 2)], 'Color', 'b');
    end
    hold off;
end

function visualizeSamplingDistribution(guidePaths, pathWeights, mapSize)
    % Visualize the sampling distribution
    figure;
    hold on;
    axis([0, mapSize(1), 0, mapSize(2)]);
    title('Sampling Distribution');
    for i = 1:length(guidePaths)
        path = guidePaths{i};
        plot(path(:, 1), path(:, 2), 'LineWidth', pathWeights(i) * 5); % Weight visualized as thickness
    end
    hold off;
end
