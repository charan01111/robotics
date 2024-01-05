function [vx, vy] = avoidObstacle(x, y, obstacles)
    % x, y: current position
    % obstacles: matrix with obstacle positions (each row represents an obstacle [x, y])

    % Parameters
    k_att = 0.1; % Attraction constant
    k_rep = 1;   % Repulsion constant
    d_rep = 5;   % Repulsion distance

    % Calculate attraction force towards a goal (you can set your goal coordinates)
    goal = [10, 10];
    att_force = k_att * (goal - [x, y]);

    % Calculate repulsion force from obstacles
    rep_force = zeros(1, 2);
    for i = 1:size(obstacles, 1)
        obstacle = obstacles(i, :);
        dist = norm([x, y] - obstacle);
        if dist < d_rep
            rep_force = rep_force + k_rep * ((1/dist) - (1/d_rep)) * (([x, y] - obstacle) / dist^3);
        end
    end

    % Total force
    total_force = att_force + rep_force;

    % Normalize and set velocity components
    magnitude = norm(total_force);
    if magnitude > 0
        vx = total_force(1) / magnitude;
        vy = total_force(2) / magnitude;
    else
        vx = 0;
        vy = 0;
    end
end
