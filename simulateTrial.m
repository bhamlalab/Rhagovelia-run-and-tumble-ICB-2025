function [captures, xx, yy, turn_angle] = simulateTrial(mu1, sigma1, mu2, sigma2, ...
    speedMean, speedSigma, turn_time_mu, turn_time_sigma, ...
    visionDist, visionAngle,foodSpeed, animate)
% SIMULATETRIALCOMMON runs the predator-prey simulation.
%
% INPUTS:
%   mu1, sigma1, mu2, sigma2   - parameters for the bimodal angle distribution
%   speedMean, speedSigma       - parameters for the Gaussian speed distribution
%   turn_time_mu, turn_time_sigma - mean and variability for time between turns
%   visionDist                - maximum detection distance (mm)
%   visionAngle               - maximum detection angle (degrees)
%   animate                   - Boolean flag: if true, run in animation mode.
%
% OUTPUT:
%   captures                - number of food detections ("captures") during simulation

    % Simulation settings
    T=1000;
    dt = 0.01;        % Time step in seconds
    Niter = round(T/dt);       % Total simulation time steps
    xmin = 0; xmax = 254;
    ymin = 0; ymax = 254;
    n_food = 4;
    
    % Generate predator movement parameters
    predatorAngles = generateAngles(Niter, mu1, sigma1, mu2, sigma2);
    predatorSpeeds = generateSpeeds(Niter, speedMean, speedSigma);
    turn_times = generateturntimes(Niter, turn_time_mu, turn_time_sigma);
    turn_angle=predatorAngles;
    
    % Initialize predator state
    x_pred = xmin + (xmax - xmin) * rand();
    y_pred = ymin + (ymax - ymin) * rand();
    heading = 0;  % initial heading in radians
    
    % Initialize food positions
    foodPositions = zeros(n_food, 2);
    for j = 1:n_food
        foodPositions(j,:) = [xmin + (xmax - xmin)*rand(), ...
                              ymin + (ymax - ymin)*rand()];
    end
    
    foodDirections = 0*2*pi * rand(n_food, 1);
    
    captures = 0;
    c1 = 1;
    currentAngle = mod(heading + predatorAngles(c1), 2*pi);
    currentSpeed = predatorSpeeds(c1);
    xx=zeros(1,T);
    yy=xx;
    xx(1)=x_pred;
    yy(1)=y_pred;
    % If animating, set up the figure and graphics objects
    if animate
        figure;
        hold on;
        axis equal;
        xlim([xmin, xmax]);
        ylim([ymin, ymax]);
        grid on;
        title('Predator-Prey Simulation Animation');
        predatorPlot = plot(x_pred, y_pred, 'ro', 'MarkerSize', 10, 'MarkerFaceColor','r');
        trailPlot = plot(xx,yy,'LineWidth',3,'Color','k');
        foodPlot = plot(foodPositions(:,1), foodPositions(:,2), 'b+', 'MarkerSize', 10, 'LineWidth',2);
        visionConePatch = patch(0, 0, 'g', 'FaceAlpha', 0.2, 'EdgeColor','none');
    end

    % Main simulation loop
    for ii = 1:Niter
        % Update movement every turn interval (in steps)
        if mod(ii, max(1, round(turn_times(c1)/dt))) == 0
            currentAngle = mod(heading + predatorAngles(c1), 2*pi);
            currentSpeed = predatorSpeeds(c1);
            c1 = c1 + 1;
        end
        
        % Compute velocity components
        vx = currentSpeed * cos(currentAngle);
        vy = currentSpeed * sin(currentAngle);
        
        % Update predator position
        x_pred = x_pred + vx * dt;
        y_pred = y_pred + vy * dt;
        
        % Wrap-around boundary conditions
        if x_pred > xmax, x_pred = xmin; 
        elseif x_pred < xmin, x_pred = xmax; end
        if y_pred > ymax, y_pred = ymin; 
        elseif y_pred < ymin, y_pred = ymax; end
        
        % Update heading based on new velocity
        heading = mod(atan2(vy, vx), 2*pi);
        
        % Update food positions (with wrap-around)
        for j = 1:n_food
            foodPositions(j,1) = mod(foodPositions(j,1) + foodSpeed*cos(foodDirections(j))*dt, xmax);
            foodPositions(j,2) = mod(foodPositions(j,2) + foodSpeed*sin(foodDirections(j))*dt, ymax);
        end
        
        % Food detection check
        for j = 1:n_food
            dx = foodPositions(j,1) - x_pred;
            dy = foodPositions(j,2) - y_pred;
            dist = sqrt(dx^2 + dy^2);
            if dist <= visionDist
                % Compute angle from predator to food (degrees)
                foodAngle = rad2deg(atan2(dy, dx));
                predAngleDeg = rad2deg(heading);
                % Calculate the smallest angular difference (accounting for wrap-around)
                angleDiff = abs(angdiff(deg2rad(foodAngle), deg2rad(predAngleDeg)))*180/pi;
                if angleDiff <= visionAngle
                    captures = captures + 1;
                    % Optionally, reset food position after capture
                    foodPositions(j,:) = [xmin + (xmax - xmin)*rand(), ...
                                          ymin + (ymax - ymin)*rand()];
                end
            end
        end
        xx(ii)=x_pred;
        yy(ii)=y_pred;
        % Update graphics if in animation mode
        if animate
            set(predatorPlot, 'XData', x_pred, 'YData', y_pred);
            if(ii>50)
            set(trailPlot, 'XData', xx(ii-50:ii), 'YData', yy(ii-50:ii));
            end
            set(foodPlot, 'XData', foodPositions(:,1), 'YData', foodPositions(:,2));
            visionCone = createVisionCone(x_pred, y_pred, heading, visionDist, visionAngle);
            set(visionConePatch, 'XData', visionCone(:,1), 'YData', visionCone(:,2));
            drawnow;
            pause(0.01);
        end
    end
end



% --- Helper Functions ---

function angles = generateAngles(n, mu1, sigma1, mu2, sigma2)
    % Generate n angles as a 50/50 mixture of two Gaussian distributions.
    angles = zeros(n,1);
    for i = 1:n
        if rand < 0.5
            sample = normrnd(mu1, sigma1);
        else
            sample = normrnd(mu2, sigma2);
        end
        angles(i) = mod(sample, 2*pi);
    end
end

function speeds = generateSpeeds(n, speedMean, speedSigma)
    % Generate n speeds from a Gaussian distribution, ensuring non-negative values.
    speeds = normrnd(speedMean, speedSigma, n, 1);
    speeds(speeds < 0) = speedMean;
end

function turn_times = generateturntimes(n, turn_time_mu, turn_time_sigma)
    % Generate n turn times from a Gaussian distribution, ensuring non-negative values.
    turn_times = normrnd(turn_time_mu, turn_time_sigma, n, 1);
    turn_times(turn_times < 0) = turn_time_mu;
end

function vertices = createVisionCone(x, y, heading, visionDist, visionAngle)
    % Create vertices for a triangular vision cone based on the predator's position.
    halfAngle = deg2rad(visionAngle);
    leftBoundary = heading - halfAngle;
    rightBoundary = heading + halfAngle;
    leftPoint = [x + visionDist*cos(leftBoundary), y + visionDist*sin(leftBoundary)];
    rightPoint = [x + visionDist*cos(rightBoundary), y + visionDist*sin(rightBoundary)];
    vertices = [x, y; leftPoint; rightPoint];
end

function d = angdiff(a, b)
    % Compute the angular difference between two angles (in radians) in the range [-pi, pi].
    d = mod((a - b) + pi, 2*pi) - pi;
end
