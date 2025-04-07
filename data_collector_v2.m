clc;
clear all;
rng('shuffle')
% Number of trials to run
N = 10;



% Other simulation parameters
visionDist = 40;   % Example
visionAngle = 45;   % Example
animate = false;    % Set to true if you want to see the animation (will slow down)

sigma1=0.3;
sigma2=0.3;
turn_time_sigma=0.0001;
speedMean=170;
speedSigma=20;
turn_time_mu=0.2;
c1=1;
for foodSpeed=1:4:200
c2=1;
for angle_separation_deg=0:3:180
    angle_separation=pi*angle_separation_deg/180;
    mu1=0.5*angle_separation;
    mu2=-mu1;
% Preallocate structure array to store results
results(N) = struct('captures', [], 'xx', [], 'yy', [], 'turn_angle', []);

% Run the simulation N times and store results
for i = 1:N
    [captures, xx, yy, turn_angle] = simulateTrial(mu1, sigma1, mu2, sigma2, ...
    speedMean, speedSigma, turn_time_mu, turn_time_sigma, ...
        visionDist, visionAngle, foodSpeed, animate);
    % Store outputs in the results struct
    results(i).captures_kd = captures;
    results(i).xx_kd = xx;
    results(i).yy_kd = yy;
    results(i).turn_angle_kd = turn_angle;
    
end
capture_mean(c1,c2)=mean([results.captures_kd]);
capture_std(c1,c2)=std([results.captures_kd]);
sep_angle(c1,c2)=angle_separation_deg;
food_speeds(c1,c2)=foodSpeed;
clear results;
fprintf('Trial %d %d completed.\n', c1,c2);
c2=c2+1;
end
c1=c1+1;
end
% Save results to a MAT file
%save('simulation_results_just_straight.mat', 'results');
% kd_mean
% st_mean
save('simulation_results.mat');

disp('All trials completed and saved.');
