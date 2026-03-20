clear; close; clc;

%% Bar Chart
figure(1)
execution_time = [56 70];
labels = ["hardware" "software"];
bar(execution_time, 'FaceColor', [14 131 92]./255);
xticklabels(labels);
ylabel('Execution Time (ms)');
ylim([0 execution_time(2) + 10]);
box off;
grid on;

%% Position Accuracy Scatter
figure(2)
% set(gcf, 'Position', [100 100 900 500]);
rng(1);
N = 200;
x = linspace(0,10,N)';
sigma_perp = 0.3;
u_perp = [1; -1]/sqrt(2);
d = sigma_perp * randn(N,1);

X = x;
Y = x + d * u_perp(2)/u_perp(1);

scatter(X,Y,36,[14 131 92]./255,'filled')
hold on
plot([0 10],[0 10],'k--','LineWidth',1)
xlim([0 10])
xlabel('Real (m)')
ylim([0 10])
ylabel('Predicted (m)')
grid on
hold off


%% Realtime Drone Position Relative to Beacons - Smooth Animated Random Walk
figure(3)
set(gcf, 'Color', '#FFFFFF');
beacons = [
    0.5 9.5;
    1.5 9.5;
    0.5 8.0
    ];

robot = [7 2];

% Random walk parameters
nSteps = 50;
dt = 0.1;                % time step
maxSpeed = 2.0;          % clamp max velocity magnitude
angularNoise = 0.3;      % how much the heading can randomly change per step
speedNoise = 0.2;        % how much the speed can randomly change per step

positions = zeros(nSteps, 2);
positions(1,:) = robot;
vel = [-.5, .5];           % starting velocity vector

% Generate smooth random walk
for k = 2:nSteps
    % Perturb velocity: random angular deviation + speed perturbation
    angle = atan2(vel(2), vel(1)) + angularNoise * randn();
    speed = norm(vel) + speedNoise * randn();
    speed = max(min(speed, maxSpeed), 0.2);  % keep speed in reasonable range
    vel = speed * [cos(angle), sin(angle)];

    newPos = positions(k-1,:) + vel * dt;

    % Bounce off walls
    for dim = 1:2
        if newPos(dim) < 0
            newPos(dim) = -newPos(dim);
            vel(dim) = -vel(dim);
        elseif newPos(dim) > 10
            newPos(dim) = 20 - newPos(dim);
            vel(dim) = -vel(dim);
        end
    end

    positions(k,:) = newPos;
end

% Set up video writer
v = VideoWriter('robot_random_walk', 'MPEG-4');
v.FrameRate = 10;
open(v);

% Animation loop
for k = 1:nSteps
    scatter(beacons(:,1), beacons(:,2), 300, ...
        [241 124 163]./255, 'o', 'LineWidth', 1.5);
    hold on
    if k > 1
        plot(positions(1:k,1), positions(1:k,2), '-', ...
            'Color', [0.5 0.5 0.5], 'LineWidth', 0.8);
    end
    scatter(positions(k,1), positions(k,2), 300, ...
        [14 131 92]./255, '^', 'filled', 'LineWidth', 1.5);
    hold off

    xlim([0 10]);
    xlabel("x (m)");
    ylim([0 10]);
    ylabel("y (m)");
    legend(["Beacons", "Trail", "Robot"], 'Location', 'northeast');

    drawnow;
    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);
fprintf('Video saved as robot_random_walk.mp4\n');