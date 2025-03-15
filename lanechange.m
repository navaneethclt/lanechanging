% Clear workspace, close all figures, and clear command window
clc; clear; close all;

%% Given parameters
W = 3.5;         % Lateral displacement (m)
u = 30 * 0.447;  % Convert mph to m/s (initial velocity)
v = 1 * 0.447;   % Velocity component in y-direction

y0 = -1.0;       % Initial lateral position
ax = 1;          % Acceleration in x-direction
ay = 0;          % Acceleration in y-direction

% Cost function weights
lambda1 = 1;     % Weight for acceleration cost
lambda2 = 1;     % Weight for time cost
lambda3 = 0.1;   % Weight for jerk cost

e1 = -5 * pi / 180; % Initial yaw angle error (radians)

% Adjust initial velocity components based on yaw error
u_x = u * cos(e1);
u_y = u * sin(e1);

%% Optimization Setup
T0 = 2;  % Initial guess for T
options = optimset('Display', 'iter', 'TolFun', 1e-6, 'TolX', 1e-6);

% Run optimization
[T_opt, J_opt] = fminunc(@(T) cost_function(T, u_x, u_y, u, v, y0, ay, ax, W, lambda1, lambda2, lambda3), T0, options);

fprintf('Optimized Time: %.4f s\n', T_opt);
fprintf('Optimized Cost: %.4f\n', J_opt);

%% Generate and Plot Trajectory
[t_vals, x_vals, y_vals, x_dot, y_dot, x_ddot, y_ddot] = generate_trajectory(T_opt, u_x, u_y, u, v, y0, ay, ax, W);
figure;
plot(x_vals, y_vals, 'b-', 'LineWidth', 2);
hold on;
scatter(x_vals(1), y_vals(1), 'ro', 'filled'); % Start point
scatter(x_vals(end), y_vals(end), 'go', 'filled'); % End point

% Plot lane boundaries
plot([min(x_vals), max(x_vals)], [-W/2, -W/2], 'k', 'LineWidth', 1.5);
plot([min(x_vals), max(x_vals)], [W/2, W/2], 'k', 'LineWidth', 1.5);
plot([min(x_vals), max(x_vals)], [W + W/2, W + W/2], 'k', 'LineWidth', 1.5);

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Optimized Trajectory with Lane Boundaries');
grid on;
axis equal;
legend('Trajectory', 'Start', 'End', 'Lane Boundaries');

%% Compute velocity, curvature, and acceleration
velocity = sqrt(x_dot.^2 + y_dot.^2);
curvature = abs(x_dot .* y_ddot - y_dot .* x_ddot) ./ (velocity.^3);
acceleration = sqrt(x_ddot.^2 + y_ddot.^2);

%% Plot velocity profile
figure;
plot(t_vals, velocity/0.447, 'r-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Velocity (mph)');
title('Velocity Profile');
grid on;

%% Plot radius of curvature
figure;
plot(t_vals, 1 ./ curvature, 'g-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Radius of Curvature (m)');
title('Curvature Profile');
grid on;

%% Plot acceleration profile
figure;
plot(t_vals, acceleration, 'b-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Acceleration Profile');
grid on;

%% Function Definitions

function J = cost_function(T, u_x, u_y, u, v, y0, ay, ax, W, lambda1, lambda2, lambda3)
    % Define symbolic variables
    syms a3 a4 a5 b3 b4 b5 real

    % Boundary equations
    eq1 = u_x + ax*T + 3*a3*T^2 + 4*a4*T^3 + 5*a5*T^4 - u == 0;
    eq2 = ax + 6*a3*T + 12*a4*T^2 + 20*a5*T^3 == 0;
    eq3 = u_x*T + (ax/2)*T^2 + a3*T^3 + a4*T^4 + a5*T^5 - (u_x*T + (ax/2)*T^2) == 0;
    eq4 = u_y + ay*T + 3*b3*T^2 + 4*b4*T^3 + 5*b5*T^4 == 0;
    eq5 = ay + 6*b3*T + 12*b4*T^2 + 20*b5*T^3 == 0;
    eq6 = y0 + u_y*T + 0.5*ay*T^2 + b3*T^3 + b4*T^4 + b5*T^5 - W == 0;

    % Solve equations
    sol = solve([eq1, eq2, eq3, eq4, eq5, eq6], [a3, a4, a5, b3, b4, b5]);
    a3 = double(sol.a3); a4 = double(sol.a4); a5 = double(sol.a5);
    b3 = double(sol.b3); b4 = double(sol.b4); b5 = double(sol.b5);

    % Compute acceleration and jerk
    syms t real
    x_ddot = ax + 6*a3*t + 12*a4*t^2 + 20*a5*t^3;
    y_ddot = ay + 6*b3*t + 12*b4*t^2 + 20*b5*t^3;
    x_jerk = diff(x_ddot, t);
    y_jerk = diff(y_ddot, t);

    % Compute max acceleration and jerk
    t_vals = linspace(0, T, 100);
    acc_vals = double(subs(sqrt(x_ddot^2 + y_ddot^2), t, t_vals));
    jerk_vals = double(subs(sqrt(x_jerk^2 + y_jerk^2), t, t_vals));

    a_max = max(acc_vals);
    j_max = max(jerk_vals);

    % Cost function: time, acceleration, and jerk penalties
    J = lambda1 * a_max^2 + lambda2 * T^2 + lambda3 * j_max^2;
end
