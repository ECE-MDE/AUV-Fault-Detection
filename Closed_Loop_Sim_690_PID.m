clear
clear all
% clf
rng('default') % For reproducibility

% names and corresponding indices for variables in state vector x
% x = [nu;        eta];
%      ^          ^
%      velocities positions
% ex: yaw = x(YAW);
global U V W ROLL_RATE PITCH_RATE YAW_RATE X Y Z ROLL PITCH YAW 

U = 1;
V = 2;
W = 3;

ROLL_RATE = 4;
PITCH_RATE = 5;
YAW_RATE = 6;

X = 7;
Y = 8;
Z = 9;

ROLL = 10;   % roll
PITCH = 11; % pitch
YAW = 12;   % yaw 

%% User defined settings
%initial time (sec):    In general there shouldn't be a need to use an
%                       initial time other than zero.
t0 = 0 ;

%final time (sec):      tf-t0 determines the length of the simulation
tf = 600;

%Controller Frequency:  The frequency with which the controller calculates
%                       a new control signal in Hertz
f_c = 10;
dt = 1/f_c;   %seconds
n_sim_iters = ceil(tf/dt);

%   The 690 dynamics have 12 states expressed in either the world     
%   (inertial) reference frame or the body reference frame. These are
%   grouped into two vectors, eta (the 6 position measurements in the world 
%   reference frame), and nu (the 6 velocity measurements in the body
%   reference frame).

%   eta=[x,y,z,roll,pitch,yaw] where x,y,z are the position in the
%   north,east,down world coordinate system. (NOTE: z>0 means the vehicle is
%   operating below the surface of the water) and roll, pitch, and yaw are
%   the vehicle attitude parameters 

eta0 = [0,0,10,0,0,0]'; 

%   nu=[u,v,w,p,q,r] where u,v,w are surge,sway, and heave and are the
%   linear velocities in the body reference frame and p,q,r are the
%   roll-rate, pitch-rate, and yaw-rate in the body reference frame

% CAN GIVE INITIAL VELOCITY
nu0 = [0,0,0,0,0,0]';

%Combine the two sets of initial conditions to create a single state vector
x0=[nu0;eta0];

%User-Defined Command Signals: 
% This simulation requires the user to define command signals for AUV depth
% and yaw. In general the yaw commands can be abstracted to a trajectory in
% x,y coordinates with additional effort. 

%THINGS WE SHOULD MESS WITH
% Max speed 3-4 m/s
depth_command = [5*ones(1,2000),10*ones(1,2000),5*ones(1,2001)];
yaw_command = [zeros(1,3000),60*ones(1,3001)];

%Speed command (m/sec): Speed commands are executed open loop so the
%simulation accepts a desired speed as part of its control vector

speed_command = 1.5*ones(1,ceil(tf/dt)+1);

%% Initialize PID controllers for 690 AUV

%Control gains for 690 PIDs (from 690 Config files used currently in AVL)

%Pitch Control gains
kp_pitch = 4.5; %proportional gain
ki_pitch = 0.2; %integral gain
kd_pitch = 7.0; %differential gain
i_output_max_pitch = 5*pi/180; %Limits on integrator term to prevent windup
output_max_pitch = 20*pi/180; %Limits on total output of the controller

%Yaw Control gains
kp_yaw = 3.0; %proportional gain
ki_yaw = 0.005; %integral gain
kd_yaw = 3.0; %differential gain
i_output_max_yaw = 5*pi/180; %Limits on integrator term to prevent windup
output_max_yaw = 20*pi/180; %Limits on total output of the controller

%Depth Control gains
kp_depth = -0.07; %proportional gain
ki_depth = -0.001; %integral gain
kd_depth = 0.095; %differential gain
i_output_max_depth = 5*pi/180; %Limits on integrator term to prevent windup
output_max_depth = 20*pi/180; %Limits on total output of the controller

%Instantiate PID objects for each control loop
pitch_loop = PID_690(kp_pitch,ki_pitch,kd_pitch,true,1,dt,i_output_max_pitch,output_max_pitch);
yaw_loop = PID_690(kp_yaw,ki_yaw,kd_yaw,true,2,dt,i_output_max_yaw,output_max_yaw);
depth_loop = PID_690(kp_depth,ki_depth,kd_depth,false,1,dt,i_output_max_depth,output_max_depth);

%% Simulation
delta = []; %control input vector

x_tot = zeros(n_sim_iters, 12);
t_tot = zeros(n_sim_iters, 1);

x_hat_tot = zeros(n_sim_iters, 12);
x_hat_cov_tot = zeros(n_sim_iters, 12, 12);

noisy_residual_tot = zeros(n_sim_iters, 12);
noisy_residual_cov_tot = zeros(n_sim_iters, 12, 12);

true_residual_tot = zeros(n_sim_iters, 12);
true_residual_cov_tot = zeros(n_sim_iters, 12, 12);
y_tot = zeros(n_sim_iters, 12);


ekf_x0 = zeros(12, 1);
ekf_x0(YAW) = pi/2;
ekf = extendedKalmanFilter(@(x, u)discretized_euler(x, u, dt), @measure, ekf_x0, 'HasMeasurementWrapping', false);
% filter = extendedKalmanFilter(@(x, u)discretized_exact(x, u, dt), @measure, x0, 'HasMeasurementWrapping', true);

% ekf.ProcessNoise = 1.0;
ekf.MeasurementNoise = zeros(12, 12);
ekf.MeasurementNoise(X, X) = 100;
ekf.MeasurementNoise(Y, Y) = 100;
ekf.MeasurementNoise(Z, Z) = 100;
ekf.MeasurementNoise(U, U) = 1;
ekf.MeasurementNoise(V, V) = 1;
ekf.MeasurementNoise(W, W) = 1;

%Simulation Loop
for i=1:n_sim_iters+1

    %time interval of simulation for this step in the loop
    tspan = [t0,t0+dt]; 

    depth_rate = x0(3);
    pitch_rate = x0(5);
    yaw_rate = x0(6);

    depth = x0(9);
    roll = x0(10);
    pitch = x0(11);
    yaw = x0(12);

    %Calculate PID control signals for AUV control surfaces
    
    %Pitch control consists of an inner and outer loop. The outer loop PID
    %takes depth command as an input and produces a pitch command as an
    %output
    
    [pitch_command(i),depth_loop] = depth_loop.control_loop(depth,depth_rate,depth_command(i));

    
    %The inner loop PID takes pitch command as an input and produces an
    %elevator command as output

    [del_e,pitch_loop] = pitch_loop.control_loop(pitch,pitch_rate,pitch_command(i));

    %Yaw control consists of a single loop PID controller

    [del_r,yaw_loop,yaw_command(i)] = yaw_loop.control_loop(yaw,yaw_rate, yaw_command(i));
    
    %The current 690 control architecture does not use roll control.

    del_roll = 0;
    
    %Fin mixing calculations
    del_r_act = -del_r*cos(roll)-del_e*sin(roll);
    del_e_act = -del_r*sin(roll)+del_e*cos(roll);

    Top_Fin_Angle = -del_r_act+del_roll;
    Bottom_Fin_Angle = del_r_act+del_roll;
    Port_Fin_Angle = del_e_act+del_roll;
    Starboard_Fin_Angle = -del_e_act+del_roll;
    
    %Constructing the control vector
    delta(:,i) = [Top_Fin_Angle;Port_Fin_Angle;Bottom_Fin_Angle;Starboard_Fin_Angle];
    
    u = [delta(:,i);speed_command(i)];

    [t,x] = ode45(@(t,x)dynamics(x,u),tspan,x0); %insert 690 dynamics

    % set a signal to 0 to simulate sensor error
    % insert step change to measurement to simulate being knocked off
    % course (depth, yaw, other)
    x0 = x(end,:);
    t0 = t(end);

    x_tot(i, :) = x0;
    t_tot(i, :) = t0;
    
    % Simulate sensor noise
    y = x0;
    y(X:Z) = y(X:Z) + normrnd(0, 10, size(x0(X:Z)));
    y(U:W) = y(U:W) + normrnd(0, 10, size(x0(U:W)));

    % Update EKF with noisy sensor measurements
    [corrected_x, corrected_x_cov] = correct(ekf,y,u);

    [x_hat, x_hat_cov] = predict(ekf,u);

    % Get EKF error vs. sensor correction and vs. ground truth
    [noisy_residual, noisy_residual_cov] = residual(ekf, y, u);
    [true_residual, true_residual_cov] = residual(ekf, x0, u);

    y_tot(i, :) = y;
    x_hat_tot(i, :) = x_hat';
    x_hat_cov_tot(i, :, :) = x_hat_cov;

    noisy_residual_tot(i, :) = noisy_residual;
    noisy_residual_cov_tot(i, :, :) = noisy_residual_cov;

    true_residual_tot(i, :) = true_residual;
    true_residual_cov_tot(i, :, :) = true_residual_cov;
    
end
%% Plot results
x_tot=x_tot';
x_hat_tot = x_hat_tot';
y_tot = y_tot';
plot_t=0:dt:ceil(tf/dt)*dt;

set(0,'DefaultFigureWindowStyle','docked');

% figure
% grid on
% hold on
% plot(plot_t,pitch_command*180/pi,'--k')
% plot(t_tot,x_tot(11,:)*180/pi,'b','LineWidth',1.5)
% legend('commanded pitch','PID simulated pitch');
% ylabel('pitch (degrees)')
% xlabel('time(seconds)')
% title('Simulated Vehicle Pitch')
% hold off

% figure
% grid on
% plot(plot_t,yaw_command*180/pi,'--k')
% title('Simulated Vehicle Yaw')
% hold on
% plot(t_tot,x_tot(12,:)*180/pi,'b','LineWidth',1.5)
% ylabel('yaw (degrees)')
% xlabel('time(seconds)')
% legend('commanded yaw','PID simulated yaw');
% hold off

% figure
% hold on
% plot(plot_t,depth_command,'--k','LineWidth',1.5);
% plot(t_tot,x_tot(9,:),'b');
% grid on
% xlabel('time (seconds');
% ylabel('depth (m)');
% legend('Commanded Depth','PID depth(m)')

figure
grid on
hold on
plot(NaN,'Color', 'r', 'LineWidth', 1.5)
plot(NaN,'Color', 'b', 'LineWidth', 1.5)
plot(t_tot, true_residual_tot, 'LineWidth', 1.5, 'Color', 'r')
plot(t_tot, noisy_residual_tot, 'LineWidth', 1.5, 'Color', 'b')
legend('True Residual', 'Noisy Residual', ...
       'u', 'v', 'w', 'p', 'q', 'r', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', ...
       'u', 'v', 'w', 'p', 'q', 'r', 'x', 'y', 'z', 'roll', 'pitch', 'yaw')
xlabel('Time (s)')
plotbrowser
plotedit('off')

figure
grid on
hold on
plot(NaN,'Color', 'r', 'LineWidth', 1.5)
plot(NaN,'Color', 'b', 'LineWidth', 1.5)
plot(t_tot, x_tot, 'LineWidth', 1.5, 'Color', 'r')
plot(t_tot, x_hat_tot, 'LineWidth', 1.5, 'Color', 'b')
legend('Ground Truth', 'Estimate', ...
       'u', 'v', 'w', 'p', 'q', 'r', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', ...
       'u', 'v', 'w', 'p', 'q', 'r', 'x', 'y', 'z', 'roll', 'pitch', 'yaw')
xlabel('Time (s)')

figure
hold on
grid on
plot3(x_hat_tot(7,:),x_hat_tot(8,:),x_hat_tot(9,:),'LineWidth',1.5, 'Color', 'r')
plot3(x_tot(7,:),x_tot(8,:),x_tot(9,:),'LineWidth',1.5, 'Color', 'b')
plot3(y_tot(7,:),y_tot(8,:),y_tot(9,:),'LineWidth',1.5, 'Color', 'g')
legend('estimate', 'ground truth', 'measurement')
title('Vehicle Trajectory in North-East-Down Coordinate System')
xlabel('north (m)')
ylabel('east (m)')
zlabel('depth (m)')
pbaspect([1 1 1])
daspect([1 1 1])
view(3)

hold off

% plotbrowser'on') % open the plot browser cause it's the best thing ever

%Helper function used for running ODE45 on the 690 vehicle dynamics
function x_dot = dynamics(x,u)
nu = x(1:6);
eta = x(7:12);
[nu_dot,eta_dot] = Sim_690_Rev12(nu,u,eta);
x_dot=[nu_dot;eta_dot];
end

function x_f = discretized_exact(x_i,u,dt)
    tspan = [0,dt]; 
    [~,x_span] = ode45(@(t,x)dynamics(x,u),tspan,x_i); %insert 690 dynamics
    x_f = x_span(end,:);
end

function x_f = discretized_euler(x_i,u,dt)
    x_f = x_i + dynamics(x_i, u) * dt;
end

function [y, bounds] = measure(x,u)
    global U V W ROLL_RATE PITCH_RATE YAW_RATE X Y Z ROLL PITCH YAW

    y = x;
%     y(X:Z) = y(X:Z) + normrnd(0, 1, size(y(X:Z)));
%     bounds = [
%     -inf inf;
%     -inf inf;
%     -inf inf;
%     -inf inf;
%     -inf inf;
%     -inf inf;
%     -inf inf;
%     -inf inf;
%     -inf inf;
% %     -pi, pi;
% %     -pi, pi;
% %     -pi, pi;
%     -pi/2, pi/2;
%     -pi/2, pi/2;
% %     -pi, pi;
%     0, 2*pi;
%     ];
end