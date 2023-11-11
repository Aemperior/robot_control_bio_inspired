% This is the main MATLAB script for Lab 5.
%
% You will need to modify the Mbed code and this script, but should not need to make any other changes.
%
%% SET YOUR INPUTS HERE

%% Foot trajectory
% Choose mode
%traj_mode = 0 %bezier
traj_mode = 1 %ellipse
%traj_mode = 2 %teleop
%% Bezier
const_point = [-0.1; -0.13]; %[x;y] or [q1,q2] constant coordinate (x,q1,q2 coordinates should be opposite sign due to direction motors are mounted)
pts_foot_R = repmat(const_point,1,8);
pts_foot_R = [0.1624    0.1624    0.1624    0.0619   -0.1636    0.0035    0.0035    0.0035;
               -0.1265   -0.1265   -0.1265   -0.2527   -0.2293   -0.1417   -0.1417   -0.1417]; % YOUR BEZIER PTS HERE

pts_foot_L = repmat(const_point,1,8);
pts_foot_L = [-0.0970   -0.0970   -0.0970    0.1671    0.1600    0.0748    0.0748    0.0748
              -0.1335   -0.1335   -0.1335   -0.1230   -0.1289   -0.1943   -0.1943   -0.1943]; % YOUR BEZIER PTS HERE

%% Ellipse
p_trajR.omega = 5;
p_trajR.x0   = 0.05;
p_trajR.y0   = -.15;
p_trajR.ry     = 0.035; % height of ellipse
p_trajR.rx    = 0.035;  % width of ellipse
p_trajR.phase_delay = pi; 

p_trajL.omega = 5;
p_trajL.x0   = 0.05;
p_trajL.y0   = -.15;
p_trajL.ry     = 0.035; % height of ellipse
p_trajL.rx    = 0.035;  % width of ellipse
p_trajL.phase_delay = 0;

% Initial leg angles for encoder resets (negative of q1,q2 in lab handout due to direction motors are mounted)
angleR1_init = 0;
angleR2_init = -pi/2; 
angleL1_init = 0;
angleL2_init = -pi/2;


%% More stuff

% Total experiment time is buffer,trajectory,buffer
traj_time         = 10;
pre_buffer_time   = 2; % this should be 0 for constant points, 2 for Bezier trajectories
pre_buffer_timeR  = 2; %Don't change, this doesn't work yet 
pre_buffer_timeL  = 2; %Don't change, this doesn't work yet 
post_buffer_time  = 2;

% Gains for impedance controller
% If a gain is not being used in your Mbed code, set it to zero
% For joint space control, use K_xx for K1, K_yy for K2, D_xx for D1, D_yy for D2
gains.K_xx = 10;
gains.K_yy = 10;
gains.K_xy = 0;

gains.D_xx = 0.05%5%1.5%0.5%;0.5;
gains.D_yy = 0.05%5%1.5%0.5;%0.5;
gains.D_xy = 0;

% Maximum duty cycle commanded by controller (should always be <=1.0)
duty_max   = 0.9;

%% Run Experiment
[output_data] = RunTrajectoryExperiment(angleR1_init, angleR2_init, pts_foot_R, ...
                                        angleL1_init, angleL2_init, pts_foot_L, ...
                                        p_trajR, p_trajL, traj_mode, ...
                                        traj_time, pre_buffer_time, pre_buffer_timeR, pre_buffer_timeL, post_buffer_time,...
                                        gains, duty_max);

%% Extract data
t = output_data(:,1);
xR = -output_data(:,12); % actual foot position in X (negative due to direction motors are mounted)
yR = output_data(:,13); % actual foot position in Y
   
xdesR = -output_data(:,16); % desired foot position in X (negative due to direction motors are mounted)
ydesR = output_data(:,17); % desired foot position in Y

xL = -output_data(:,30); % actual foot position in X (negative due to direction motors are mounted)
yL = output_data(:,31); % actual foot position in Y
   
xdesL = -output_data(:,34); % desired foot position in X (negative due to direction motors are mounted)
ydesL = output_data(:,35); % desired foot position in Y

%% Plot foot vs desired

%% Right foot
figure(3); clf;
subplot(211); hold on
plot(t,xdesR,'r-'); plot(t,xR);
xlabel('Time (s)'); ylabel('X (m)'); legend({'Desired Right Foot','Actual'});

subplot(212); hold on
plot(t,ydesR,'r-'); plot(t,yR);
xlabel('Time (s)'); ylabel('Y (m)'); legend({'Desired Right Foot','Actual'});

figure(4); clf; hold on
plot(xdesR,ydesR,'r-'); plot(xR,yR,'k');
xlabel('X (m)'); ylabel('Y (m)'); legend({'Desired Right Foot','Actual'});

%% Left foot

figure(5); clf;
subplot(211); hold on
plot(t,xdesL,'r-'); plot(t,xL);
xlabel('Time (s)'); ylabel('X (m)'); legend({'Desired Left Foot','Actual'});

subplot(212); hold on
plot(t,ydesL,'r-'); plot(t,yL);
xlabel('Time (s)'); ylabel('Y (m)'); legend({'Desired Left Foot','Actual'});

figure(6); clf; hold on
plot(xdesL,ydesL,'r-'); plot(xL,yL,'k');
xlabel('X (m)'); ylabel('Y (m)'); legend({'Desired Left Foot','Actual'});
