%% Plot data from prevoius trial
figure;
clf;
x = -output_data(:,12); % actual foot position in X (negative due to direction motors are mounted)
y = output_data(:,13); % actual foot position in Y
plot(x,y);

axis equal
a = axis;
axis([-.3 .3 -.3 .2])

%% Get user input (4 clicks)
[X Y] = ginput(4)
pts = [-X' ; Y']; % make X negative due to direction motors are mounted
hold on
plot(X,Y,'r--o','MarkerFaceColor','r');
t = 0:.01:1;
% add points to start and end trajectory with zero velocity and acceleration
pts = [pts(:,1) pts(:,1)  pts pts(:,end) pts(:,end)];

%% Evaluate bezier trajectory
traj = polyval_bz(pts,t);
plot(-traj(1,:), traj(2,:), 'g'); % make x-values negative due to direction motors are mounted
legend('Workspace Limit','Control Points','Trajectory');

%% Provide matrix "pts" as output for user
pts