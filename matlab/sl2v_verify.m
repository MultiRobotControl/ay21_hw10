%% Load .bag file and create bag file object. 

fname = '2021-06-07-20-41-36.bag'; 
bag = rosbag(fname);
 
% Display available topics and message types in bag file. 
bag.AvailableTopics

% Create a time series of the Odometry data
% Retrieve the messages as a cell array
odom1_msgs = select(bag,'Topic','/cora1/cora/sensors/p3d');
 
% Create a timeseries object of the subset of message fields we are interested in
odom1_ts = timeseries(odom1_msgs,'Pose.Pose.Position.X','Pose.Pose.Position.Y', ...
    'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y',...
    'Pose.Pose.Orientation.Z','Twist.Twist.Linear.X','Twist.Twist.Angular.Z');

odom2_msgs = select(bag,'Topic','/cora2/cora/sensors/p3d');
 
% Create a timeseries object of the subset of message fields we are interested in
odom2_ts = timeseries(odom2_msgs,'Pose.Pose.Position.X','Pose.Pose.Position.Y', ...
    'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y',...
    'Pose.Pose.Orientation.Z','Twist.Twist.Linear.X','Twist.Twist.Angular.Z');

% Create a time series of the Cmd data
% Retrieve the messages as a cell array
cmd1_msgs = select(bag,'Topic','/cora1/cora/cmd_vel');
 
cmd1_ts = timeseries(cmd1_msgs,'Linear.X','Linear.Y','Linear.Z',...
        'Angular.X','Angular.Y','Angular.Z');
    
cmd2_msgs = select(bag,'Topic','/cora2/cora/cmd_vel');
 
cmd2_ts = timeseries(cmd2_msgs,'Linear.X','Linear.Y','Linear.Z',...
        'Angular.X','Angular.Y','Angular.Z');

% Create a time series of the Odometry data
% Retrieve the messages as a cell array
rabbit_msgs = select(bag,'Topic','/rabbit');
 
% Create a timeseries object of the subset of message fields we are interested in
rabbit_ts = timeseries(rabbit_msgs,'Point.X','Point.Y','Point.Z');
%% 1) Plot X-Y Positions
str1 = 'Start'; 
str2 = 'End';

% Plot the Data index corresponding to 'Pose.Pose.Position.X' & 'Pose.Pose.Position.Y'
plot(odom1_ts.Data(:,1),odom1_ts.Data(:,2),'r'); hold on
plot(odom2_ts.Data(:,1),odom2_ts.Data(:,2),'g'); 
plot(rabbit_ts.Data(:,1),rabbit_ts.Data(:,2),'k--');
text(-250,-350-30,str1);
text(-765,875+30,str2);
plot(-250,-350,'ro');
plot(-765,875,'b*','MarkerSize',12);
title('X-Y Position of USV and Rabbit')
legend('USV1','USV2', 'Rabbit Position', 'Location','best')
xlabel('X')
ylabel('Y')
grid on

%% 2) Plot Distance of each USV with Virtual Leader
X0 = rabbit_ts.Data(1:end,1);
Y0 = rabbit_ts.Data(1:end,2);
X1 = odom1_ts.Data(2:end,1);
Y1 = odom1_ts.Data(2:end,2);
X2 = odom2_ts.Data(2:end,1);
Y2 = odom2_ts.Data(2:end,2);

dist1 = sqrt((X1-X0).^2 + (Y1-Y0).^2);  
dist2 = sqrt((X2-X0).^2 + (Y2-Y0).^2); 

figure; clf();
% Plot the Data index corresponding to 'Pose.Pose.Position.X' & 'Pose.Pose.Position.Y'
plot(odom1_ts.Time(2:end),dist1,'r'); hold on
plot(odom2_ts.Time(2:end),dist2,'g'); 
title('Distance Between Each USV and Virtual Leader')
legend('USV1','USV2','Location','best')
xlabel('Time')
ylabel('Distance')
grid on

%% 3) Plots of Distance between USVs

Udist1 = sqrt((X2-X1).^2 + (Y2-Y1).^2);

figure; clf();
% Plot the Data index corresponding to 'Pose.Pose.Position.X' & 'Pose.Pose.Position.Y'
plot(odom1_ts.Time(2:end), Udist1,'r'); hold on
title('Inter-Vehicle Distance')
%legend('USV1-2','USV2-3')
xlabel('Time')
ylabel('Distance')
hold off
grid on