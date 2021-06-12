% ME4823 
% LT S. Royster
% HW8
% Spring 2021
clear all
close all
clc

%Load the file

fname = '2021-06-12-12-17-30.bag';       % Filename

%Create a bag file object with the file name
bag = rosbag(fname)
%Display a list of the topics and message types in the bag file
bag.AvailableTopics

%Create time series for the Odometry & Command data
%Retrieve the messages as a cell array
rabbit_msgs = select(bag,'Topic','rabbit/rabbit');
odom_cora1_msgs = select(bag,'Topic','cora1/cora/sensors/p3d');
cmd_cora1_msgs = select(bag,'Topic','cora1/cora/cmd_msg');
odom_cora2_msgs = select(bag,'Topic','cora2/cora/sensors/p3d');
cmd_cora2_msgs = select(bag,'Topic','cora2/cora/cmd_msg');

rabbit2_msgs = select(bag,'Topic','rabbit2/rabbit');
odom_cora3_msgs = select(bag,'Topic','cora3/cora/sensors/p3d');
cmd_cora3_msgs = select(bag,'Topic','cora3/cora/cmd_msg');
odom_cora4_msgs = select(bag,'Topic','cora4/cora/sensors/p3d');
cmd_cora4_msgs = select(bag,'Topic','cora4/cora/cmd_msg');

%Create a timeseries object of the subset of message fields we are interested in
odom_cora1_ts = timeseries(odom_cora1_msgs,'Pose.Pose.Position.X','Pose.Pose.Position.Y', ...
    'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z', ...
    'Twist.Twist.Linear.X','Twist.Twist.Linear.Y','Twist.Twist.Angular.Z');
cmd_cora1_ts = timeseries(cmd_cora1_msgs,'Linear.X','Linear.Y','Linear.Z','Angular.X','Angular.Y','Angular.Z');
odom_cora2_ts = timeseries(odom_cora2_msgs,'Pose.Pose.Position.X','Pose.Pose.Position.Y', ...
    'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z', ...
    'Twist.Twist.Linear.X','Twist.Twist.Linear.Y','Twist.Twist.Angular.Z');
cmd_cora2_ts = timeseries(cmd_cora2_msgs,'Linear.X','Linear.Y','Linear.Z','Angular.X','Angular.Y','Angular.Z');
rabbit_ts = timeseries(rabbit_msgs,'Point.X','Point.Y');

odom_cora3_ts = timeseries(odom_cora3_msgs,'Pose.Pose.Position.X','Pose.Pose.Position.Y', ...
    'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z', ...
    'Twist.Twist.Linear.X','Twist.Twist.Linear.Y','Twist.Twist.Angular.Z');
cmd_cora3_ts = timeseries(cmd_cora3_msgs,'Linear.X','Linear.Y','Linear.Z','Angular.X','Angular.Y','Angular.Z');
odom_cora4_ts = timeseries(odom_cora4_msgs,'Pose.Pose.Position.X','Pose.Pose.Position.Y', ...
    'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z', ...
    'Twist.Twist.Linear.X','Twist.Twist.Linear.Y','Twist.Twist.Angular.Z');
cmd_cora4_ts = timeseries(cmd_cora4_msgs,'Linear.X','Linear.Y','Linear.Z','Angular.X','Angular.Y','Angular.Z');
rabbit2_ts = timeseries(rabbit2_msgs,'Point.X','Point.Y');

%Plot X / Y Positions of the Rabbit
figure(1); clf();
subplot(2,1,1)
    %Plot the Data indicies for X and Y
    hold on
    plot(rabbit_ts.Data(:,2),rabbit_ts.Data(:,1))
    plot(odom_cora1_ts.Data(:,2),odom_cora1_ts.Data(:,1))
    plot(odom_cora2_ts.Data(:,2),odom_cora2_ts.Data(:,1))
    xlabel('East [m]')
    ylabel('North [m]')
    legend('Rabbit Position','CORA1 Position','CORA2 Position')
    title('Rabbit1 X/Y Position')
    axis padded
    grid on
subplot(2,1,2)
    %Plot the Data indicies for X and Y
    hold on
    plot(rabbit2_ts.Data(:,2),rabbit2_ts.Data(:,1))
    plot(odom_cora3_ts.Data(:,2),odom_cora3_ts.Data(:,1))
    plot(odom_cora4_ts.Data(:,2),odom_cora4_ts.Data(:,1))
    xlabel('East [m]')
    ylabel('North [m]')
    legend('Rabbit Position','CORA3 Position','CORA4 Position')
    title('Rabbit2 X/Y Position')
    axis padded
    grid on

%Plot X & Y vs Time
figure(2); clf();
%Plot X vs time
subplot(2,1,1)
hold on
plot(rabbit_ts.Time-rabbit_ts.Time(1),rabbit_ts.Data(:,1))
plot(odom_cora1_ts.Time-odom_cora1_ts.Time(1),odom_cora1_ts.Data(:,1))
plot(odom_cora2_ts.Time-odom_cora2_ts.Time(1),odom_cora2_ts.Data(:,1))
plot(rabbit2_ts.Time-rabbit2_ts.Time(1),rabbit2_ts.Data(:,1))
plot(odom_cora3_ts.Time-odom_cora3_ts.Time(1),odom_cora3_ts.Data(:,1))
plot(odom_cora4_ts.Time-odom_cora4_ts.Time(1),odom_cora4_ts.Data(:,1))
xlabel('Time [s]')
ylabel('X-position (NORTH) [m]')
legend('Rabbit position','CORA1 Position','CORA2 Position','Location','best')
title('X & Y position vs. Time')
axis padded
grid on
%Plot Y vs Time
subplot(2,1,2)
hold on
plot(rabbit_ts.Time-rabbit_ts.Time(1),rabbit_ts.Data(:,2))
plot(odom_cora1_ts.Time-odom_cora1_ts.Time(1),odom_cora1_ts.Data(:,2))
plot(odom_cora2_ts.Time-odom_cora2_ts.Time(1),odom_cora2_ts.Data(:,2))
plot(rabbit2_ts.Time-rabbit2_ts.Time(1),rabbit2_ts.Data(:,2))
plot(odom_cora3_ts.Time-odom_cora3_ts.Time(1),odom_cora3_ts.Data(:,2))
plot(odom_cora4_ts.Time-odom_cora4_ts.Time(1),odom_cora4_ts.Data(:,2))
grid on
xlabel('Time [s]')
ylabel('Y-position (EAST) [m]')
legend('Rabbit Position','CORA1 Position','CORA2 Position','Location','best')
axis padded

% Plot dist v. time
figure(3); clf();
hold on
plot(rabbit_ts.Time-rabbit_ts.Time(1),...
    sqrt((rabbit_ts.Data(:,1)-odom_cora1_ts.Data(1:length(rabbit_ts.Data(:,1)),1)).^2 +...
    (rabbit_ts.Data(:,2)-odom_cora1_ts.Data(1:length(rabbit_ts.Data(:,1)),2)).^2))
plot(rabbit_ts.Time-rabbit_ts.Time(1),...
    sqrt((rabbit_ts.Data(:,1)-odom_cora2_ts.Data(1:length(rabbit_ts.Data(:,1)),1)).^2 +...
    (rabbit_ts.Data(:,2)-odom_cora2_ts.Data(1:length(rabbit_ts.Data(:,1)),2)).^2))
plot(rabbit2_ts.Time-rabbit2_ts.Time(1),...
    sqrt((rabbit2_ts.Data(:,1)-odom_cora3_ts.Data(1:length(rabbit2_ts.Data(:,1)),1)).^2 +...
    (rabbit2_ts.Data(:,2)-odom_cora3_ts.Data(1:length(rabbit2_ts.Data(:,1)),2)).^2))
plot(rabbit2_ts.Time-rabbit2_ts.Time(1),...
    sqrt((rabbit2_ts.Data(:,1)-odom_cora4_ts.Data(1:length(rabbit2_ts.Data(:,1)),1)).^2 +...
    (rabbit2_ts.Data(:,2)-odom_cora4_ts.Data(1:length(rabbit2_ts.Data(:,1)),2)).^2))
xlabel('Time [s]')
ylabel('Distance [m]')
legend('Cora1 to rabbit','Cora2 to rabbit','Cora3 to rabbit2','Cora4 to rabbit2','Location','best')
title(' Distance vs. Time')
axis padded
grid on
% Plot dist v. time
figure(4); clf();
hold on
plot(odom_cora1_ts.Time-odom_cora1_ts.Time(1),...
    sqrt((odom_cora2_ts.Data(1:length(odom_cora1_ts.Data(:,1)),1)-odom_cora1_ts.Data(:,1)).^2 +...
    (odom_cora2_ts.Data(1:length(odom_cora1_ts.Data(:,1)),2)-odom_cora1_ts.Data(:,2)).^2))
plot(odom_cora1_ts.Time-odom_cora1_ts.Time(1),...
    sqrt((odom_cora3_ts.Data(1:length(odom_cora1_ts.Data(:,1)),1)-odom_cora1_ts.Data(:,1)).^2 +...
    (odom_cora3_ts.Data(1:length(odom_cora1_ts.Data(:,1)),2)-odom_cora1_ts.Data(:,2)).^2))
%plot(odom_cora3_ts.Time-odom_cora3_ts.Time(1),...
%    sqrt((odom_cora3_ts.Data(:,1)-odom_cora2_ts.Data(1:length(odom_cora3_ts.Data(:,1)),1)).^2 +...
%    (odom_cora3_ts.Data(:,2)-odom_cora2_ts.Data(1:length(odom_cora3_ts.Data(:,1)),2)).^2))
plot(odom_cora2_ts.Time-odom_cora2_ts.Time(1),...
    sqrt((odom_cora3_ts.Data(1:length(odom_cora2_ts.Data(:,1)),1)-odom_cora2_ts.Data(:,1)).^2 +...
    (odom_cora3_ts.Data(1:length(odom_cora2_ts.Data(:,1)),2)-odom_cora2_ts.Data(:,2)).^2))
plot(odom_cora4_ts.Time-odom_cora4_ts.Time(1),...
    sqrt((odom_cora3_ts.Data(1:length(odom_cora4_ts.Data(:,1)),1)-odom_cora4_ts.Data(:,1)).^2 +...
    (odom_cora3_ts.Data(1:length(odom_cora4_ts.Data(:,1)),2)-odom_cora4_ts.Data(:,2)).^2))
plot(odom_cora1_ts.Time-odom_cora1_ts.Time(1),...
    sqrt((odom_cora4_ts.Data(1:length(odom_cora1_ts.Data(:,1)),1)-odom_cora1_ts.Data(:,1)).^2 +...
    (odom_cora4_ts.Data(1:length(odom_cora1_ts.Data(:,1)),2)-odom_cora1_ts.Data(:,2)).^2))
%plot(odom_cora4_ts.Time-odom_cora4_ts.Time(1),...
%   sqrt((odom_cora4_ts.Data(:,1)-odom_cora2_ts.Data(1:length(odom_cora4_ts.Data(:,1)),1)).^2 +...
%    (odom_cora4_ts.Data(:,2)-odom_cora2_ts.Data(1:length(odom_cora4_ts.Data(:,1)),2)).^2))
plot(odom_cora2_ts.Time-odom_cora2_ts.Time(1),...
    sqrt((odom_cora4_ts.Data(1:length(odom_cora2_ts.Data(:,1)),1)-odom_cora2_ts.Data(:,1)).^2 +...
    (odom_cora4_ts.Data(1:length(odom_cora2_ts.Data(:,1)),2)-odom_cora2_ts.Data(:,2)).^2))

xlabel('Time [s]')
ylabel('Distance [m]')
legend('Cora1 & Cora2','Cora1 & Cora 3','Cora 2 & Cora 3',...
    'Cora 3 & Cora 4','Cora1 & Cora 4','Cora2 & Cora4','Location','best')
title('Distance vs. Time')
axis padded
grid on
