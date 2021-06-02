clear

% Globals
% We will declare global variables that this function and the callbacks
% can all access
% When we receive an odometry messge, save it here.
global USV_ODOM;
global USV2_ODOM;
% global OTHER_ODOM;
% When we receive a rabbit posiition, save it here
global RABBIT_POSITION;

% Try to start ROS - if it is already started, restart
try
    rosinit
catch
    rosshutdown
    rosinit
end

% Subscribers
usv_sub = rossubscriber('/cora1/cora/sensors/p3d',@usv_odom_callback, ...
    'DataFormat', 'struct');
usv2_sub = rossubscriber('/cora2/cora/sensors/p3d',@usv2_odom_callback, ...
    'DataFormat', 'struct');
rabbit = rossubscriber('/rabbit',@rabbit_pose_callback, ...
    'DataFormat', 'struct');
% For now we'll just assign a blank message
RABBIT_POSITION = rosmessage('geometry_msgs/PointStamped');

% Setup Publisher
cmd_pub = rospublisher('/cora1/cora/cmd_vel','geometry_msgs/Twist');
cmd_pub2 = rospublisher('/cora2/cora/cmd_vel','geometry_msgs/Twist');
cmd_msg = rosmessage(cmd_pub);
cmd_msg2 = rosmessage(cmd_pub2);

% Infinite loop
while true
    
%     OTHER_ODOM = USV2_ODOM;
    
    % Call a function to implement the VBAP algorithm.
    [u_c1,u_c2,r_c1,r_c2] = vbap_multi2(USV_ODOM, USV2_ODOM, RABBIT_POSITION);
    
    % Publish the results
    cmd_msg.Linear.X = u_c1;
    cmd_msg.Angular.Z = r_c1;
    send(cmd_pub, cmd_msg);
    cmd_msg2.Linear.X = u_c2;
    cmd_msg2.Angular.Z = r_c2;
    send(cmd_pub2, cmd_msg2);
    
    pause(0.1);
end