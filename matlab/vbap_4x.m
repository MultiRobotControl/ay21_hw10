clear

% Globals
% We will declare global variables that this function and the callbacks
% can all access
% When we receive a CORA1 odometry message, save it here.
global USV_ODOM;
% When we receive a CORA2 odometry message, save it here.
global USV2_ODOM;
% When we recieve a CORA3 odometry message, save it here
global USV3_ODOM;
% When we recieve a CORA4 odometry message, save it here
global USV4_ODOM;
% When we receive a rabbit posiition, save it here
global RABBIT_POSITION;
% When we receive a rabbit posiition, save it here
global RABBIT2_POSITION;


% Try to start ROS - if it is already started, restart
try
    rosinit
catch
    rosshutdown
    rosinit
end

% Subscribers
usv1_sub = rossubscriber('cora1/cora/sensors/p3d',@usv_odom_callback, ...
    'DataFormat', 'struct');
usv2_sub = rossubscriber('cora2/cora/sensors/p3d',@usv2_odom_callback, ...
    'DataFormat', 'struct');
usv3_sub = rossubscriber('cora3/cora/sensors/p3d',@usv3_odom_callback, ...
    'DataFormat', 'struct');
usv4_sub = rossubscriber('cora4/cora/sensors/p3d',@usv4_odom_callback, ...
    'DataFormat', 'struct');
rabbit_sub = rossubscriber('rabbit/rabbit',@rabbit_posit_callback, ...
    'DataFormat', 'struct');
rabbit2_sub = rossubscriber('rabbit2/rabbit',@rabbit2_posit_callback, ...
    'DataFormat', 'struct');
% Assign  message
RABBIT_POSITION = rosmessage('geometry_msgs/PointStamped');
RABBIT2_POSITION = rosmessage('geometry_msgs/PointStamped');

% Setup Publisher
cmd_pub = rospublisher('/cora1/cora/cmd_vel','geometry_msgs/Twist');
cora1_cmd_msg = rosmessage(cmd_pub);
cmd_pub2 = rospublisher('/cora2/cora/cmd_vel','geometry_msgs/Twist');
cora2_cmd_msg = rosmessage(cmd_pub2);
cmd_pub3 = rospublisher('/cora3/cora/cmd_vel','geometry_msgs/Twist');
cora3_cmd_msg = rosmessage(cmd_pub3);
cmd_pub4 = rospublisher('/cora4/cora/cmd_vel','geometry_msgs/Twist');
cora4_cmd_msg = rosmessage(cmd_pub4);

% Infinite loop
while true
    % Call a function to implement the VBAP algorithm.

    
    other_usv_odoms = {USV_ODOM.Pose.Pose.Position.X,USV_ODOM.Pose.Pose.Position.Y;...
                       USV2_ODOM.Pose.Pose.Position.X,USV2_ODOM.Pose.Pose.Position.Y;...
                       USV3_ODOM.Pose.Pose.Position.X,USV3_ODOM.Pose.Pose.Position.Y;...
                       USV4_ODOM.Pose.Pose.Position.X,USV4_ODOM.Pose.Pose.Position.Y};
    
    [u_c_usv1, r_c_usv1] = vbap_slmv(USV_ODOM, other_usv_odoms, RABBIT_POSITION);
    [u_c_usv2, r_c_usv2] = vbap_slmv2(USV2_ODOM, other_usv_odoms, RABBIT_POSITION);
    [u_c_usv3, r_c_usv3] = vbap_slmv(USV3_ODOM, other_usv_odoms, RABBIT2_POSITION);
    [u_c_usv4, r_c_usv4] = vbap_slmv2(USV4_ODOM, other_usv_odoms, RABBIT2_POSITION);
    
    %u_c_usv3 = min(u_c_usv3,0.9*u_c_usv1);
    %u_c_usv4 = min(u_c_usv4,0.9*u_c_usv2);
    
    % Publish the results
    cora1_cmd_msg.Linear.X = u_c_usv1;
    cora1_cmd_msg.Angular.Z = r_c_usv1;
    cora2_cmd_msg.Linear.X = u_c_usv2;
    cora2_cmd_msg.Angular.Z = r_c_usv2;
    cora3_cmd_msg.Linear.X = u_c_usv3;
    cora3_cmd_msg.Angular.Z = r_c_usv3;  
    cora4_cmd_msg.Linear.X = u_c_usv4;
    cora4_cmd_msg.Angular.Z = r_c_usv4; 
    send(cmd_pub, cora1_cmd_msg);
    send(cmd_pub2, cora2_cmd_msg); 
    send(cmd_pub3, cora3_cmd_msg); 
    send(cmd_pub4, cora4_cmd_msg); 
    ['u_c_usv1','u_c_usv2','u_c_usv3','u_c_usv4','r_c_usv1','r_c_usv2','r_c_usv3','r_c_usv4']
    [u_c_usv1,u_c_usv2,u_c_usv3,u_c_usv4,r_c_usv1,r_c_usv2,r_c_usv3,r_c_usv4]
    pause(0.1);   
end

    