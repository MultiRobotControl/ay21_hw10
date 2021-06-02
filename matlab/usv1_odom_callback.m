function usv1_odom_callback(~, msg)

% Example callback function to be called with odometry message

% For testing only - print a message when this function is called.
%disp('Received USV1 Odometry')

% Declare global variables to store odometry message
global USV1_ODOM;

USV1_ODOM = msg;
