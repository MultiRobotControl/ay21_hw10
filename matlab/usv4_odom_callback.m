function usv4_odom_callback(~, msg)

% Example callback function to be called with odometry message

% For testing only - print a message when this function is called.
% disp('Received USV Odometry');

% Declare global variables to store odometry message
global USV4_ODOM;

USV4_ODOM = msg;
