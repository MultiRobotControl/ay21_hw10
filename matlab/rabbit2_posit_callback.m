function rabbit2_posit_callback(~, msg)

% Callback function to be called with rabbit postion message

% For testing only - prints a message when this function is called.
% disp('Received Rabbit Position')

% Declare global variables to store position message
global RABBIT2_POSITION;

RABBIT2_POSITION = msg;