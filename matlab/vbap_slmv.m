function [u_c, r_c] = vbap_slmv(usv_odom, other_usv_odoms, RABBIT_POSITION)
% Function prototype for implementing 
% Virtual Body, Artificial Potential - Single Leader, Single Vehicle

% Get odom data from CORA
u = usv_odom.Twist.Twist.Linear.X;    % CORA Surge Velocity [m/s]
v = usv_odom.Twist.Twist.Linear.Y;    % CORA Sway Velocity [m/s]
q = [usv_odom.Pose.Pose.Orientation.W,usv_odom.Pose.Pose.Orientation.X,...
    usv_odom.Pose.Pose.Orientation.Y,usv_odom.Pose.Pose.Orientation.Z];
e = quat2eul(q);
psi = e(:,1); % CORA Yaw [rad]

% Determine Distance between CORA and rabbit
dist2rabbit_x = RABBIT_POSITION.Point.X - usv_odom.Pose.Pose.Position.X;
dist2rabbit_y = RABBIT_POSITION.Point.Y - usv_odom.Pose.Pose.Position.Y;

% Relative steering angle from CORA to rabbit
psi_error = wrapToPi(atan2(dist2rabbit_y,dist2rabbit_x)-psi);

% "Spring" btwn USVs
d_o = 10;    %nominal spring length [m]
d_1 = 1.5*d_o; % Max influence length [m]

% control gains
k_v = 0.2; % Surge Velocity gain
k_h = 10;    % Steering gain
k_o = -0.2;  % Other USVs "spring" force

% ship control law 
u_c_ldr = k_v * sqrt(dist2rabbit_x^2 +dist2rabbit_y^2);  % Surge Velocity Command
r_c_ldr = k_h * psi_error;        % Yaw Rate Command i.e Turn Rate Cmd

% spring control law
u_c_o = 0;

for i=1:height(other_usv_odoms)
h_ij = sqrt((other_usv_odoms{i,1}-usv_odom.Pose.Pose.Position.X)^2 + ...
       (other_usv_odoms{i,2}-usv_odom.Pose.Pose.Position.Y)^2);
e_ij = k_o * (h_ij-d_o);
    if h_ij < d_1
        r_c_o(i) = (e_ij*sign(atan2(other_usv_odoms{i,2}-usv_odom.Pose.Pose.Position.Y,...
        other_usv_odoms{i,1}-usv_odom.Pose.Pose.Position.X)-psi)); 
    elseif h_ij == 0
        r_c_o(i)=0;
    else
        r_c_o(i)=0;
    end
   % r_c_o(i)=min(r_c_o(i),pi/2);
end 

% Total Control Law

u_c = u_c_ldr + u_c_o;
r_c = r_c_ldr + sum(r_c_o);

%[u_c_ldr, r_c, r_c_ldr, r_c_o(1),r_c_o(2),r_c_o(3)]

% Saturate
u_c = min(abs(u_c),25);
r_c = min(r_c, 2*pi);
r_c = max(r_c, -2*pi);

return
