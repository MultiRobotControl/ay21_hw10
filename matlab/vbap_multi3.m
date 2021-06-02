function [u_c1, u_c2, u_c3, r_c1, r_c2, r_c3] = vbap_multi3(USV_ODOM, USV2_ODOM, USV3_ODOM, RABBIT_POSITION)

    dx1 = RABBIT_POSITION.Point.X - USV_ODOM.Pose.Pose.Position.X;
    dy1 = RABBIT_POSITION.Point.Y - USV_ODOM.Pose.Pose.Position.Y;
    dx2 = RABBIT_POSITION.Point.X - USV2_ODOM.Pose.Pose.Position.X;
    dy2 = RABBIT_POSITION.Point.Y - USV2_ODOM.Pose.Pose.Position.Y;
    dx3 = RABBIT_POSITION.Point.X - USV3_ODOM.Pose.Pose.Position.X;
    dy3 = RABBIT_POSITION.Point.Y - USV3_ODOM.Pose.Pose.Position.Y;
    
    dx_cora12 = USV2_ODOM.Pose.Pose.Position.X - USV_ODOM.Pose.Pose.Position.X;
    dy_cora12 = USV2_ODOM.Pose.Pose.Position.Y - USV_ODOM.Pose.Pose.Position.Y;
    dx_cora13 = USV3_ODOM.Pose.Pose.Position.X - USV_ODOM.Pose.Pose.Position.X;
    dy_cora13 = USV3_ODOM.Pose.Pose.Position.Y - USV_ODOM.Pose.Pose.Position.Y;
    dx_cora23 = USV2_ODOM.Pose.Pose.Position.X - USV3_ODOM.Pose.Pose.Position.X;
    dy_cora23 = USV2_ODOM.Pose.Pose.Position.Y - USV3_ODOM.Pose.Pose.Position.Y;
    
    psi1_L = atan2(dy1,dx1);
    psi2_L = atan2(dy2,dx2);
    psi3_L = atan2(dy3,dx3);
    
    quat1 = USV_ODOM.Pose.Pose.Orientation; 
    angles1 = quat2eul([quat1.W quat1.X quat1.Y quat1.Z]); 
    psi1 = angles1(1);

    quat2 = USV2_ODOM.Pose.Pose.Orientation; 
    angles2 = quat2eul([quat2.W quat2.X quat2.Y quat2.Z]); 
    psi2 = angles2(1);
    
    quat3 = USV3_ODOM.Pose.Pose.Orientation; 
    angles3 = quat2eul([quat3.W quat3.X quat3.Y quat3.Z]); 
    psi3 = angles3(1);

k_v = 0.12; k_h = 1.3; k_o = 0.05;
d_0 = 25; d_1 = 2 * d_0;

dist1 = sqrt(dx1^2 + dy1^2);
aerr1 = wrapToPi(psi1_L - psi1);
dist2 = sqrt(dx2^2 + dy2^2);
aerr2 = wrapToPi(psi2_L - psi2);
dist3 = sqrt(dx3^2 + dy3^2);
aerr3 = wrapToPi(psi3_L - psi3);

h_12 = sqrt(dx_cora12^2 + dy_cora12^2);
e_12 = k_o .* (h_12-d_0);
psi_12 = atan2(dy_cora12,dx_cora12);
headerr12 = wrapToPi(psi_12 - psi1);

h_13 = sqrt(dx_cora13^2 + dy_cora13^2);
e_13 = k_o .* (h_13-d_0);
psi_13 = atan2(dy_cora13,dx_cora13);
headerr13 = wrapToPi(psi_13 - psi3);

h_23 = sqrt(dx_cora23^2 + dy_cora23^2);
e_23 = k_o .* (h_23-d_0);
psi_23 = atan2(dy_cora23,dx_cora23);
headerr23 = wrapToPi(psi_23 - psi2);

if h_12 <= d_1
    psiJ1 = e_12 * sign(headerr12);
else
    psiJ1 = 0;
end

if h_13 <= d_1
    psiJ3 = e_13 * sign(headerr13);
else
    psiJ3 = 0;
end

if h_23 <= d_1
    psiJ2 = e_23 * sign(headerr23);
else
    psiJ2 = 0;
end

% Total Control Law
u_c1 = k_v .* dist1;
u_c2 = k_v .* dist2;
u_c3 = k_v .* dist3;
r_c1 = k_h * (aerr1 + psiJ2 + psiJ3);
r_c2 = k_h * (aerr2 - psiJ1 - psiJ3);
r_c3 = k_h * (aerr3 + psiJ1 + psiJ2);

% Saturate
u_c1 = min(abs(u_c1),10.0);
u_c2 = min(abs(u_c2),10.0);
u_c3 = min(abs(u_c3),10.0);
r_c1 = min(r_c1, 2*pi);
r_c1 = max(r_c1, -2*pi);
r_c2 = min(r_c2, 2*pi);
r_c2 = max(r_c2, -2*pi);
r_c3 = min(r_c3, 2*pi);
r_c3 = max(r_c3, -2*pi);
return