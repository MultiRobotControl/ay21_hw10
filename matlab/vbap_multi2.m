function [u_c1, u_c2, r_c1, r_c2] = vbap_multi2(USV_ODOM, USV2_ODOM, RABBIT_POSITION)

    dx1 = RABBIT_POSITION.Point.X - USV_ODOM.Pose.Pose.Position.X;
    dy1 = RABBIT_POSITION.Point.Y - USV_ODOM.Pose.Pose.Position.Y;
    dx2 = RABBIT_POSITION.Point.X - USV2_ODOM.Pose.Pose.Position.X;
    dy2 = RABBIT_POSITION.Point.Y - USV2_ODOM.Pose.Pose.Position.Y;

    dx_cora = USV2_ODOM.Pose.Pose.Position.X - USV_ODOM.Pose.Pose.Position.X;
    dy_cora = USV2_ODOM.Pose.Pose.Position.Y - USV_ODOM.Pose.Pose.Position.Y;
    
    psi1_L = atan2(dy1,dx1);
    psi2_L = atan2(dy2,dx2);
    
    quat1 = USV_ODOM.Pose.Pose.Orientation; 
    angles1 = quat2eul([quat1.W quat1.X quat1.Y quat1.Z]); 
    psi1 = angles1(1);

    quat2 = USV2_ODOM.Pose.Pose.Orientation; 
    angles2 = quat2eul([quat2.W quat2.X quat2.Y quat2.Z]); 
    psi2 = angles2(1);

k_v = 0.1; k_h = 2.0; k_o = 0.1;
d_0 = 15; d_1 = 2 * d_0;

dist1 = sqrt(dx1^2 + dy1^2);
aerr1 = wrapToPi(psi1_L - psi1);
dist2 = sqrt(dx2^2 + dy2^2);
aerr2 = wrapToPi(psi2_L - psi2);

h_ij = sqrt(dx_cora^2 + dy_cora^2);
e_ij = k_o .* (h_ij-d_0);
psi_ij = atan2(dy_cora,dx_cora);
headerr = wrapToPi(psi_ij - psi1);

if h_ij <= d_1
    psiJ = e_ij * sign(headerr);
else
    psiJ = 0;
end
headerr1 = aerr1 + sum(psiJ);
headerr2 = aerr2 - sum(psiJ);

% Converge on Target if within 50m
tgt_x = -765; tgt_y = 875;
tdist1 = sqrt((tgt_x - USV_ODOM.Pose.Pose.Position.X)^2 + ...
        (tgt_y - USV_ODOM.Pose.Pose.Position.Y)^2);
tdist2 = sqrt((tgt_x - USV2_ODOM.Pose.Pose.Position.X)^2 + ...
        (tgt_y - USV2_ODOM.Pose.Pose.Position.Y)^2);
psi1_t = atan2(tgt_y - USV_ODOM.Pose.Pose.Position.Y,tgt_x - ...
    USV_ODOM.Pose.Pose.Position.X);
psi2_t = atan2(tgt_y - USV2_ODOM.Pose.Pose.Position.Y,tgt_x - ...
    USV2_ODOM.Pose.Pose.Position.X);

if tdist1 < 50
    dist1 = tdist1;
    dist2 = tdist2;
    headerr1 = wrapToPi(psi1_t - psi1);
    headerr2 = wrapToPi(psi2_t - psi2);
end

% Total Control Law
u_c1 = k_v * dist1;
u_c2 = k_v * dist2;
r_c1 = k_h * headerr1;
r_c2 = k_h * headerr2;

% Saturate
u_c1 = min(abs(u_c1),10.0);
u_c2 = min(abs(u_c2),10.0);
r_c1 = min(r_c1, 2*pi);
r_c2 = max(r_c2, -2*pi);
r_c1 = min(r_c1, 2*pi);
r_c2 = max(r_c2, -2*pi);
return