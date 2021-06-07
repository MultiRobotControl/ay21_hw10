function [u_c1,u_c2,u_c3,u_c4,r_c1,r_c2,r_c3,r_c4] = vbap_multi4(USV_ODOM,USV2_ODOM,USV3_ODOM,USV4_ODOM,RABBIT_POSITION)

    dx1 = RABBIT_POSITION.Point.X - USV_ODOM.Pose.Pose.Position.X;
    dy1 = RABBIT_POSITION.Point.Y - USV_ODOM.Pose.Pose.Position.Y;
    dx2 = RABBIT_POSITION.Point.X - USV2_ODOM.Pose.Pose.Position.X;
    dy2 = RABBIT_POSITION.Point.Y - USV2_ODOM.Pose.Pose.Position.Y;
    
    dx3 = USV_ODOM.Pose.Pose.Position.X - USV3_ODOM.Pose.Pose.Position.X;
    dy3 = USV_ODOM.Pose.Pose.Position.Y - USV3_ODOM.Pose.Pose.Position.Y;
    dx4 = USV2_ODOM.Pose.Pose.Position.X - USV4_ODOM.Pose.Pose.Position.X;
    dy4 = USV2_ODOM.Pose.Pose.Position.Y - USV4_ODOM.Pose.Pose.Position.Y;
    
    dx_Lcora = USV2_ODOM.Pose.Pose.Position.X - USV_ODOM.Pose.Pose.Position.X;
    dy_Lcora = USV2_ODOM.Pose.Pose.Position.Y - USV_ODOM.Pose.Pose.Position.Y;
    dx_Fcora = USV4_ODOM.Pose.Pose.Position.X - USV3_ODOM.Pose.Pose.Position.X;
    dy_Fcora = USV4_ODOM.Pose.Pose.Position.Y - USV3_ODOM.Pose.Pose.Position.Y;
    
    psi1_L = atan2(dy1,dx1);
    psi2_L = atan2(dy2,dx2);
    psi3_L = atan2(dy3,dx3);
    psi4_L = atan2(dy4,dx4);
    
    quat1 = USV_ODOM.Pose.Pose.Orientation; 
    angles1 = quat2eul([quat1.W quat1.X quat1.Y quat1.Z]); 
    psi1 = angles1(1);

    quat2 = USV2_ODOM.Pose.Pose.Orientation; 
    angles2 = quat2eul([quat2.W quat2.X quat2.Y quat2.Z]); 
    psi2 = angles2(1);
    
    quat3 = USV3_ODOM.Pose.Pose.Orientation; 
    angles3 = quat2eul([quat3.W quat3.X quat3.Y quat3.Z]); 
    psi3 = angles3(1);
    
    quat4 = USV4_ODOM.Pose.Pose.Orientation; 
    angles4 = quat2eul([quat4.W quat4.X quat4.Y quat4.Z]); 
    psi4 = angles4(1);

k_v = 0.1; k_h = 2.0; k_o = 0.1;
d_0 = 15; d_1 = 2 * d_0;

dist1 = sqrt(dx1^2 + dy1^2);
aerr1 = wrapToPi(psi1_L - psi1);
dist2 = sqrt(dx2^2 + dy2^2);
aerr2 = wrapToPi(psi2_L - psi2);
dist3 = sqrt(dx3^2 + dy3^2);
aerr3 = wrapToPi(psi3_L - psi3);
dist4 = sqrt(dx4^2 + dy4^2);
aerr4 = wrapToPi(psi4_L - psi4);

hL_ij = sqrt(dx_Lcora^2 + dy_Lcora^2);
eL_ij = k_o .* (hL_ij-d_0);
psiL_ij = atan2(dy_Lcora,dx_Lcora);
headerrL = wrapToPi(psiL_ij - psi1);

if hL_ij <= d_1
    psiLJ = eL_ij * sign(headerrL);
else
    psiLJ = 0;
end
headerr1 = aerr1 + sum(psiLJ);
headerr2 = aerr2 - sum(psiLJ);

hF_ij = sqrt(dx_Fcora^2 + dy_Fcora^2);
eF_ij = k_o .* (hF_ij-d_0);
psiF_ij = atan2(dy_Fcora,dx_Fcora);
headerrF = wrapToPi(psiF_ij - psi3);

if hF_ij <= d_1
    psiFJ = eF_ij * sign(headerrF);
else
    psiFJ = 0;
end
headerr3 = aerr3 + sum(psiFJ);
headerr4 = aerr4 - sum(psiFJ);

% Converge on Target if within 50m
tgt_x = -765; tgt_y = 875;
tdist1 = sqrt((tgt_x - USV_ODOM.Pose.Pose.Position.X)^2 + ...
        (tgt_y - USV_ODOM.Pose.Pose.Position.Y)^2);
tdist2 = sqrt((tgt_x - USV2_ODOM.Pose.Pose.Position.X)^2 + ...
        (tgt_y - USV2_ODOM.Pose.Pose.Position.Y)^2);
tdist3 = sqrt((tgt_x - USV3_ODOM.Pose.Pose.Position.X)^2 + ...
        (tgt_y - USV3_ODOM.Pose.Pose.Position.Y)^2);
tdist4 = sqrt((tgt_x - USV4_ODOM.Pose.Pose.Position.X)^2 + ...
        (tgt_y - USV4_ODOM.Pose.Pose.Position.Y)^2);
    
psi1_t = atan2(tgt_y - USV_ODOM.Pose.Pose.Position.Y,tgt_x - ...
    USV_ODOM.Pose.Pose.Position.X);
psi2_t = atan2(tgt_y - USV2_ODOM.Pose.Pose.Position.Y,tgt_x - ...
    USV2_ODOM.Pose.Pose.Position.X);
psi3_t = atan2(tgt_y - USV3_ODOM.Pose.Pose.Position.Y,tgt_x - ...
    USV3_ODOM.Pose.Pose.Position.X);
psi4_t = atan2(tgt_y - USV4_ODOM.Pose.Pose.Position.Y,tgt_x - ...
    USV4_ODOM.Pose.Pose.Position.X);

if tdist1 < 50
    dist1 = tdist1;
    dist2 = tdist2;
    dist3 = tdist3;
    dist4 = tdist4;
    headerr1 = wrapToPi(psi1_t - psi1);
    headerr2 = wrapToPi(psi2_t - psi2);
    headerr3 = wrapToPi(psi3_t - psi3);
    headerr4 = wrapToPi(psi4_t - psi4);
    k_v = 0.2;
end

% Total Control Law
u_c1 = k_v .* dist1;
u_c2 = k_v .* dist2;
u_c3 = k_v .* dist3;
u_c4 = k_v .* dist4;
r_c1 = k_h * headerr1;
r_c2 = k_h * headerr2;
r_c3 = k_h * headerr3;
r_c4 = k_h * headerr4;

% Saturate
u_c1 = min(abs(u_c1),10.0);
u_c2 = min(abs(u_c2),10.0);
u_c3 = min(abs(u_c3),10.0);
u_c4 = min(abs(u_c4),10.0);
r_c1 = min(r_c1, 2*pi);
r_c1 = max(r_c1, -2*pi);
r_c2 = min(r_c2, 2*pi);
r_c2 = max(r_c2, -2*pi);
r_c3 = min(r_c3, 2*pi);
r_c3 = max(r_c3, -2*pi);
r_c4 = min(r_c4, 2*pi);
r_c4 = max(r_c4, -2*pi);

return