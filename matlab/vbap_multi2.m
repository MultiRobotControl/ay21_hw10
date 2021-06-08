function [v1_c,r1_c,v2_c,r2_c] = vbap_multi2(USV1_ODOM,USV2_ODOM,RABBIT_POSITION)
% Function prototype for implementing 
kv = 0.1; kh = 3; k0 = 0.15;
d0 = 25; d1 = 2*d0; 

% Distance errors with rabbit
Xerr1 = RABBIT_POSITION.Point.X - USV1_ODOM.Pose.Pose.Position.X;
Yerr1 = RABBIT_POSITION.Point.Y - USV1_ODOM.Pose.Pose.Position.Y;
Xerr2 = RABBIT_POSITION.Point.X - USV2_ODOM.Pose.Pose.Position.X;
Yerr2 = RABBIT_POSITION.Point.Y - USV2_ODOM.Pose.Pose.Position.Y;

% Distance errors with each other 
XUerr1 = USV2_ODOM.Pose.Pose.Position.X - USV1_ODOM.Pose.Pose.Position.X;
YUerr1 = USV2_ODOM.Pose.Pose.Position.Y - USV1_ODOM.Pose.Pose.Position.Y; 

% Convert to quaternions
quat1 = USV1_ODOM.Pose.Pose.Orientation; 
angles1 = quat2eul([quat1.W quat1.X quat1.Y quat1.Z]); 
psi1 = angles1(1);
quat2 = USV2_ODOM.Pose.Pose.Orientation; 
angles2 = quat2eul([quat2.W quat2.X quat2.Y quat2.Z]); 
psi2 = angles2(1);

% Desired heading towards leader
psiLead1 = atan2(Yerr1,Xerr1);
psiLead2 = atan2(Yerr2,Xerr2);

% Distance and heading errors between individual USV and rabbit 
distErr1 = sqrt(Xerr1^2 + Yerr1^2); 
headErr1 = wrapToPi(psiLead1 - psi1); 
distErr2 = sqrt(Xerr2^2 + Yerr2^2); 
headErr2 = wrapToPi(psiLead2 - psi2); 

% Spring force
hIJ = sqrt(XUerr1.^2 + YUerr1.^2);
e12 = k0 * (hIJ - d0); 
psi12 = atan2(YUerr1, XUerr1); 
UheadErr1 = (psi12 - psi1); % psiI = psi1 
UdistErr1 = hIJ - d0; 

if hIJ<d1
    psiJ = e12 * sign(UheadErr1); 
else 
    psiJ = 0;
end

% Control commands
v1_c = kv * distErr1;
r1_c = kh * headErr1 + psiJ; 
v2_c = kv * distErr2;
r2_c = kh * headErr2 - psiJ;

% Saturation
v1_c = min(abs(v1_c),7.5);
v2_c = min(abs(v2_c),7.5);
r1_c = min(r1_c, 2*pi);
r1_c = max(r1_c, -2*pi);
r2_c = min(r2_c, 2*pi);
r2_c = max(r2_c, -2*pi);

fprintf("USV1--> PsiLead=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    psiLead1,psi1,headErr1,r1_c,distErr1,v1_c);
fprintf("USV2--> PsiLead=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    psiLead2,psi2,headErr2,r2_c,distErr2,v2_c);

fprintf("USV1 to USV2 --> hIJ=%.2f, PsiJ=%.2f, Heading Err=%.2f, Distance Err (hIJ-d0)=%.2f \n", ...
    hIJ,psiJ,UheadErr1,UdistErr1);

return