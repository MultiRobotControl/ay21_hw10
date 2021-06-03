function [v1_c,r1_c,v2_c,r2_c,v3_c,r3_c] = vbap_multi3(USV1_ODOM,USV2_ODOM,USV3_ODOM,RABBIT_POSITION)
% Function prototype for implementing 
kv = 0.1; kh = 3; k0 = 0.025;
d0 = 25; d1 = 2*d0; 

% Distance errors with rabbit
Xerr1 = RABBIT_POSITION.Point.X - USV1_ODOM.Pose.Pose.Position.X;
Yerr1 = RABBIT_POSITION.Point.Y - USV1_ODOM.Pose.Pose.Position.Y;
Xerr2 = RABBIT_POSITION.Point.X - USV2_ODOM.Pose.Pose.Position.X;
Yerr2 = RABBIT_POSITION.Point.Y - USV2_ODOM.Pose.Pose.Position.Y;
Xerr3 = RABBIT_POSITION.Point.X - USV3_ODOM.Pose.Pose.Position.X;
Yerr3 = RABBIT_POSITION.Point.Y - USV3_ODOM.Pose.Pose.Position.Y;

% Distance errors with each other 
XUerr1 = USV2_ODOM.Pose.Pose.Position.X - USV1_ODOM.Pose.Pose.Position.X;
YUerr1 = USV2_ODOM.Pose.Pose.Position.Y - USV1_ODOM.Pose.Pose.Position.Y; 
XUerr2 = USV2_ODOM.Pose.Pose.Position.X - USV3_ODOM.Pose.Pose.Position.X;
YUerr2 = USV2_ODOM.Pose.Pose.Position.Y - USV3_ODOM.Pose.Pose.Position.Y; 
XUerr3 = USV3_ODOM.Pose.Pose.Position.X - USV1_ODOM.Pose.Pose.Position.X;
YUerr3 = USV3_ODOM.Pose.Pose.Position.Y - USV1_ODOM.Pose.Pose.Position.Y; 

% Convert to quaternions
quat1 = USV1_ODOM.Pose.Pose.Orientation; 
angles1 = quat2eul([quat1.W quat1.X quat1.Y quat1.Z]); 
psi1 = angles1(1);
quat2 = USV2_ODOM.Pose.Pose.Orientation; 
angles2 = quat2eul([quat2.W quat2.X quat2.Y quat2.Z]); 
psi2 = angles2(1);
quat3 = USV3_ODOM.Pose.Pose.Orientation; 
angles3 = quat2eul([quat3.W quat3.X quat3.Y quat3.Z]); 
psi3 = angles3(1);

% Desired heading towards leader
psiLead1 = atan2(Yerr1,Xerr1);
psiLead2 = atan2(Yerr2,Xerr2);
psiLead3 = atan2(Yerr3,Xerr3);

% Distance and heading errors between individual USV and rabbit 
distErr1 = sqrt(Xerr1^2 + Yerr1^2); 
headErr1 = wrapToPi(psiLead1 - psi1); 
distErr2 = sqrt(Xerr2^2 + Yerr2^2); 
headErr2 = wrapToPi(psiLead2 - psi2); 
distErr3 = sqrt(Xerr3^2 + Yerr3^2); 
headErr3 = wrapToPi(psiLead3 - psi3); 

% Spring force
h12 = sqrt(XUerr1.^2 + YUerr1.^2);
e12 = k0 * (h12 - d0); 
psi12 = atan2(YUerr1, XUerr1); 
UheadErr1 = (psi12 - psi1); % psiI = psi1 
UdistErr1 = h12 - d0; 
h23 = sqrt(XUerr2.^2 + YUerr2.^2);
e23 = k0 * (h23 - d0); 
psi23 = atan2(YUerr2, XUerr2); 
UheadErr2 = (psi23 - psi2); 
UdistErr2 = h23 - d0; 
h13 = sqrt(XUerr3.^2 + YUerr3.^2);
e13 = k0 * (h13 - d0); 
psi13 = atan2(YUerr3, XUerr3); 
UheadErr3 = (psi13 - psi3); 
UdistErr3 = h13 - d0; 

if h12<d1
    psiJ1 = e12 * sign(wrapToPi(UheadErr1)); 
else 
    psiJ1 = 0;
end
if h23<d1
    psiJ2 = e23 * sign(wrapToPi(UheadErr2)); 
else 
    psiJ2 = 0;
end
if h13<d1
    psiJ3 = e13 * sign(wrapToPi(UheadErr3)); 
else 
    psiJ3 = 0;
end

% Control commands
v1_c = kv * distErr1;
r1_c = kh * headErr1 + psiJ1 + psiJ3; 
v2_c = kv * distErr2;
r2_c = kh * headErr2 + psiJ1 + psiJ2;
v3_c = kv * distErr3;
r3_c = kh * headErr3 + psiJ2 + psiJ3;

% Saturation
v1_c = min(abs(v1_c),7.5);
v2_c = min(abs(v2_c),7.5);
v3_c = min(abs(v3_c),7.5);
r1_c = min(r1_c, 2*pi);
r1_c = max(r1_c, -2*pi);
r2_c = min(r2_c, 2*pi);
r2_c = max(r2_c, -2*pi);
r3_c = min(r3_c, 2*pi);
r3_c = max(r3_c, -2*pi);


fprintf("USV1--> PsiLead=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    psiLead1,psi1,headErr1,r1_c,distErr1,v1_c);
fprintf("USV2--> PsiLead=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    psiLead2,psi2,headErr2,r2_c,distErr2,v2_c);
fprintf("USV3--> PsiLead=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    psiLead3,psi3,headErr3,r3_c,distErr3,v3_c);

fprintf("USV1 to USV2 --> hIJ=%.2f, Heading Err=%.2f, Distance Err (hIJ-d0)=%.2f \n", ...
    h12,UheadErr1,UdistErr1);
fprintf("USV2 to USV3 --> hIJ=%.2f, Heading Err=%.2f, Distance Err (hIJ-d0)=%.2f \n", ...
    h23,UheadErr2,UdistErr2);
fprintf("USV3 to USV1 --> hIJ=%.2f, Heading Err=%.2f, Distance Err (hIJ-d0)=%.2f \n", ...
    h13, UheadErr3,UdistErr3);

return