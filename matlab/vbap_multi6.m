function [v1_c,r1_c,v2_c,r2_c,v3_c,r3_c,v4_c,r4_c,v5_c,r5_c,v6_c,r6_c] = vbap_multi6(USV1_ODOM,USV2_ODOM,USV3_ODOM,USV4_ODOM,USV5_ODOM,USV6_ODOM,RABBIT_POSITION)
% Function prototype for implementing 
kv = 0.1; kv2=0.05; kh = 3; k0 = 0.005;
D=50; d0 = 25; d1 = 2*d0; 

% Distance with Target
D1 = sqrt((USV1_ODOM.Pose.Pose.Position.X+765)^2 + (USV1_ODOM.Pose.Pose.Position.Y-875)^2);
D2 = sqrt((USV2_ODOM.Pose.Pose.Position.X+765)^2 + (USV2_ODOM.Pose.Pose.Position.Y-875)^2);
D3 = sqrt((USV3_ODOM.Pose.Pose.Position.X+765)^2 + (USV3_ODOM.Pose.Pose.Position.Y-875)^2);
D4 = sqrt((USV4_ODOM.Pose.Pose.Position.X+765)^2 + (USV4_ODOM.Pose.Pose.Position.Y-875)^2);
D5 = sqrt((USV5_ODOM.Pose.Pose.Position.X+765)^2 + (USV5_ODOM.Pose.Pose.Position.Y-875)^2);
D6 = sqrt((USV6_ODOM.Pose.Pose.Position.X+765)^2 + (USV6_ODOM.Pose.Pose.Position.Y-875)^2);


% Distance errors with rabbit
Xerr1 = RABBIT_POSITION.Point.X - USV1_ODOM.Pose.Pose.Position.X;
Yerr1 = RABBIT_POSITION.Point.Y - USV1_ODOM.Pose.Pose.Position.Y;
Xerr2 = RABBIT_POSITION.Point.X - USV2_ODOM.Pose.Pose.Position.X;
Yerr2 = RABBIT_POSITION.Point.Y - USV2_ODOM.Pose.Pose.Position.Y;
Xerr3 = RABBIT_POSITION.Point.X - USV3_ODOM.Pose.Pose.Position.X;
Yerr3 = RABBIT_POSITION.Point.Y - USV3_ODOM.Pose.Pose.Position.Y;
Xerr4 = RABBIT_POSITION.Point.X - USV4_ODOM.Pose.Pose.Position.X;
Yerr4 = RABBIT_POSITION.Point.Y - USV4_ODOM.Pose.Pose.Position.Y;
Xerr5 = RABBIT_POSITION.Point.X - USV5_ODOM.Pose.Pose.Position.X;
Yerr5 = RABBIT_POSITION.Point.Y - USV5_ODOM.Pose.Pose.Position.Y;
Xerr6 = RABBIT_POSITION.Point.X - USV6_ODOM.Pose.Pose.Position.X;
Yerr6 = RABBIT_POSITION.Point.Y - USV6_ODOM.Pose.Pose.Position.Y;

% Distance errors with each other 
XUerr1 = USV2_ODOM.Pose.Pose.Position.X - USV1_ODOM.Pose.Pose.Position.X;
YUerr1 = USV2_ODOM.Pose.Pose.Position.Y - USV1_ODOM.Pose.Pose.Position.Y; 
XUerr2 = USV2_ODOM.Pose.Pose.Position.X - USV3_ODOM.Pose.Pose.Position.X;
YUerr2 = USV2_ODOM.Pose.Pose.Position.Y - USV3_ODOM.Pose.Pose.Position.Y; 
XUerr3 = USV3_ODOM.Pose.Pose.Position.X - USV1_ODOM.Pose.Pose.Position.X;
YUerr3 = USV3_ODOM.Pose.Pose.Position.Y - USV1_ODOM.Pose.Pose.Position.Y;
XUerr4 = USV4_ODOM.Pose.Pose.Position.X - USV1_ODOM.Pose.Pose.Position.X;
YUerr4 = USV4_ODOM.Pose.Pose.Position.Y - USV1_ODOM.Pose.Pose.Position.Y; 
XUerr5 = USV5_ODOM.Pose.Pose.Position.X - USV2_ODOM.Pose.Pose.Position.X;
YUerr5 = USV5_ODOM.Pose.Pose.Position.Y - USV2_ODOM.Pose.Pose.Position.Y; 
XUerr6 = USV6_ODOM.Pose.Pose.Position.X - USV3_ODOM.Pose.Pose.Position.X;
YUerr6 = USV6_ODOM.Pose.Pose.Position.Y - USV3_ODOM.Pose.Pose.Position.Y; 

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
quat4 = USV4_ODOM.Pose.Pose.Orientation; 
angles4 = quat2eul([quat4.W quat4.X quat4.Y quat4.Z]); 
psi4 = angles4(1);
quat5 = USV5_ODOM.Pose.Pose.Orientation; 
angles5 = quat2eul([quat5.W quat5.X quat5.Y quat5.Z]); 
psi5 = angles5(1);
quat6 = USV6_ODOM.Pose.Pose.Orientation; 
angles6 = quat2eul([quat6.W quat6.X quat6.Y quat6.Z]); 
psi6 = angles6(1);

% Desired heading towards leader
psiLead1 = atan2(Yerr1,Xerr1);
psiLead2 = atan2(Yerr2,Xerr2);
psiLead3 = atan2(Yerr3,Xerr3);
psiLead4 = atan2(Yerr4,Xerr4);
psiLead5 = atan2(Yerr5,Xerr5);
psiLead6 = atan2(Yerr6,Xerr6);

% Distance and heading errors between individual USV and rabbit 
distErr1 = sqrt(Xerr1^2 + Yerr1^2); 
headErr1 = wrapToPi(psiLead1 - psi1); 
distErr2 = sqrt(Xerr2^2 + Yerr2^2); 
headErr2 = wrapToPi(psiLead2 - psi2); 
distErr3 = sqrt(Xerr3^2 + Yerr3^2); 
headErr3 = wrapToPi(psiLead3 - psi3); 
distErr4 = sqrt(Xerr4^2 + Yerr4^2); 
headErr4 = wrapToPi(psiLead4 - psi4); 
distErr5 = sqrt(Xerr5^2 + Yerr5^2); 
headErr5 = wrapToPi(psiLead5 - psi5); 
distErr6 = sqrt(Xerr6^2 + Yerr6^2); 
headErr6 = wrapToPi(psiLead6 - psi6); 

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

h14 = sqrt(XUerr4.^2 + YUerr4.^2);
e14 = k0 * (h14 - d0); 
psi14 = atan2(YUerr4, XUerr4); 
UheadErr4 = (psi14 - psi4); % psiI = psi1 
UdistErr4 = h14 - d0; 

h25 = sqrt(XUerr5.^2 + YUerr5.^2);
e25 = k0 * (h25 - d0); 
psi25 = atan2(YUerr5, XUerr5); 
UheadErr5 = (psi25 - psi5); 
UdistErr5 = h25 - d0; 
h36 = sqrt(XUerr6.^2 + YUerr6.^2);
e36 = k0 * (h36 - d0); 
psi36 = atan2(YUerr6, XUerr6); 
UheadErr6 = (psi36 - psi6); 
UdistErr6 = h36 - d0;

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

if h14<d1
    psiJ4 = e14 * sign(wrapToPi(UheadErr4)); 
else
    psiJ4 = 0;
end
if h25<d1
    psiJ5 = e25 * sign(wrapToPi(UheadErr5)); 
else 
    psiJ5 = 0;
end
if h36<d1
    psiJ6 = e36 * sign(wrapToPi(UheadErr6)); 
else 
    psiJ6 = 0;
end

% Control commands
v1_c = kv * distErr1;
r1_c = kh * headErr1 + psiJ1 + psiJ3 + psiJ4; 
v2_c = kv * distErr2;
r2_c = kh * headErr2 - psiJ1 - psiJ2 - psiJ5;
v3_c = kv * distErr3;
r3_c = kh * headErr3 + psiJ2 - psiJ3 + psiJ6;
v4_c = kv2 * distErr4;
r4_c = kh * headErr4 - psiJ1 - psiJ3 - psiJ4; 
v5_c = kv2 * distErr5;
r5_c = kh * headErr5 + psiJ1 + psiJ2 + psiJ5;
v6_c = kv2 * distErr6;
r6_c = kh * headErr6 - psiJ2 + psiJ3 - psiJ6;

if D1<D
    v1_c = 10;
    fprintf("Distance From Target=%.2f", D1);
end
if D2<D
    v2_c = 10;
    fprintf("Distance From Target=%.2f", D2);
end 
if D3<D
    v3_c = 10; 
    fprintf("Distance From Target=%.2f", D3);
end 
if D4<D
    v4_c = 10;
    fprintf("Distance From Target=%.2f", D4);
end
if D5<D
    v5_c = 10;
    fprintf("Distance From Target=%.2f", D5);
end 
if D6<D
    v6_c = 10; 
    fprintf("Distance From Target=%.2f", D6);
end 

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

v4_c = min(abs(v4_c),7.5);
v5_c = min(abs(v5_c),7.5);
v6_c = min(abs(v6_c),7.5);
r4_c = min(r4_c, 2*pi);
r4_c = max(r4_c, -2*pi);
r5_c = min(r5_c, 2*pi);
r5_c = max(r5_c, -2*pi);
r6_c = min(r6_c, 2*pi);
r6_c = max(r6_c, -2*pi);

fprintf("USV1--> PsiLead=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    psiLead1,psi1,kh*headErr1,r1_c,distErr1,v1_c);
fprintf("USV2--> PsiLead=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    psiLead2,psi2,kh*headErr2,r2_c,distErr2,v2_c);
fprintf("USV3--> PsiLead=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    psiLead3,psi3,kh*headErr3,r3_c,distErr3,v3_c);
fprintf("USV4--> PsiLead=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    psiLead4,psi4,kh*headErr4,r4_c,distErr4,v4_c);
fprintf("USV5--> PsiLead=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    psiLead5,psi5,kh*headErr5,r5_c,distErr5,v5_c);
fprintf("USV6--> PsiLead=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    psiLead6,psi6,kh*headErr6,r6_c,distErr6,v6_c);

fprintf("USV1 to USV2 --> hIJ=%.2f, Heading Err=%.2f, Distance Err (hIJ-d0)=%.2f, D1=%.2f \n", ...
    psiJ1+psiJ3,UheadErr1,UdistErr1,D1);
fprintf("USV2 to USV3 --> hIJ=%.2f, Heading Err=%.2f, Distance Err (hIJ-d0)=%.2f, D2=%.2f \n", ...
    psiJ1+psiJ2,UheadErr2,UdistErr2,D2);
fprintf("USV3 to USV1 --> hIJ=%.2f, Heading Err=%.2f, Distance Err (hIJ-d0)=%.2f, D3=%.2f \n", ...
    psiJ2+psiJ3, UheadErr3,UdistErr3, D3);
fprintf("USV1 to USV4 --> hIJ=%.2f, Heading Err=%.2f, Distance Err (hIJ-d0)=%.2f, D1=%.2f \n", ...
    psiJ1+psiJ4,UheadErr4,UdistErr4,D4);
fprintf("USV2 to USV5 --> hIJ=%.2f, Heading Err=%.2f, Distance Err (hIJ-d0)=%.2f, D2=%.2f \n", ...
    psiJ5+psiJ2,UheadErr5,UdistErr5,D5);
fprintf("USV3 to USV6 --> hIJ=%.2f, Heading Err=%.2f, Distance Err (hIJ-d0)=%.2f, D3=%.2f \n", ...
    psiJ3+psiJ6, UheadErr6,UdistErr6, D6);


return