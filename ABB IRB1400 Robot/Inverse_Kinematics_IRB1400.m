%% Function
function [ Qf ] = Inverse_Kinematics_IRB1400( Hi )
%% Sample Input
% Hi = [ 1 0 0 500; 0 1 0 100; 0 0 1 1500; 0 0 0 1];
% Qf=Inverse_Kinematics_IRB1400(Hi);
%% Initialization
Qv = sym('Q%d',[1 6]);      % Qi is taken as theta_i
H = eye(4);
Qf = zeros(6,1);

%% DH Parameters
DH = [  Qv(1)          475     150     pi/2     
        pi/2 + Qv(2)   0       600     0
        Qv(3)          0       120     pi/2
        0              720     0       0
        Qv(4)          0       0       -pi/2
        Qv(5)          0       0       pi/2
        Qv(6)          85      0       0   ];

%% Homogeneous Transformation Computation
for i=1:4
    
    A{i} = [ cos(DH(i,1))    -sin(DH(i,1))*cos(DH(i,4))  sin(DH(i,1))*sin(DH(i,4))     DH(i,3)*cos(DH(i,1))
             sin(DH(i,1))    cos(DH(i,1))*cos(DH(i,4))   -cos(DH(i,1))*sin(DH(i,4))    DH(i,3)*sin(DH(i,1))
             0               sin(DH(i,4))                cos(DH(i,4))                  DH(i,2)
             0               0                           0                             1                   ];
    H = H * A{i};
end

%% Solving theta_1,theta_2, theta_3
Wc = Hi(1:3,4) - DH(7,2)*Hi(1:3,1:3)*[0 0 1]';
Equations = H(1:3,4) == Wc ;
Q = solve(Equations,Qv(1:3),'Real',true);

%% Finding R_6 wrt Frame 3
R_30 = H(1:3,1:3);
R_30 = subs(R_30,Qv(1:3),[Q.Q1(1),Q.Q2(1),Q.Q3(1)]);
R_30 = real(double(R_30));
R_63 = R_30' * Hi(1:3,1:3);

%% Finding Euler Angles
Qf(4:6) = rotm2eul(R_63,'ZYZ');  
Qf(1) = real(double(Q.Q1(1)));
Qf(2) = real(double(Q.Q2(1)));
Qf(3) = real(double(Q.Q3(1)));

%% Display
% fprintf('\nThe inverse kinematics for the given position and orientation:\n');
% for i=1:6
%     fprintf('\nQ_%d = %0.3f rad (or) %0.3f deg',i,Qf(i),radtodeg(Qf(i)));
% end
end
