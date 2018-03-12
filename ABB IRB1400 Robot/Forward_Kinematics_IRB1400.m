function [ Pos ] = Forward_Kinematics_IRB1400 ( Q )
%% Initialization
Q = degtorad(Q);
H = eye(4);

%% DH Parameters
DH = [  Q(1)          475     150     pi/2
        Q(2)+pi/2     0       600     0
        Q(3)          0       120     pi/2
        0             720     0       0     ];
    
%% Homogeneous Matrix
for i=1:4
    
    A{i} = [ cos(DH(i,1))    -sin(DH(i,1))*cos(DH(i,4))  sin(DH(i,1))*sin(DH(i,4))     DH(i,3)*cos(DH(i,1))
             sin(DH(i,1))    cos(DH(i,1))*cos(DH(i,4))   -cos(DH(i,1))*sin(DH(i,4))    DH(i,3)*sin(DH(i,1))
             0               sin(DH(i,4))                cos(DH(i,4))                  DH(i,2)
             0               0                           0                             1                   ];
    
H = round(H*A{i},4);
end
Pos = H(1:3,4);
end