function [ J, theta_dot ] = Jacobian(Pos, State)
%% Initialization
syms Q1 Q2 Q3;
H = eye(4);

%% DH Parameters
DH = [  Q1          475     150     pi/2
        pi/2 + Q2   0       600     0
        Q3          0       120     pi/2
        0           720     0       0    ];

%% Homogeneous Transformation
for i=1:4
    A{i} = [ cos(DH(i,1))    -sin(DH(i,1))*cos(DH(i,4))  sin(DH(i,1))*sin(DH(i,4))     DH(i,3)*cos(DH(i,1))
             sin(DH(i,1))    cos(DH(i,1))*cos(DH(i,4))   -cos(DH(i,1))*sin(DH(i,4))    DH(i,3)*sin(DH(i,1))
             0               sin(DH(i,4))                cos(DH(i,4))                  DH(i,2)
             0               0                           0                             1                   ];
    if(i~=3)
        Jw{:,i} = H(1:3,1:3) * [0; 0; 1] ;      % For Jacobian
    end
    H = H * A{i};
end

%% Jacobian
Jw(:,3)=[];
Jv = jacobian([H(1,4),H(2,4),H(3,4)],[Q1,Q2,Q3]);
J = [ Jv ; Jw ];

% % To print the jacobian matrix:
% fprintf('\n J = \n\n'); 
% disp(J);

%% Solution
J = subs(J,[Q1 Q2 Q3],Pos);
J = double(J);
Jinv = pinv(J);
theta_dot = Jinv*State';

%% Display
% fprintf('\n J = \n');
% disp(J);
% fprintf('\n Theta_dot = \n\n');
% disp(theta_dot);
end