%% Function
function [] = Simulation_IRB1400( Hi )
%% Input
% Hi = [ 500 100 1415 ];
% Simulation_IRB1400(Hi);
%% Initialization
Qv = sym('Q%d',[1 3]);          % Qi is taken as theta_i
Qh = [ 0 0 0 ];
X_plot = zeros(1,5);
Y_plot = zeros(1,5);
Z_plot = zeros(1,5);
H = eye(4);
Qf = zeros(3,1);
N_step = 50;

%% DH Parameters
DH = [  Qv(1)          475     150     pi/2     
        pi/2 + Qv(2)   0       600     0
        Qv(3)          0       120     pi/2
        0              720     0       0  ];

%% Homogeneous Transformation Computation
for i=1:4   
    A{i} = [ cos(DH(i,1))    -sin(DH(i,1))*cos(DH(i,4))  sin(DH(i,1))*sin(DH(i,4))     DH(i,3)*cos(DH(i,1))
             sin(DH(i,1))    cos(DH(i,1))*cos(DH(i,4))   -cos(DH(i,1))*sin(DH(i,4))    DH(i,3)*sin(DH(i,1))
             0               sin(DH(i,4))                cos(DH(i,4))                  DH(i,2)
             0               0                           0                             1                   ];
    H = H * A{i};
end

%% Solving theta_1,theta_2, theta_3
Equations = H(1:3,4) == Hi' ;
Q = solve(Equations,Qv,'Real',true);
Qf(1) = real(double(Q.Q1(1)));
Qf(2) = real(double(Q.Q2(1)));
Qf(3) = real(double(Q.Q3(1)));
Qf = Qf.';

%% Home Position Computing & Plotting
HP = subs(DH,Qv,Qh);
H = eye(4);
for i=1:4    
    A{i} = [ cos(HP(i,1))    -sin(HP(i,1))*cos(HP(i,4))  sin(HP(i,1))*sin(HP(i,4))     HP(i,3)*cos(HP(i,1))
             sin(HP(i,1))    cos(HP(i,1))*cos(HP(i,4))   -cos(HP(i,1))*sin(HP(i,4))    HP(i,3)*sin(HP(i,1))
             0               sin(HP(i,4))                cos(HP(i,4))                  HP(i,2)
             0               0                           0                             1                   ];
      
    H = H * A{i};
    X_plot(i+1) = H(1,4);
    Y_plot(i+1) = H(2,4);
    Z_plot(i+1) = H(3,4); 
end

figure('Name','Simulation');
pl = plot3(X_plot,Y_plot,Z_plot,'-o','LineWidth',2,'MarkerFaceColor',[0 0 0]);
axis([ -200 1000 -100 100 0 1500 ]);
drawnow();
title('ABB IRB1400 Robot');
xlabel('X axis(mm)');
ylabel('Y axis(mm)');
zlabel('Z axis(mm)');

%% Simulation

Q_step = (Qf - Qh) / N_step;
Q_new = Q_step;
for j=1:N_step
    
    HC = subs(DH,Qv,Q_new); 
    H = eye(4);
    X_plot = zeros(1,5);
    Y_plot = zeros(1,5);
    Z_plot = zeros(1,5);
    
    for i=1:4

    A{i} = [ cos(HC(i,1))    -sin(HC(i,1))*cos(HC(i,4))   sin(HC(i,1))*sin(HC(i,4))    HC(i,3)*cos(HC(i,1))
             sin(HC(i,1))    cos(HC(i,1))*cos(HC(i,4))    -cos(HC(i,1))*sin(HC(i,4))   HC(i,3)*sin(HC(i,1))
             0               sin(HC(i,4))                 cos(HC(i,4))                 HC(i,2)
             0               0                            0                            1                 ];
      
    H = H * A{i};
    X_plot(i+1) = H(1,4);
    Y_plot(i+1) = H(2,4);
    Z_plot(i+1) = H(3,4);
    
    end
    
    set(pl,'xdata',X_plot,'ydata',Y_plot,'zdata',Z_plot,'Marker','o','LineWidth',2,'MarkerFaceColor',[0 0 0]); 
    drawnow();
    Q_new = Q_new + Q_step;

end
end