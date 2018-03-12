%% ABB IRB 1400 Robot
clc; clear; close all;
%% Plot Workspace
Workspace_IRB1400();

%% Foward Kinematics_IRB4100
P_i = [ 0 0 0 ];
Q_Forward = Forward_Kinematics_IRB1400(P_i);
fprintf('\nThe forward kinematic position for the given angles:');
display(Q_Forward);

%% Inverse Kinematics IRB1400
H_f = [ 1 0 0 500; 0 1 0 100; 0 0 1 1500; 0 0 0 1];
Q_Inverse = Inverse_Kinematics_IRB1400(H_f);
fprintf('\nThe inverse kinematics for the given positon and orientation is:');
display(Q_Inverse);

%% Jacobian Matrix
Q = [0.1974 0.4357 -0.0273];
State = [ 5 5 10 0 0 0 ];
[ J, Theta_dot ] = Jacobian(Q,State);
fprintf('\nThe jacobian for the given input is:');
display(J);
fprintf('\nThe computed joint velocities for the given input are:');
display(Theta_dot);

%% Simulation
P_f = [ 500 100 1415 ];
Simulation_IRB1400(P_f);