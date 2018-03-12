%% Initialization
function [] = Workspace_IRB1400()
N_step = 100;
P = zeros(101,3);

%% Extreme Positions(Constraints)
Position = zeros(5,1,3);
Position(1,:) = [ 0 60 60 ];
Position(2,:) = [ 0 -60 60 ];
Position(3,:) = [ 0 -60 -60 ];
Position(4,:) = [ 0 60 -60 ];
Position(5,:) = [ 0 60 60 ];

%% Workspace Plot
figure('Name','Workspace');
axes;
title({'ABB IRB1400 Robot','Workspace 2D side view'});
xlabel('X-axis');
ylabel('Z-axis');
grid on;
box on;
hold on;
for i=1:4
    Q_new = (Position(i,:));
    Q_step = ((Position(i+1,:)-Position(i,:))/N_step);
    for j=1:N_step+1
        [P(j,:)] = Forward_Kinematics_IRB1400(Q_new);
        Q_new = Q_new + Q_step;
    end
    plot(P(:,1),P(:,3))
    axis equal;
end
end