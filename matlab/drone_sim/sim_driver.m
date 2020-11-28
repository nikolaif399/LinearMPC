
N_traj = 200;
N = 20;
dt = 0.05;

% Reference Trajectory Generation
xref = cos(0.02*(0:N_traj-1));
yref = sin(0.02*(0:N_traj-1));
zref = 0.1*(0:N_traj-1);

dxref = [0, diff(xref)/dt];
dyref = [0, diff(yref)/dt];
dzref = [0, diff(zref)/dt]; 

refTraj = [xref;yref;zref;dxref;dyref;dzref];
xCur = refTraj(:,1);

nx = size(refTraj,1);

% Simulate
step = 1;
figure
while(step + N < N_traj)
    tic
    
    % Get ref trajectory for next N steps
    mpcRef = refTraj(:,step:step+N-1);
    
    % Collect MPC Control (roll,pitch,yaw,thrust commands)
    %u = ...
    
    % Simulate nonlinear drone dynamic model
    %xCur = ...
        
    % Update plots and sleep for real time animation
    xCur = mpcRef(1:3,1);
    plot3(mpcRef(1,:),mpcRef(2,:),mpcRef(3,:),'b')
    hold on
    plot3(xCur(1,:),xCur(2,:),xCur(3,:),'r*')
    hold off
    title('Drone Trajectory')
    
    xlim([min(xref) max(xref)])
    ylim([min(yref) max(yref)])
    zlim([min(zref) max(zref)])
    grid on
    
    step = step + 1;
    tsleep = dt - toc;
    if (tsleep > 0)
        pause(tsleep)
    end
    
end




