function refTraj = generateReference(name,dt)

if strcmp(name,'rising_spiral')
    N_traj = 200;
    xref = 5*cos(0.02*(0:N_traj-1));
    yref = 5*sin(0.02*(0:N_traj-1));
    zref = 0.02*(0:N_traj-1);
    rollref = zeros(size(zref));
    pitchref = zeros(size(zref));

    dxref = [0, diff(xref)/dt];
    dyref = [0, diff(yref)/dt];
    dzref = [0, diff(zref)/dt]; 

    refTraj = [xref;yref;zref;dxref;dyref;dzref;rollref;pitchref];    
elseif strcmp(name,'straight')
    xref = [(0:0.1:5), (5:-0.1:0)];
    yref = zeros(size(xref));
    zref = zeros(size(xref));
    rollref = zeros(size(zref));
    pitchref = zeros(size(zref));

    dxref = [0, diff(xref)/dt];
    dyref = [0, diff(yref)/dt];
    dzref = [0, diff(zref)/dt]; 

    refTraj = [xref;yref;zref;dxref;dyref;dzref;rollref;pitchref];
end