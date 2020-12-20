function refTraj = generateReference(name,dt)

if strcmp(name,'rising_spiral')
    N_traj = 200;
    xref = 2*cos(0.02*(0:N_traj-1));
    yref = 2*sin(0.02*(0:N_traj-1));
    zref = 0.02*(0:N_traj-1);

    dxref = [0, diff(xref)/dt];
    dyref = [0, diff(yref)/dt];
    dzref = [0, diff(zref)/dt]; 
    
    ddxref = [0, diff(dxref)/dt];
    ddyref = [0, diff(dyref)/dt];
    ddzref = [0, diff(dzref)/dt];
        
elseif strcmp(name,'straight')
    xref = [(0:0.08:4), (4:-0.08:0)];
    yref = zeros(size(xref));
    zref = zeros(size(xref));    

    dxref = [0, diff(xref)/dt];
    dyref = [0, diff(yref)/dt];
    dzref = [0, diff(zref)/dt]; 
    
    ddxref = [0, diff(dxref)/dt];
    ddyref = [0, diff(dyref)/dt];
    ddzref = [0, diff(dzref)/dt];

elseif strcmp(name,'sawtooth')
   
    xref = 0:dt:6;
    yref = sawtooth(2*xref,0.5);
    zref = zeros(size(xref));    

    dxref = [0, diff(xref)/dt];
    dyref = [0, diff(yref)/dt];
    dzref = [0, diff(zref)/dt]; 
    
    ddxref = [0, diff(dxref)/dt];
    ddyref = [0, diff(dyref)/dt];
    ddzref = [0, diff(dzref)/dt];

elseif strcmp(name,'sawtooth_vertical')
   
    xref = 0:dt:6;
    yref = zeros(size(xref));
    zref = sawtooth(2*xref,0.5);

    dxref = [0, diff(xref)/dt];
    dyref = [0, diff(yref)/dt];
    dzref = [0, diff(zref)/dt]; 
    
    ddxref = [0, diff(dxref)/dt];
    ddyref = [0, diff(dyref)/dt];
    ddzref = [0, diff(dzref)/dt];
end
    rollref = -ddyref/9.81;
    pitchref = ddxref/9.81;
    refTraj = [xref;yref;zref;dxref;dyref;dzref;rollref;pitchref];
end