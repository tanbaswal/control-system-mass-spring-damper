param13;


% instantiate system, controller, and reference classes
alpha=0.0;
system = systemDynamics(alpha,P);
controller = systemController(P); 
reference = signalGenerator(57*pi/180, 0.05);
disturbance = signalGenerator(0.25,0.0); 
noise = signalGenerator(0.01);

% instantiate the data plots and animation
dataPlot = dataPlotter(P); 
animation = systemAnimation(P);
dataPlotObserver = dataPlotterObserver(P);

% main simulation loop
t = P.t_start;      % time starts at t_start  
y = system.h();   % system output at start of simulation
while t < P.t_end
    % set time for next plot
    t_next_plot = t + P.t_plot;
    % updates control and dynamics at faster simulation rate
    while t < t_next_plot
        r = reference.square(t);      % assign reference  
        d = 0.0;  %disturbance.step(t);       % simulate input disturbance
        n = 0.0;          % simulate noise
        
        [u,xhat] = controller.update(r,y+n);    %update controller
        y = system.update(u+d);       % Propagate the dynamics
        t = t + P.Ts;                   % advance time by Ts
    end
    % update animation and data plots
    animation.update(system.state);
    dataPlot.update(t, r, system.state,u);
    dataPlotObserver.update(t, system.state, xhat, d, 0);
end    