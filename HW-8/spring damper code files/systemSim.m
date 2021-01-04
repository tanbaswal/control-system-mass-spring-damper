t_start = 0;    % start time
t_end = 100;     % end time
t_plot = 0.1;   % sample rate for plotter and animation
Ts =0.01;       % sample rate for dynamics and controller

% instantiate system, controller, and reference classes
system = systemDynamics();
controller = systemController(); 
reference = signalGenerator(0.5, 0.05);
disturbance = signalGenerator(0.0); 


% instantiate the data plots and animation
dataPlot = dataPlotter(); 
animation = systemAnimation();

% main simulation loop
t = t_start;      % time starts at t_start  
y = system.h();   % system output at start of simulation
while t < t_end
    % set time for next plot
    t_next_plot = t + t_plot;
    % updates control and dynamics at faster simulation rate
    while t < t_next_plot
        r = reference.square(t);      % assign reference  
        d = disturbance.step(t);      % simulate input disturbance
        n = 0.0; %noise.random(t);          % simulate noise
        x=system.state;
        u = controller.update(r,x);    %update controller
        y = system.update(u+d);       % Propagate the dynamics
        t = t + Ts;                   % advance time by Ts
    end
    % update animation and data plots
    animation.update(system.state);
    dataPlot.update(t, r, system.state,u);
end    