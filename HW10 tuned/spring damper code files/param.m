% Single Link Arm Parameter File

clear 

% Physical parameters of arm known to the controller
P.m = 4.493;  % kg
P.k = 2.943; % constant
P.b = 0.499; % N m s


% parameters for animation
P.height = 2.0;
P.width = 2.0;

% initial conditions
P.z0 = 0;     
P.zdot0 = 0; 

%  tuning parameters
%tr = 0.8; % part (a)
tr = 1.25;  % tuned to get fastest possible rise time before saturation.
zeta = 0.707;
P.ki = 0.1;  % integrator gain

% equalibrium angle and torque
P.z_e = 0*pi/180;
P.force_e = P.k*P.z_e;

% desired closed loop polynomial
wn = 2.2/tr;
Delta_cl_d = [1, 2*zeta*wn, wn^2];

% compute PD gains
P.kp = (Delta_cl_d(3)*P.m)-P.k;
P.kd = (Delta_cl_d(2)*P.m)-P.b;


% Simulation Parameters
P.t_start = 0.0;  % Start time of simulation
P.t_end = 50.0;   % End time of simulation
P.Ts = 0.01;      % sample time for simulation
P.t_plot = 0.1;   % the plotting and animation is updated at this rate

% dirty derivative parameters
P.sigma = 0.05; % cutoff freq for dirty derivative

% control saturation limits
P.force_max = 6; % max torque, N-m

fprintf('\t kp: %f\n', P.kp)
fprintf('\t ki: %f\n', P.ki)
fprintf('\t kd: %f\n', P.kd)


