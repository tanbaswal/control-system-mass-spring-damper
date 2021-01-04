% Single Link Arm Parameter File
addpath ./..
clear 
% Simulation Parameters
P.t_start = 0.0;  % Start time of simulation
P.t_end = 50.0;   % End time of simulation
P.Ts = 0.01;      % sample time for simulation
P.t_plot = 0.1;   % the plotting and animation is updated at this rate


% Physical parameters of arm known to the controller
P.m = 4.493;  % kg
P.k = 2.943; % constant
P.b = 0.499; % N m s
v=-P.k/P.m;
n=-P.b/P.m;
p=1.0/P.m;

fprintf('\t v: %f\n', v)
fprintf('\t n: %f\n', n)
fprintf('\t p: %f\n', p)

% parameters for animation
P.height = 2.0;
P.width = 2.0;

% initial conditions
P.z0 = 0;     
P.zdot0 = 0; 

% control saturation limits
P.force_max = 6; % max torque, N-m

% dirty derivative parameters
P.sigma = 0.05; % cutoff freq for dirty derivative

%  tuning parameters
%tr = 1.25; % previous tuned parameter
tr = 0.37;  % tuned to get fastest possible rise time before saturation.
zeta = 0.707;
integrator_pole = -0.5;

% equalibrium angle and torque
P.z_e = 0*pi/180;
P.force_e = P.k*P.z_e;

% desired closed loop polynomial
wn = 2.2/tr;



% state space design 
A = [0 1; v n];
B = [0; p];
C = [1 0];
% form augmented system
A1 = [0 1 0; v n 0;-1 0 0];
B1 = [0; p; 0];

% gains for pole locations
des_char_poly = conv([1,2*zeta*wn,wn^2],poly(integrator_pole));
des_poles = roots(des_char_poly);

% is the system controllable?
if rank(ctrb(A1,B1))~=3 
    disp('System Not Controllable'); 
else
    K1   = place(A1,B1,des_poles); 
    P.K  = K1(1:2);
    P.ki = K1(3);

end






fprintf('\t K: [%f, %f]\n', P.K(1), P.K(2))
fprintf('\t ki: %f\n', P.ki)


