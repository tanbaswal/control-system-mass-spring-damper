% Single Link Arm Parameter File
addpath ./..
clear 
% Simulation Parameters
P.t_start = 0.0;  % Start time of simulation
P.t_end = 20.0;   % End time of simulation
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
wn_obs=10; %natural frequency of observer
zeta = 0.707;
integrator_pole = -0.5;
zeta_obs=1.707;    % damping ratio of observer
dist_pole = -5.5;  % disturbance observer pole

% equalibrium angle and torque
P.z_e = 0*pi/180;
P.force_e = P.k*P.z_e;

% desired closed loop polynomial
wn = 2.2/tr;



% state space design 
P.A = [0 1; v n];
P.B = [0; p];
P.C = [1 0];
% form augmented system
A1 = [0 1 0; v n 0;-1 0 0];
P.B1 = [0; p; 0];

% gains for pole locations
des_char_poly = conv([1,2*zeta*wn,wn^2],poly(integrator_pole));
des_poles = roots(des_char_poly);

% is the system controllable?
if rank(ctrb(A1,P.B1))~=3 
    disp('System Not Controllable'); 
else
    K1   = place(A1,P.B1,des_poles); 
    P.K  = K1(1:2);
    P.ki = K1(3);

end

% observer design
% form augmented system for disturbance observer
P.A2 = [P.A, P.B; zeros(1,2), 0];
P.C2 = [P.C, 0];

% desired observer poles
des_obsv_char_poly = conv([1,2*zeta*wn,wn^2],...
                          poly(dist_pole));
des_obsv_poles = roots(des_obsv_char_poly);     %des_poles(0:n).*2;

% is the system observable?
if rank(obsv(P.A2,P.C2))~=3
    disp('System Not Observable'); 
else % if so, compute gains
    P.L2 = place(P.A2',P.C2',des_obsv_poles)'; 
end





fprintf('\t K: [%f, %f]\n', P.K(1), P.K(2))
fprintf('\t ki: %f\n', P.ki)
fprintf('\t L^T: [%f, %f, %f]\n', P.L2(1), P.L2(2), P.L2(3))

