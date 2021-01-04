classdef systemDynamics < handle
    properties
      state
      Ts
      limit
      m
      k
      b
    end
    methods
        %---constructor-------------------------
        function self = systemDynamics()
            y0 = 0;    % initial condition for y
            ydot0 = 0;  % initial condition for ydot
            self.state=[y0;ydot0;];
            self.Ts = 0.01;
            self.limit = 1.0;   % saturation limit
            % system parameters
            self.m = 4.493; 
            self.k = 2.943;
            self.b = 0.499;
            % modify the system parameters by random value
            alpha = 0.2;        % Uncertainty parameter
            self.m = self.m * (1+2*alpha*rand-alpha);
            self.k = self.k * (1+2*alpha*rand-alpha); 
            self.b = self.b * (1+2*alpha*rand-alpha);
        end
        
        function xdot = f(self, state,u)
            % Return xdot = f(x,u)
            y = state(1); 
            ydot = state(2);
            % The equations of motion.
            yddot = -(self.b/self.m) * ydot - (self.k/self.m) * y + (1/self.m) * u;
            % build xdot and return
            xdot = [ydot; yddot];
        end 
        
        function y = h(self) 
            % y = h(x)
            y = self.state(1); 
        end
        
        function y = update(self, u)
            % This is the external method that takes input u
            % and returns output y.
            u = self.saturate(u, self.limit);   % saturate input
            self.rk4_step(u);                   % propagate state by one time step
            y = self.h();                       % compute the output at current time
        end
        
        function self = rk4_step(self, u)
            % Integrate ODE using Runge-Kutta-4 algorithm
            F1 = self.f(self.state, u);
            F2 = self.f(self.state + self.Ts/2*F1, u); 
            F3 = self.f(self.state + self.Ts/2*F2, u);
            F4 = self.f(self.state + self.Ts*F3, u);
            self.state = self.state + self.Ts/6 * (F1 + 2*F2 + 2*F3 + F4);
        end
        
        function out = saturate(~, in, limit)
            if abs(in) > limit
                out = limit * sign(in);
            else
                out = in;
            end
        end
    end
end



        
        
        