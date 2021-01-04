classdef systemDynamics < handle
    properties
      state
      output
      Ts
      limit
      m
      k
      b
    end
    methods
        %---constructor-------------------------
        function self = systemDynamics(alpha,P)
           
            self.state=[P.z0;P.zdot0;];
            self.Ts = P.Ts;
            self.limit = P.force_max;   % saturation limit
            % system parameters
            self.m = P.m; 
            self.k = P.k;
            self.b = P.b;
            % modify the system parameters by random value
                
            self.m = self.m * (1+2*alpha*rand-alpha);
            self.k = self.k * (1+2*alpha*rand-alpha); 
            self.b = self.b * (1+2*alpha*rand-alpha);
        end
        
        function xdot = f(self, state,u)
            % Return xdot = f(x,u)
            z = state(1); 
            zdot = state(2);
            % The equations of motion.
            zddot = -(self.b/self.m) * zdot - (self.k/self.m) * z + (1/self.m) * u;
            % build xdot and return
            xdot = [zdot; zddot];
        end 
        
        function y = h(self) 
            % y = h(x)
            z = self.state(1);
            y=z;
        end
        
        function y = update(self, u)
            % This is the external method that takes input u
            % and returns output y.
            u = self.saturate(u, self.limit);   % saturate input
            self.rk4_step(u);                   % propagate state by one time step
            y = self.h();
            self.output=y;   % compute the output at current time
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



        
        
        