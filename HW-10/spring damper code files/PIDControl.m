classdef PIDControl < handle
    properties
        kp
        ki
        kd
        limit
        beta
        Ts
        init_flag
        z_dot
        z_d1
        error_dot
        error_d1
        integrator
    end
    methods
        function self = PIDControl(kp, ki, kd, limit, sigma, Ts)
            self.kp = kp;  % Proportional control gain
            self.ki = ki;   % Integral control gain
            self.kd = kd;  % Derivative control gain
            self.limit = limit;  % output saturation limit
            % dirty derivative gain
            self.beta = (2*sigma-Ts)/(2*sigma+Ts); 
            self.Ts = Ts;
            self.init_flag = 1;  % true for first time step
            self.z_dot = 0.0;  % estimated derivative of y
            self.z_d1 = 0.0;  % Signal y delayed by one sample
            self.error_dot = 0.0;  % estimated derivative of error
            self.error_d1 = 0.0;  % Error delayed by one sample
            self.integrator = 0.0;  % value of the integrator
        end
         %----------------------------
        function u_sat = PID(self, z_r, z, flag)
            %    PID control,
            %    
            %    if flag==True, then returns
            %      u = kp*error+ki*integral(error)+kd*error_dot.
            %    else returns 
            %      u = kp*error+ki*integral(error)-kd*y_dot.
            %    
            %    error_dot and y_dot are computed numerically using 
            %      a dirty derivative the integral is computed 
            %      numerically

            % Compute the current error
            error = z_r - z;
            % integrate error
            self.integrator = self.integrator + (self.Ts/2)*(error+self.error_d1);
            % initialize differentiators.
            if self.init_flag==1
                self.z_d1 = z;
                self.error_d1 = error;
                self.init_flag=0;
            end
            % PID Control
            if flag==true
                % differentiate error
                self.error_dot = self.beta*self.error_dot + (1-self.beta)/self.Ts*(error - self.error_d1);
                % PID control
                u_unsat = self.kp*error + self.ki*self.integrator + self.kd*self.error_dot;
            else
                % differentiate y
                self.z_dot = self.beta*self.z_dot + (1-self.beta)*((z-self.z_d1)/self.Ts);
                % PID control
                u_unsat = self.kp*error + self.ki*self.integrator - self.kd*self.z_dot;
            end
            % return saturated control signal
            u_sat = self.saturate(u_unsat);
            % integrator anti-windup
            if self.ki~=0
                self.integrator = self.integrator + self.Ts/self.ki*(u_sat-u_unsat);
            end
            % update delayed variables
            self.error_d1 = error;
            self.z_d1 = z;
        end
        function u_sat = PD(self, z_r, z, flag)
            %    PD control,
            %    
            %    if flag==True, then returns
            %        u = kp*error + kd*error_dot.
            %    else returns 
            %        u = kp*error - kd*y_dot.
            %    
            %    error_dot and y_dot are computed numerically using 
            %      a dirty derivative

            % Compute the current error
            error = z_r - z;
            % initialize differentiators.
            if self.init_flag==1
                self.z_d1 = y;
                self.error_d1 = error;
                self.init_flag=0;
            end
            if flag==true
                % differentiate error
                self.error_dot = self.beta*self.error_dot + (1-self.beta)/self.Ts*(error - self.error_d1);
                % PD control
                u_unsat = self.kp*error + self.kd*self.error_dot;
            else
                % differentiate y
                self.z_dot = self.beta*self.z_dot + (1-self.beta)*((z-self.z_d1)/self.Ts);
                % PD control                
                u_unsat = self.kp*error - self.kd*self.z_dot;
            end
            % return saturated control signal
            u_sat = self.saturate(u_unsat);
            % update delayed variables
            self.error_d1 = error;
            self.z_d1 = z;
        end
        function out = saturate(self,u)
            if abs(u) > self.limit
                u = self.limit*sign(u);
            end
            out = u;
        end
    end
end







