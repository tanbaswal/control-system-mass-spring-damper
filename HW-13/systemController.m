classdef systemController < handle
    properties
        m
        b
        k
        x_hat
        force_d1
        K
        ki
        limit
        Ts
        integrator
        error_d1
        L
        A
        B
        C

    end
    methods
        function self = systemController(P)
         
           % plant parameters known to controller
           self.m = P.m;
           self.b = P.b;
           self.k = P.k;
           self.x_hat = [0.0; 0.0];
           self.force_d1 = 0.0;
           self.K = P.K;
           self.ki = P.ki;
           self.limit = P.force_max;
           self.Ts = P.Ts;
           self.integrator = 0.0;
           self.error_d1 = 0.0;
           self.L  = P.L;
           self.A  = P.A;
           self.B  = P.B;
           self.C  = P.C;


        
       
       
       
        
        end
        
        function [force,x_hat] = update(self, z_r, y)
            % update the observer and extract z_hat
            x_hat = self.updateObserver(y);
            z_hat = x_hat(1);
             % integrate error
            error = z_r - z_hat;
            self.integrateError(error); 
            % feedback linearized force
            force_f1=self.k*z_hat;
            % compute the linearized force using PD control
            force_tilde=-self.K*x_hat - self.ki*self.integrator;
            %compute total force
            force=self.saturate(force_f1+force_tilde);
            self.force_d1 = force;
        end
        
         function xhat = updateObserver(self, y)
            % update observer using RK4 integration
            F1 = self.observer_f(self.x_hat, y);
            F2 = self.observer_f(self.x_hat + self.Ts/2*F1, y);
            F3 = self.observer_f(self.x_hat + self.Ts/2*F2, y);
            F4 = self.observer_f(self.x_hat + self.Ts*F3, y);
            self.x_hat = self.x_hat...
                + self.Ts/6 * (F1 + 2*F2 + 2*F3 + F4);
            xhat = self.x_hat;
         end
        
          function x_hat_dot = observer_f(self, x_hat, y)
             % compute feedback linearizing torque tau_fl
            z_hat = x_hat(1);
            x_e = [0; 0];
            y_e = 0;
            force_fl = self.k*z_hat;
            x_hat_dot = self.A * (x_hat-x_e)...
                        + self.B*(self.force_d1 - force_fl)...
                        + self.L*((y-y_e) - self.C * (x_hat-x_e));
        end
        
         function self = integrateError(self, error)
            self.integrator = self.integrator + (self.Ts/2.0)*(error+self.error_d1);
            self.error_d1 = error;
        end

        function out = saturate(self,u)
            if abs(u) > self.limit
                u = self.limit*sign(u);
            end
            out = u;
           
        end        

        
        
        
        
        
        
    end
end

        
