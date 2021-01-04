classdef systemController < handle
    properties
        m
        b
        k
        K
        ki
        limit
        Ts
        integrator
        error_d1

    end
    methods
        function self = systemController(P)
         
           % plant parameters known to controller
           self.m = P.m;
           self.b = P.b;
           self.k = P.k;
           self.K = P.K;
           self.ki = P.ki;
           self.limit = P.force_max;
           self.Ts = P.Ts;
           self.integrator = 0.0;
           self.error_d1 = 0.0;


        
       
       
       
        
        end
        
        function force = update(self, z_r, y)
            z=y(1);
             % integrate error
            error = z_r - z;
            self.integrateError(error); 
            % feedback linearized force
            force_f1=self.k*z;
            % compute the linearized force using PD control
            force_tilde=-self.K*y - self.ki*self.integrator;
            %compute total force
            force=self.saturate(force_f1+force_tilde);
    
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

        
