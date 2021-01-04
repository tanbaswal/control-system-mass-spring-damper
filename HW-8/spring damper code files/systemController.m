classdef systemController
    properties
        kp 
        kd 
        ki
        Ts 
        k
        limit
    end
    methods
        function self = systemController()
        % specify gains
        self.kp = 8.13;
        self.kd = 9.5;  
        self.Ts =0.01; 
        self.limit = 6.0;
        self.k=2.943;
        end
        
        function force = update(self, z_r, x)
            z=x(1);
            zdot=x(2);
            % feedback linearized force
            force_f1=self.k*z;
            % compute the linearized force using PD control
            force_tilde=self.kp*(z_r-z)-self.kd*zdot;
            %compute total force
            force=force_f1+force_tilde;
            force = self.saturate(force,self.limit);
        end
        
        
        
        
        
        function out = saturate(self, u,force_max)
            if abs(u) > force_max
                out = force_max * sign(u);
            else
              out = u;
            end
        end
    end
end

        
