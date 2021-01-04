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
        self.kp = 3.806;
        self.kd = 10.76;  
        self.Ts =0.01; 
        self.limit = 2.0;
        self.k=2.943;
        end
        
        function force = update(self, z_r, x)
            z=x(1);
            zdot=x(2);
            % feedback linearized force
            force_f1=self.k*z;
            %equilibrium force around z_e = 0
            z_e=0.0;
            force_e=self.k*z_e;
            % compute the linearized force using PD control
            force_tilde=self.kp*(z_r-z)-self.kd*zdot;
            %compute total force
            force=force_f1+force_tilde;
            force = self.saturate(force);
        end
        
        
        
        
        
        function out = saturate(self, u)
            if abs(u) > self.limit
                out = self.limit * sign(u);
            else
              out = u;
            end
        end
    end
end

        
