classdef systemController
    properties
        kp 
        kd 
        ki
        Ts 
        k
        limit
        sigma
        zCtrl
    end
    methods
        function self = systemController()
        % specify gains
        self.kp = 2.50;
        self.kd = 6.436;  
        self.Ts =0.01; 
        self.limit = 6.0;
        self.k=2.943;
        self.sigma=0.05;
        self.ki=0.1;
        self.zCtrl=PIDControl(self.kp,self.ki,self.kd,self.limit,self.sigma,self.Ts);
        end
        
        function force = update(self, z_r, y)
            z=y(1);
            
            % feedback linearized force
            force_f1=self.k*z;
            % compute the linearized force using PD control
            force_tilde=self.zCtrl.PID(z_r,z,false);
            %compute total force
            force=force_f1+force_tilde;
            
        end
        
        
        
        
        
        
        
    end
end

        
