classdef systemController
    properties
        m
        b
        k
        zCtrl
    end
    methods
        function self = systemController(P)
         % Instantiates the SS_ctrl object
           self.zCtrl=PIDControl(P.kp,P.ki,P.kd,P.force_max,P.sigma,P.Ts);
           % plant parameters known to controller
           self.m = P.m;
           self.b = P.b;
           self.k = P.k;

        
       
       
       
        
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

        
