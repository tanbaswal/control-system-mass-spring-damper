classdef systemController
    properties
        m
        b
        k
        K
        kr
        limit
        Ts
    end
    methods
        function self = systemController(P)
         
           % plant parameters known to controller
           self.m = P.m;
           self.b = P.b;
           self.k = P.k;
           self.K = P.K;
           self.kr = P.kr;
           self.limit = P.force_max;
           self.Ts = P.Ts;


        
       
       
       
        
        end
        
        function force = update(self, z_r, y)
            z=y(1);
            
            % feedback linearized force
            force_f1=self.k*z;
            % compute the linearized force using PD control
            force_tilde=-self.K*y+self.kr*z_r;
            %compute total force
            force=self.saturate(force_f1+force_tilde);
            
        end
        
        function out = saturate(self,u)
            if abs(u) > self.limit
                u = self.limit*sign(u);
            end
            out = u;
        end        

        
        
        
        
        
        
    end
end

        
