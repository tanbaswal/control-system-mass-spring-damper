
classdef systemAnimation
    properties
       %prop
       box_handle
       spring_handle 
       damper_handle
       width
       height
    end
    methods
        %------constructor-----------
        function self = systemAnimation()
            self.width=2.0;
            self.height=2.0;
            figure(1), clf
            % draw ground track
            plot([-2*3, 2*3],[0,0],'k');
            hold on
            % initialize the box, spring, and damper
            self=self.draw_box(0); 
            self=self.draw_spring(0);
            self=self.draw_damper(0);
            % Set the x,y axis limits
            axis([-3*3, 3*3, -0.1, 3*3]);
            xlabel('z'); % label x-axis
            xline(-2*3);
        end
        
        function self=update(self, state)
            % external method to update the animation
            z= state(1);   % Horizontal position of box, m     
            self=self.draw_box(z); 
            self=self.draw_spring(z);
            self=self.draw_damper(z); 
            drawnow
        end
        
        function self=draw_box(self, z)
            % specify X-Y locations of corners of the cart
            X = [z-self.width/2, z+self.width/2, z+self.width/2, z-self.width/2];
            Y = [0,0,self.height,self.height];
            % plot or update cart
            if isempty(self.box_handle)
                self.box_handle = fill(X,Y,'r');
            else
                set(self.box_handle,'XData',X);
            end   
        end
        
        function self=draw_damper(self, z)
            % specify X-Y locations of ends of spring
            X = [z-(self.width/2), -2*3];
            Y=  [0.2,0.2];
            % plot or update rod
            if isempty(self.damper_handle)
                self.damper_handle = plot(X, Y, 'g');
            else
                set(self.damper_handle,'XData', X, 'YData', Y);
            end
        end
        
            
        function self=draw_spring(self, z)
            % specify X-Y locations of ends of spring
            X = [z-(self.width/2), -2*3];
            Y=  [1.8,1.8];
            % plot or update rod
            if isempty(self.spring_handle)
                self.spring_handle = plot(X, Y, 'k');
            else
                set(self.spring_handle,'XData', X, 'YData', Y);
            end
        end
    end
end

        
            
        
        
       