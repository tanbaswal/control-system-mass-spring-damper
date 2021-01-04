classdef dataPlotter < handle
    properties
        
        % data histories
        time_history
        r_history 
        y_history
        ydot_history
        u_history
        index
        % figure handles
        r_handle 
        y_handle
        ydot_handle 
        u_handle
    end
    methods
        %--constructor--------------------------
        function self = dataPlotter(P)
           % Instantiate lists to hold the time and data histories
           self.time_history=NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
           self.r_history=NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
           self.y_history=NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
           self.ydot_history=NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
           self.u_history= NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
           self.index=1;
           % Create figure and axes handles
           figure(2), clf
           subplot(311)
                hold on
                self.r_handle=plot(self.time_history,self.r_history,'r');
                self.y_handle=plot(self.time_history,self.y_history,'b');
                ylabel('z')
                title('SYSTEM DATA')
           subplot(312)
                hold on
                self.ydot_handle = plot(self.time_history,self.ydot_history, 'b');
                ylabel('zdot(velocity)')
           subplot(313)
                hold on
                self.u_handle=plot(self.time_history,self.u_history, 'b');    
                ylabel('FORCE(input)')
                 
        end
        
        function self=update(self, time, reference, states,control)
            % update the time history of all plot variables
            self.time_history(self.index) = time; 
            self.r_history(self.index) = reference; 
            self.y_history(self.index) = states(1);
            self.ydot_history(self.index) = states(2);
            self.u_history(self.index) = control;
            self.index = self.index + 1;
            % update the plots with associated histories
            set(self.r_handle,'Xdata', self.time_history,'Ydata', self.r_history)
            set(self.y_handle,'Xdata', self.time_history,'Ydata', self.y_history)
            set(self.ydot_handle,'Xdata', self.time_history,'Ydata', self.ydot_history)
            set(self.u_handle,'Xdata', self.time_history, 'Ydata', self.u_history)
        end
    end
end

                
                
           