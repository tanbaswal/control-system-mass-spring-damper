classdef dataPlotterObserver < handle
    properties
        % data histories
        time_history
        z_history
        z_hat_history
        z_dot_history
        z_hat_dot_history
        d_history
        d_hat_history
        index
        % figure handles
        z_handle
        z_hat_handle
        z_dot_handle
        z_hat_dot_handle
        d_handle
        d_hat_handle
    end
    methods
        %--constructor--------------------------
        function self = dataPlotterObserver(P)
            % Instantiate lists to hold the time and data histories
            self.time_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.z_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.z_hat_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.z_dot_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.z_hat_dot_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.d_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.d_hat_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.index = 1;

            % Create figure and axes handles
            figure(3), clf
            subplot(3, 1, 1)
                hold on
                self.z_handle = plot(self.time_history, self.z_history, 'b');
                self.z_hat_handle = plot(self.time_history, self.z_hat_history, 'g');
                ylabel('Z(m)')
                title('SMD State and Observed State')
            subplot(3, 1, 2)
                hold on
                self.z_dot_handle = plot(self.time_history, self.z_dot_history, 'b');
                self.z_hat_dot_handle = plot(self.time_history, self.z_hat_dot_history, 'g');
                ylabel('Zdot(m/s)')
            subplot(3, 1, 3)
                hold on
                self.d_handle = plot(self.time_history, self.d_history, 'b');
                self.d_hat_handle = plot(self.time_history, self.d_hat_history, 'g');
                ylabel('d')
        end
        
        function self = update(self, t, x, xhat, d, dhat)
            % update the time history of all plot variables
            self.time_history(self.index) = t;
            self.z_history(self.index) = x(1);
            self.z_dot_history(self.index) = x(2);
            self.z_hat_history(self.index) = xhat(1);
            self.z_hat_dot_history(self.index) = xhat(2);
            self.d_history(self.index) = d;
            self.d_hat_history(self.index) = dhat;
            self.index = self.index + 1;
            % update the plots with associated histories
            set(self.z_handle, 'Xdata', self.time_history, 'Ydata', self.z_history)
            set(self.z_dot_handle, 'Xdata', self.time_history, 'Ydata', self.z_dot_history)
            set(self.z_hat_handle, 'Xdata', self.time_history, 'Ydata', self.z_hat_history)
            set(self.z_hat_dot_handle, 'Xdata', self.time_history, 'Ydata', self.z_hat_dot_history)
            set(self.d_handle, 'Xdata', self.time_history, 'Ydata', self.d_history)
            set(self.d_hat_handle, 'Xdata', self.time_history, 'Ydata', self.d_hat_history)
        end
    end
end
