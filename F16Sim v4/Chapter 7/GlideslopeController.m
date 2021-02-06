%% Reduced matrices and simulation

load Trim5000_300


% A_red_lo = sel(A_longitude_lo,1:5,1:5);
% B_red_lo = sel(A_longitude_lo,1:5,[6,7]);
% C_red_lo = sel(C_longitude_lo,1:5,1:5);
% D_red_lo = sel(C_longitude_lo,1:5,[6,7]);

A_red_lo = sel(A_longitude_lo, [1 3 4 2 5], [1 3 4 2 5]);
B_red_lo = sel(A_longitude_lo, [1 3 4 2 5], [6 7]);
C_red_lo = sel(C_longitude_lo, [1 3 4 2 5], [1 3 4 2 5]);
D_red_lo = sel(C_longitude_lo, [1 3 4 2 5], [6 7]);

sys_lo=ss(A_red_lo,B_red_lo,C_red_lo,D_red_lo);


Kq = -1
Ktheta = 2
Kv = 500
Kglide = 40








sim GlideslopeController_model
%% Figures

Altitude_plot = figure(1);
hold on;
plot(ans.Altitude{1}.Values(:))
title('Altitude')
xlabel('Time [s]');
ylabel('Altitude [ft]');
grid on

% 
% Velocity_plot = figure(2);
% hold on;
% plot(ans.Velocity{1}.Values(:))
% title('Velocity')
% xlabel('Time [s]');
% ylabel('Velocity [ft]');
% grid on
% 
% Alpha_plot = figure(3);
% hold on;
% plot(ans.Alpha{1}.Values(:))
% title('Alpha')
% xlabel('Time [s]');
% ylabel('Alpha [ft]');
% grid on
% 
% Theta_plot = figure(4);
% hold on;
% plot(ans.Theta{1}.Values(:))
% title('Theta')
% xlabel('Time [s]');
% ylabel('Theta [ft]');
% grid on
% 
% q_plot = figure(5);
% hold on;
% plot(ans.q{1}.Values(:))
% title('q')
% xlabel('Time [s]');
% ylabel('q [ft]');
% grid on
% 
% gamma_plot = figure(6);
% hold on;
% plot(ans.gamma{1}.Values(:))
% title('gamma + 3')
% xlabel('Time [s]');
% ylabel('gamma [ft]');
% grid on
% 
% 
% Gamma_plot = figure(7);
% hold on;
% plot(ans.Gamma{1}.Values(:))
% title('Gamma')
% xlabel('Time [s]');
% ylabel('Gamma [ft]');
% grid on
% 
% 
% 











