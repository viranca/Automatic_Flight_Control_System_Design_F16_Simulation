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

%Kflare = 0.00005            % Kflare = 0.00005, Kglide = 40, Kv = 500, Ktheta = 2, Kq = -1
Kflare = 0.00045  

% Flare height calculation
x_1 = 1100;                          %[ft]    % Distance intended touchdown point to glideslope transmitter
V_0 = trim_state_lo(7);              %[ft/s]  % Airspeed
V_descent_t0 = V_0 * sin(-3*pi/180); %[ft/s]  % Descent rate at start of the manoeuvre
tau = x_1/(4*V_0+(V_descent_t0/tan(3*pi/180)));%[s] flare geometry time variable 
hflare = -V_descent_t0 * tau;       %[ft]    % Flare height





sim GlideslopeController_model
%% Figures

Altitude_plot = figure(1);
hold on;
plot(ans.Altitude{1}.Values(:))
title('Altitude')
xlabel('Time [s]');
ylabel('Altitude [ft]');
grid on



Thrust_input_plot = figure(12);
hold on;
plot(ans.Thrust_input{1}.Values(:))
title('Thrust_input')
xlabel('Time [s]');
ylabel('Thrust_input [ft]');
grid on

elevator_deflection_input_plot = figure(13);
hold on;
plot(ans.elevator_deflection_input{1}.Values(:))
yline(3);
title('elevator_deflection_input')
xlabel('Time [s]');
ylabel('elevator_deflection_input [ft]');
grid on


h_dot2_plot = figure(10);
hold on;
plot(ans.h_dot2{1}.Values(:))
title('h_dot2')
xlabel('Time [s]');
ylabel('h_dot2 [ft]');
grid on

h_dot3_plot = figure(11);
hold on;
plot(ans.h_dot3{1}.Values(:))
yline(3);
title('h_dot3')
xlabel('Time [s]');
ylabel('h_dot3 [ft]');
grid on


Flare_plot = figure(8);
hold on;
plot(ans.Altitude{1}.Values(:))
title('Altitude')
xlabel('Time [s]');
xlim([136 138])
ylabel('Altitude [ft]');
grid on


Velocity_plot = figure(2);
hold on;
plot(ans.Velocity{1}.Values(:))
title('Velocity')
xlabel('Time [s]');
ylabel('Velocity [ft/s]');
grid on

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











