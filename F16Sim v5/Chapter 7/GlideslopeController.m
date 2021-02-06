%%GLIDESLOPE & FLARE CONTROLLER
%%

%% Reduced matrices and simulation
%%

load Trim5000ft_300fts

A_red_lo = sel(A_longitude_lo, [1 3 4 2 5], [1 3 4 2 5]);
B_red_lo = sel(A_longitude_lo, [1 3 4 2 5], [6 7]);
C_red_lo = sel(C_longitude_lo, [1 3 4 2 5], [1 3 4 2 5]);
D_red_lo = sel(C_longitude_lo, [1 3 4 2 5], [6 7]);

sys_lo=ss(A_red_lo,B_red_lo,C_red_lo,D_red_lo);

Kq = -1;
Ktheta = 2;
Kv = 500;
Kglide = 40;
Kflare = 0.00045;  

% Flare height calculation
x_1 = 1100;                          %[ft]    % Distance intended touchdown point to glideslope transmitter
V_0 = trim_state_lo(7);              %[ft/s]  % Airspeed
V_descent_t0 = V_0 * sin(-3*pi/180); %[ft/s]  % Descent rate at start of the manoeuvre
tau = x_1/(4*V_0+(V_descent_t0/tan(3*pi/180)));%[s] flare geometry time variable 
hflare = -V_descent_t0 * tau;        %[ft]    % Flare height

sim GlideslopeController_model
%% Figures
%%

Glideslope_plot = figure(1);
hold on;
plot(ans.Altitude{1}.Values(:), 'Linewidth', 1)
xlabel('Time [s]');
ylabel('Altitude [ft]');
grid on

Flare_plot = figure(2);
hold on;
plot(ans.Altitude{1}.Values(:), 'Linewidth', 1)
xlabel('Time [s]');
xlim([136 138])
yline(3000+hflare, 'r:', 'Linewidth', 1)
ylabel('Altitude [ft]');
legend('Trajectory', 'Start of flare')
grid on

Vertical_velocity_plot = figure(3);
hold on;
plot(ans.h_dot{1}.Values(:), 'Linewidth', 1)
yline(-3, 'r:', 'Linewidth', 1);
yline(-2, 'r:', 'Linewidth', 1);
xlabel('Time [s]');
ylabel('Vertical velocity [ft/s]');
legend('Velocity', 'Limits')
grid on

Velocity_plot = figure(4);
hold on;
plot(ans.Velocity{1}.Values(:), 'Linewidth', 1)
xlabel('Time [s]');
ylabel('Velocity [ft/s]');
grid on

Glideslope_plot = figure(5);
hold on;
plot(ans.Gamma{1}.Values(:), 'Linewidth', 1)
xlabel('Time [s]');
ylabel('Glideslope error [deg]');
yline(0, 'r:', 'Linewidth', 1)
legend('Glideslope error', 'Reference')
grid on


Thrust_plot = figure(6);
hold on;
plot(ans.Thrust_input{1}.Values(:),  'Linewidth', 1)
xlabel('Time [s]');
ylabel('Thrust [lb]');
grid on

Elevator_deflection_plot = figure(7);
hold on;
plot(ans.elevator_deflection_input{1}.Values(:),  'Linewidth', 1)
yline(3);
xlabel('Time [s]');
ylabel('Elevator deflection [deg]');
grid on











