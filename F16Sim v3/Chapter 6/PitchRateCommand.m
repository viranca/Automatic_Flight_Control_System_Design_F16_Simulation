%% PITCH RATE CONTROLLER
%%

clear all
close all

%% Load reduced longitudial system from the open loop analysis
%%
load('Reduced_system.mat')

%% Construct the short period reduced model
%%
% Convert (longitudinal) 4 state matrices to 2 state
A_2s = longitudinal_A([2 4], [2 4]);
B_2s = longitudinal_B([2 4], 1);
C_2s = longitudinal_C([2 4], [2 4]);
D_2s = longitudinal_D([2 4], 1);

% Create state-space model of 2 states
lon_model_2s = ss(A_2s, B_2s, C_2s, D_2s);

%% Compare time responses of the reduced 2 state and 4 state models
%%
% Specify time intervals and timesteps 
time_st = 0:0.1:15;          % short term time
time = 0:0.1:300;            % long term time

% Create state-space model of 4 states
lon_model_4s = ss(longitudinal_A, longitudinal_B, longitudinal_C, longitudinal_D);

% Store step responses of the two models for a short and long duration
Y4_st = step(-lon_model_4s, time_st);
Y2_st = step(-lon_model_2s, time_st); 

Y4 = step(-lon_model_4s, time);
Y2 = step(-lon_model_2s, time);                

% Plotting the time responses of the 2 systems for comparison
% Short term plot
figure(1)
plot(time_st, Y4_st(:,4), 'k', 'Linewidth', 1)
hold on 
plot(time_st, Y2_st(:,2), 'r', 'Linewidth', 1)
xlabel('Time [s]')
ylabel('Pitch rate [deg/s]')
grid on 
legend('4 State Model', '2 State Model')

% Long term plot
figure(2)
plot(time, Y4(:,4), 'k', 'Linewidth', 1)
hold on 
plot(time, Y2(:,2), 'r', 'Linewidth', 1)
xlabel('Time [s]')
ylabel('Pitch rate [deg/s]')
grid on 
legend('4 State Model', '2 State Model')

%% Converting CAP and Gibson requirements to the familiar frequency domain
%%
% Parameters
V_ft = 300;                  % ft/s
V_m = 0.3048 * V_ft;         % m/s

% Requirements
nat_freq = 0.03 * V_m;                 % rad/s
time_con_req = 1 / (0.75 * nat_freq);  % s
damp_ratio = 0.5; 

%% Pitch Rate Command System (Satisfying Requirements)
%%
% Parameters
V_gust = 4.572;                  % wind gust velocity in m/s

% Computing the required poles
pole_1 = (-damp_ratio * nat_freq) + nat_freq * ((damp_ratio^2 - 1))^0.5;
pole_2 = (-damp_ratio * nat_freq) - nat_freq * ((damp_ratio^2 - 1))^0.5;
poles = [pole_1 pole_2];

% Computing the gains
gains = place(A_2s, B_2s, poles); % compute gain matrix
K_alpha = gains(1);                     % deg/rad
K_q = gains(2);                         % deg/(rad/s)

% Compute transferfunction with poleplacement
A_ac_2s_new = A_2s - B_2s*gains;                          % compute change in A-matrix
lon_model_2s_new = ss(A_ac_2s_new,B_2s, C_2s, D_2s);      % new state-space system

H_CL = tf(lon_model_2s_new);                              % tf after poleplacement
H_CL_alpha = H_CL(1);
H_CL_q = H_CL(2);

% Computing the gust influences
alpha_i = atan(V_gust/V_m);          % Induced angle of attack in radians
alpha_i_deg = alpha_i * (180/pi);    % Induced angle of attack in degrees
el_def = K_alpha * alpha_i;          % Elevator deflection

%% Lead-lag filter
%%
s = tf('s');

% Compute the current time constant
H_CL_q = zpk(H_CL_q);
[num, den] = ss2tf(A_2s, B_2s, C_2s, D_2s);
time_con_cur = num(2, end-1)/num(2, end);                   % calculate current time constant from numerators of tf function

% Compute transferfunction of lead-lag filter
H_LL = (time_con_req * s + 1)/(time_con_cur * s + 1);       % a gain value K should be included

% Pole-zero cancellation
H_inc_LL = minreal(H_LL * H_CL);                            % transfer function including lead lag filter
H_alpha_new = H_inc_LL(1);   
H_q_new = H_inc_LL(2);

%% CAP & Gibson Criteria
%%
% Parameters
g = 9.81;            % m/s^2

% Current CAP and Gibson parameters 
[num, den] = tfdata(H_q_new, 'v');
time_con_cur2 = num(2)/num(3);                       % checking current time constant
[wn_cur,zeta_cur] = damp(H_q_new);                   % checking current natural frequency and damp ratio
par_cur = [wn_cur(1), zeta_cur(1), time_con_cur2];   % current parameters

% CAP and Gibson parameters design 
H_design = tf(lon_model_4s);                 % transfer function of 2 state system design adjustments
[wn_design, zeta_design] = damp(H_design);   % natural frequency and damping ratio design adjustments
par_design = [wn_design(3), zeta_design(3), time_con_cur];   % parameters design

% CAP calculations of current parameters and design
CAP_cur = (par_cur(1)^2 * g * par_cur(3)) / (V_m);            % CAP current
CAP_design = (par_design(1)^2 * g * par_design(3)) / (V_m);   % CAP design adjustments

% Compute transfer function for pitch angle
H_angle = minreal(H_q_new * (1/s));

% Construct an input step signal
Time = 0:0.01:5;
Time2 = 5.01:0.01:10;
Time3 = [Time Time2];

u1 = ones(size(Time)) * -1;
u2 = zeros(size(Time2));
u3 = [u1 u2];

% Compute responses for pitch rate and attitude
Y_q = lsim(H_q_new, u3, Time3);
Y_att = lsim(H_angle, u3, Time3); 

% Plot responses for pitch rate and attitude
figure(3)
plot(Time3, Y_q, 'Linewidth', 1)
xlabel('Time [s]')
ylabel('Pitch rate [deg/s]')
grid on 

figure(4)
plot(Time3, Y_att, 'Linewidth', 1)
xlabel('Time [s]')
ylabel('Pitch angle [deg]')
grid on 

% Parameters from pitch rate and attitude response 
q_m = max(Y_q);                % Maximum pitch rate
q_s = Y_q(500);                % Steady state value pitch Rate
q_overshoot = q_m/q_s;         % Pitch rate overshoot ratio
DB = max(Y_att) - Y_att(end);  % Dropback
DB_div_q_s_design = DB/q_s;    % Gibson Criteria design

DB_div_q_s_current = time_con_cur2 - ((2 * zeta_cur(1) )/ wn_cur(1));        %Gibson Criteria current