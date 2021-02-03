%%
%% OPEN LOOP ANALYSIS
%%

clear all
load('300fts20000ft.mat')
s = tf('s');

%% Determine the longitudinal and lateral reduced state space model
%%
longitudinal_A = A_longitude_lo([3 4 2 5],[3 4 2 5]);
longitudinal_B = A_longitude_lo([3 4 2 5], 7);
longitudinal_C = C_longitude_lo([3 4 2 5],[3 4 2 5]);
longitudinal_D = D_longitude_lo([3 4 2 5], 2);

lateral_A = A_lateral_lo([4 1 5 6],[4 1 5 6]);
lateral_B = A_lateral_lo([4 1 5 6], [8 9]); 
lateral_C = C_lateral_lo([4 1 5 6],[4 1 5 6]);
lateral_D = D_lateral_lo([4 1 5 6], [1 2]);

save('Reduced_system','longitudinal_A', 'longitudinal_B', 'longitudinal_C', 'longitudinal_D')

%% Steady state system for both models and damped
%%
ss_longitudinal = ss(longitudinal_A, longitudinal_B, longitudinal_C, longitudinal_D);
ss_lateral = ss(lateral_A, lateral_B, lateral_C, lateral_D);
[Wn_long, zeta_long, P_long] = damp(ss_longitudinal);
[Wn_lat, zeta_lat, P_lat] = damp(ss_lateral)

%% Calculate natural frequency, damping ratio, period, and the time to damp to half the amplitude for periodic eigenmotions
%%

% Poles
P_shortperiod_index = find(real(P_long) == min(real(P_long)));
P_phugoid_index = find(real(P_long) == max(real(P_long)));
P_dutchroll_index = find(imag(P_lat) ~= 0.0);

% Natural frequency
Wn_shortperiod = unique(Wn_long(P_shortperiod_index));
Wn_phugoid = unique(Wn_long(P_phugoid_index));
Wn_dutchroll = unique(Wn_lat(P_dutchroll_index));

% Damping ratio
zeta_shortperiod = unique(zeta_long(P_shortperiod_index));
zeta_phugoid = unique(zeta_long(P_phugoid_index));
zeta_dutchroll = unique(zeta_lat(P_dutchroll_index));

% Period
period_shortperiod = 2*pi/imag(P_long(P_shortperiod_index));
period_phugoid = 2*pi/imag(P_long(P_phugoid_index));
period_dutchroll = 2*pi/imag(P_lat(P_dutchroll_index));

% Time to damp to half the amplitude
T1_2_shortperiod = log(1/2)/real(P_long(P_shortperiod_index));
T1_2_phugoid = log(1/2)/real(P_long(P_phugoid_index));
T1_2_dutchroll = log(1/2)/real(P_lat(P_dutchroll_index));

%% Calculate natural frequency, time constant and time to damp to half the amplitude for aperiodic eigenmotions
%%

% Poles
P_aperiodic_index = find(real(P_lat) == min(real(P_lat)));
P_spiral_index = find(real(P_lat) == max(real(P_lat)));

% Natural frequency
Wn_aperiodic = unique(Wn_lat(P_aperiodic_index));
Wn_spiral = unique(Wn_lat(P_spiral_index));

% Time constant
tau_aperiodic = -1/P_lat(P_aperiodic_index);
tau_spiral = -1/P_lat(P_spiral_index);

% Time to damp to half the amplitude
T1_2_aperiodic = log(1/2)/real(P_lat(P_aperiodic_index));
T1_2_spiral = log(1/2)/real(P_lat(P_spiral_index));

%% Step responses
%%

% Options for step responses
opt_shortperiod_phugoid = stepDataOptions('InputOffset',[0 0] ,'StepAmplitude', [ 0 -1]);
opt_aperiodic_spiral = stepDataOptions('InputOffset',[0 0 0] ,'StepAmplitude', [ 0 -1 0]);

% Timespans for the different eigenmotions
t_shortperiod = 0:0.1:10;
t_phugoid = 0:0.1:1500;
t_dutchroll = 0:0.1:50;
t_aperiodic = 0:0.1:10;
t_spiral = 0:0.1:1000;

% Create responses
[y_shortperiod, t_shortperiod] = step(ss_longitudinal, opt_shortperiod_phugoid, t_shortperiod);
[y_phugoid, t_phugoid] = step(ss_longitudinal, opt_shortperiod_phugoid, t_phugoid);
[y_dutchroll, t_dutchroll] =  impulse(ss_lateral);
[y_aperiodic, t_aperiodic] = step(ss_lateral, opt_aperiodic_spiral, t_aperiodic);
[y_spiral, t_spiral] = step(ss_lateral, opt_aperiodic_spiral, t_spiral);



%% Plotting the results
%%

% Periodic eigenmotions
% Short period
figure
plot(t_shortperiod, y_shortperiod(:,1,2))
xlabel('Time [s]')
ylabel('Pitch angle [deg]')
title('Pitch angle for the short period eigenmotion')

figure
plot(t_shortperiod, y_shortperiod(:,4,2))
xlabel('Time [s]')
ylabel('Pitch rate [deg/s]')
title('Pitch rate for the short period eigenmotion')

% Phugoid
figure
plot(t_phugoid, y_phugoid(:,1,2))
xlabel('Time [s]')
ylabel('Pitch angle [deg]')
title('Pitch angle for the phugoid eigenmotion')

figure
plot(t_phugoid, y_phugoid(:,4,2))
xlabel('Time [s]')
ylabel('Pitch rate [deg/s]')
title('Pitch rate for the phugoid eigenmotion')

% Dutch roll
figure
plot(t_dutchroll, y_dutchroll(:,3,3))
xlabel('Time [s]')
ylabel('Roll rate [deg/s]')
title('Roll rate for the Dutch roll eigenmotion')

figure
plot(t_dutchroll, y_dutchroll(:,4,3))
xlabel('Time [s]')
ylabel('Yaw rate [deg/s]')
title('Yaw rate for the Dutch roll eigenmotion')

% Aperiodic eigenmotions
% Aperiodic roll
figure
plot(t_aperiodic, y_aperiodic(:,1,2))
xlabel('Time [s]')
ylabel('Roll angle [deg]')
title('Roll angle for the aperiodic roll eigenmotion')

figure
plot(t_aperiodic, y_aperiodic(:,3,2))
xlabel('Time [s]')
ylabel('Roll rate [deg/s]')
title('Roll rate for the aperiodic roll eigenmotion')

% Spiral
figure
plot(t_spiral, y_spiral(:,1,2))
xlabel('Time [s]')
ylabel('Roll angle [deg]')
title('Roll angle for the spiral eigenmotion')

figure
plot(t_spiral, y_spiral(:,3,2))
xlabel('Time [s]')
ylabel('Roll rate [deg/s]')
title('Roll rate for the spiral eigenmotion')

