%%
%% OPEN LOOP ANALYSIS
%%

clear all
load('300fts20000ft.mat')

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

%% State space system for both models and finding the eigenvectors
%%
ss_longitudinal = ss(longitudinal_A, longitudinal_B, longitudinal_C, longitudinal_D);
ss_lateral = ss(lateral_A, lateral_B, lateral_C, lateral_D);

[eigenvectors_longitudinal, eigenvalues_longitudinal] = eig(longitudinal_A);
[eigenvectors_lateral, eigenvalues_lateral] = eig(lateral_A);

eigenvalue_shortperiod = eigenvalues_longitudinal(1, 1);
eigenvalue_phugoid = eigenvalues_longitudinal(3, 3);
eigenvalue_dutchroll = eigenvalues_lateral(1, 1);
eigenvalue_aperiodicroll = eigenvalues_lateral(3, 3);
eigenvalue_spiral = eigenvalues_lateral(4, 4);

%% Calculate natural frequency, damping ratio, period, and the time to damp to half the amplitude for periodic eigenmotions
%%
% Natural frequency
shortperiod_natfreq = sqrt((real(eigenvalue_shortperiod))^2 + (imag(eigenvalue_shortperiod))^2);
phugoid_natfreq = sqrt((real(eigenvalue_phugoid))^2 + (imag(eigenvalue_phugoid))^2);
dutchroll_natfreq = sqrt((real(eigenvalue_dutchroll))^2 + (imag(eigenvalue_dutchroll))^2);

% Damping ratio
shortperiod_damping = -real(eigenvalue_shortperiod) / shortperiod_natfreq;
phugoid_damping = -real(eigenvalue_phugoid) / phugoid_natfreq;
dutchroll_damping = -real(eigenvalue_dutchroll) / dutchroll_natfreq;

% Period
shortperiod_period = (2*pi)/imag(eigenvalue_shortperiod);
phugoid_period = (2*pi)/imag(eigenvalue_phugoid);
dutchroll_period = (2*pi)/imag(eigenvalue_dutchroll);

% Time to damp to half the amplitude
shortperiod_amplhalf = log(1/2)/real(eigenvalue_shortperiod);
phugoid_amplhalf = log(1/2)/real(eigenvalue_phugoid);
dutchroll_amplhalf = log(1/2)/real(eigenvalue_dutchroll);

periodic_motions = [[shortperiod_natfreq, shortperiod_damping, shortperiod_period, shortperiod_amplhalf]
                    [phugoid_natfreq, phugoid_damping, phugoid_period, phugoid_amplhalf]
                    [dutchroll_natfreq, dutchroll_damping, dutchroll_period, dutchroll_amplhalf]];

%% Calculate natural frequency, time constant and time to damp to half the amplitude for aperiodic eigenmotions
%%
% Natural frequency
aperiodicroll_natfreq = sqrt((real(eigenvalue_aperiodicroll))^2 + (imag(eigenvalue_aperiodicroll))^2);
spiral_natfreq = sqrt((real(eigenvalue_spiral))^2 + (imag(eigenvalue_spiral))^2);

% Time constant
aperiodicroll_timecon= 1/aperiodicroll_natfreq;
spiral_timecon = 1/spiral_natfreq;

% Time to damp to half the amplitude
aperiodicroll_amplhalf = log(1/2)/real(eigenvalue_aperiodicroll);
spiral_amplhalf = log(1/2)/real(eigenvalue_spiral);

aperiodic_motions = [[aperiodicroll_natfreq, aperiodicroll_timecon, aperiodicroll_amplhalf]
                     [spiral_natfreq, spiral_timecon, spiral_amplhalf]];

%% Time responses plotted
%%
% Options for step responses
opt_neg_el = stepDataOptions('InputOffset', 0 ,'StepAmplitude', -1);
opt_aileron = stepDataOptions('InputOffset', 0 ,'StepAmplitude', -1);

% Time spans
shortperiod_time =  0:0.1:10;
phugoid_time =      0:0.1:750;
dutchroll_time =    0:0.1:200;
aperiodicroll_time= 0:0.1:10;
spiral_time =       0:0.1:750;

% Time responses
[shortperiod_y, shortperiod_time] = step(ss_longitudinal, opt_neg_el, shortperiod_time);
[phugoid_y, phugoid_time] = step(ss_longitudinal, opt_neg_el, phugoid_time);
[dutchroll_y, dutchroll_time] =  impulse(ss_lateral);
[aperiodic_y, aperiodicroll_time] = step(ss_lateral, opt_aileron, aperiodicroll_time);
[spiral_y, spiral_time] = step(ss_lateral, opt_aileron, spiral_time);

%% Plotting the results
%%

% Periodic eigenmotions
% Short period
figure
plot(shortperiod_time, shortperiod_y(:, 3), 'LineWidth', 1)
xlabel('Time [s]')
ylabel('Pitch angle [deg]')
title('Pitch angle for the short period eigenmotion')
grid

figure
plot(shortperiod_time, shortperiod_y(:, 4), 'LineWidth', 1)
xlabel('Time [s]')
ylabel('Pitch rate [deg/s]')
title('Pitch rate for the short period eigenmotion')
grid

% Phugoid
figure
plot(phugoid_time, phugoid_y(:, 3), 'LineWidth', 1)
xlabel('Time [s]')
ylabel('Pitch angle [deg]')
title('Pitch angle for the phugoid eigenmotion')
grid
xlim([0 600])

figure
plot(phugoid_time, phugoid_y(:, 4), 'LineWidth', 1)
xlabel('Time [s]')
ylabel('Pitch rate [deg/s]')
title('Pitch rate for the phugoid eigenmotion')
grid
xlim([0 600])

% Dutch roll
figure
plot(dutchroll_time, dutchroll_y(:, 3), 'LineWidth', 1)
xlabel('Time [s]')
ylabel('Roll rate [deg/s]')
title('Roll rate for the Dutch roll eigenmotion')
grid
xlim([0 150])


figure
plot(dutchroll_time, dutchroll_y(:, 1), 'LineWidth', 1)
xlabel('Time [s]')
ylabel('Yaw rate [deg/s]')
title('Yaw rate for the Dutch roll eigenmotion')
grid
xlim ([0 150])
ylim ([-2 2])

% Aperiodic eigenmotions
% Aperiodic roll
figure
plot(aperiodicroll_time, aperiodic_y(:, 2), 'LineWidth', 1)
xlabel('Time [s]')
ylabel('Roll angle [deg]')
title('Roll angle for the aperiodic roll eigenmotion')
grid

figure
plot(aperiodicroll_time, aperiodic_y(:, 3), 'LineWidth', 1)
xlabel('Time [s]')
ylabel('Roll rate [deg/s]')
title('Roll rate for the aperiodic roll eigenmotion')
grid

% Spiral
figure
plot(spiral_time, spiral_y(:, 2), 'LineWidth', 1)
xlabel('Time [s]')
ylabel('Roll angle [deg]')
title('Roll angle for the spiral eigenmotion')
grid
xlim([0 600])

figure
plot(spiral_time, spiral_y(:, 3), 'LineWidth', 1)
xlabel('Time [s]')
ylabel('Roll rate [deg/s]')
title('Roll rate for the spiral eigenmotion')
grid
xlim([0 600])

