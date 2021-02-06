%clear all;
close all;
clc;

load('Reduced_system.mat')

A_long_lo = longitudinal_A;
B_long_lo = longitudinal_B;
C_long_lo = longitudinal_C;
D_long_lo = longitudinal_D;

mpsftps = 3.2808399;    % 1 Meter = 3.2808399 feet
velocity = 300;         % Assumed flight condition
vert_gust = 4.572;      % in m/s
gD = 9.80665;           % gravitation constant

%% Q1: Short period reduced model from the model without the actuator dynamics

A_contr = longitudinal_A([2 4], [2 4]);
B_contr = longitudinal_B([2 4], 1);
C_contr = longitudinal_C([2 4], [2 4]);
D_contr = longitudinal_D([2 4], 1);

%Create state-space model, convert to state-space model
ss_contr = ss(A_contr,B_contr,C_contr,D_contr);

%% Q2: Compare time responses of 4 and 2 state models

% Pitch rate step response 4 state model
% Convert state-space filter parameters to transfer function form
[num_long,den_long] = ss2tf(A_long_lo,B_long_lo,C_long_lo,D_long_lo,1);
% Create transfer function model
tf_long = tf(num_long(4,:),den_long);
% Minimal realization or pole-zero cancelation
tf_long = minreal(tf_long);
% Convert unconstrained MPC controller to zero/pole/gain form
tf_long = zpk(tf_long);

% Pitch rate step response reduced 2 state model
ss_contr  = ss(A_contr,B_contr,C_contr,D_contr);
[num_contr,den_contr]  = ss2tf(A_contr,B_contr,C_contr,D_contr);
tf_contr  = zpk(tf(ss_contr));

% % Plot the step response for a longer period of time
Time = 0:1:250;
figure;
step(-tf_long,Time);
hold on
step(-tf_contr(2),Time);
hold off
legend('4 state model','2 state model','Location','North')
plotname = ['step response 2 and 4 state model, t_max = ' num2str(Time(end)) ' s.png'];
saveas(gcf,plotname)

% Plot the step response for a schorter period of time: 'zoom' in on the
% first few seconds
Time = 0:0.01:8;
figure;
step(-tf_long,Time);
hold on
step(-tf_contr(2),Time);
hold off
legend('4 state model','2 state model','Location','North')
plotname = ['step response 2 and 4 state model, t_max = ' num2str(Time(end)) ' s.png'];
saveas(gcf,plotname)

%% Q3: Convert Gibson and CAP criteria to frequency domain
% velocity multiplied by mpsftps to go to the m/s
w_n_sp    = 0.03 * velocity / mpsftps;      % Desired nat freq 
T_theta_2 = 1 / ( 0.75 * w_n_sp );          % Desired parameter T_theta_2
d_sp      = 0.5;                            % Desired damping ratio

%% Q4: Get the necessary feedback gains for angle of attack and pitch rate
% Influence of the gust wind on the angle of attack
a_gust = atan(vert_gust*mpsftps/velocity);

%Pole placement based on the imposed requirements
pole = [complex(- w_n_sp * d_sp , w_n_sp * sqrt(1-d_sp^2)); complex(- w_n_sp * d_sp , - w_n_sp * sqrt(1-d_sp^2))];
% place -> Pole placement design
gain = place(A_contr,B_contr,pole);
Ka   = gain(1);
Kq   = gain(2);
%Required elevator deflection
d_el = Ka * a_gust;

% Identify possible feedback data
tf_contr_4 = feedback(tf_contr,gain*pi/180);

%% Q5: Obtain current T_theta_2, w_n_sp, d_sp

% Get current data from current transfer function tf_contr
w_n_sp_current    = sqrt(den_contr(end));                       % = last term of denominator squared
T_theta_2_current = num_contr(2,end-1)/num_contr(2,end);        % numerator of tf_contr
d_sp_current      = den_contr(end-1) / ( 2 * w_n_sp_current );  % middle term of denominator = 2 * d_sp * w_n_sp
Kq_current        = num_contr(2,end);

% Apply pole-zero cancellation to the transfer function
pzfilter      = zpk(tf([T_theta_2 1], [T_theta_2_current 1]));
tf_contr_corr = minreal(tf_contr_4 * pzfilter);
tf_contr_corr = zpk(tf_contr_corr);

Time = 0:0.1:12;
[amplitude, timeline]=step(tf_contr_corr(2),Time);

% Transfer function with prefilter
figure;
%[amplitude, timeline]=step(tf_contr_corr(2),Time);
plot(timeline,amplitude,'-g')
% hold on
% step(-tf_contr(2),Time);
% hold off
legend('transfer function with prefilter','transfer function without prefilter','Location','North')
plotname = ['step response with and without prefilter, t_max = ' num2str(Time(end)) ' s.png'];
%saveas(gcf,plotname)

tf_contr_corr(2) = 1/amplitude(end) * tf_contr_corr(2)

 
%% Q6: CAP and Gibson criteria: calculate current parameter values
% Calculate the desired CAP and Gibson criteria
CAP_required         = w_n_sp^2 / ( velocity / ( gD * mpsftps * T_theta_2) )
DB_over_qss_required = T_theta_2 - 2 * d_sp / w_n_sp

% Calculate the current CAP and Gibson criteria
CAP_current          = w_n_sp_current^2 / ( velocity / ( gD * mpsftps * T_theta_2_current) )
DB_over_qss_current  = T_theta_2_current - 2 * d_sp_current / w_n_sp_current

% Calculate the final CAP and Gibson criteria
% w_n_sp_final      = sqrt(67.73);
% T_theta_2_final   = 1/6.172;
% d_sp_final        = 8.23 / ( 2 * w_n_sp_current );
% CAP_final         = w_n_sp_final^2 / ( velocity / ( gD * mpsftps * T_theta_2_final) )
% DB_over_qss_final = (0.0795-0.0571)/0.0571%T_theta_2_final - 2 * d_sp_final / w_n_sp_final

% The obtained DB over q_ss can not be calculated by the given formula
% since that formula is only for open loop systems. Hence the following
% procedure is used:

tf_integrate = tf(1,[1 0]);
tf_theta    = tf_integrate * tf_contr_corr(2);
Time = 0:0.001:20;
testinput    = [ ones(1,round(length(Time)/2)) zeros(1,round(length(Time)/2)-1) ];

[q,t]=lsim(tf_contr_corr(2),testinput,Time);
figure;
subplot(2,1,1); plot(t,testinput)
title('Input signal')
ylabel('Input u(t)')
ylim([-0.2 1.2])
subplot(2,1,2); plot(t,q)
title('Pitch rate response')
ylabel('Pitch rate q(t)')
xlabel('Time [s]')
print -dpng PitchrateDropBack

[theta,t]=lsim(tf_theta,testinput,Time);
figure;
plot(t,theta);
ylabel('Pitch angle \theta(t)')
xlabel('Time [s]')
print -dpng PitchAngleDropBack

%CAP_final         = w_n_sp_final^2 / ( velocity / ( gD * mpsftps * T_theta_2_final) )
DB_over_qss_final = ( max(theta)-theta(end))/q(round(length(q)/2)-1)%
DB_over_qss_finall= T_theta_2 - 2 * d_sp / w_n_sp
qm_over_qs = max(q)/q(round(length(q)/2)-1)

%% Q7: Verify gain and phase margin requirements

[Gm,Pm,Wg,Wp] = margin(tf_contr_corr(2))
20*log(Gm)/log(10)

figure;
margin(tf_contr_corr(2))
print -dpng BodeDiagram