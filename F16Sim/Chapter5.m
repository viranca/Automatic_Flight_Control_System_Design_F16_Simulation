close all
clear all

x_a = 0; %change accordingly
g_D = 9.80665;

%load linearised models for different accelerometer positions
acc0 = load('accelerometer.mat');
acc5 = load('accelerometer5.mat');
acc59 = load('accelerometer59.mat');
acc6 = load('accelerometer6.mat');
acc7 = load('accelerometer7.mat');
acc15 = load('accelerometer15.mat');
%% reduce eqns to output only a_n and have d_e as only input

s = tf('s');

A_red0 = acc0.A_lo;
B_red0 = acc0.B_lo(:,2);
C_red0 = acc0.C_lo(19,:);
D_red0 = acc0.D_lo(19,2); 

A_red5 = acc5.A_lo;
B_red5 = acc5.B_lo(:,2);
C_red5 = acc5.C_lo(19,:);
D_red5 = acc5.D_lo(19,2);

A_red59 = acc59.A_lo;
B_red59 = acc59.B_lo(:,2);
C_red59 = acc59.C_lo(19,:);
D_red59 = acc59.D_lo(19,2);

A_red6 = acc6.A_lo;
B_red6 = acc6.B_lo(:,2);
C_red6 = acc6.C_lo(19,:);
D_red6 = acc6.D_lo(19,2);

A_red7 = acc7.A_lo;
B_red7 = acc7.B_lo(:,2);
C_red7 = acc7.C_lo(19,:);
D_red7 = acc7.D_lo(19,2);

A_red15 = acc15.A_lo;
B_red15 = acc15.B_lo(:,2);
C_red15 = acc15.C_lo(19,:);
D_red15 = acc15.D_lo(19,2);

%build the systems
sys0 = ss(A_red0, B_red0, C_red0, D_red0);
sys5 = ss(A_red5, B_red5, C_red5, D_red5);
sys59 = ss(A_red59, B_red59, C_red59, D_red59);
sys6 = ss(A_red6, B_red6, C_red6, D_red6);
sys7 = ss(A_red7, B_red7, C_red7, D_red7);
sys15 = ss(A_red15, B_red15, C_red15, D_red15);

% H0 = minreal(tf(sys0));
% H5 = minreal(tf(sys5));
% H59 = minreal(tf(sys59));
% H6 = minreal(tf(sys6));
% H7 = minreal(tf(sys7));
% H15 = minreal(tf(sys15));

%% setup step response
%step response options
opt = stepDataOptions('InputOffset',0 ,'StepAmplitude',-10);
t = 0:0.01:0.5;

%calculate responses
[y0, t0] = step(sys0,opt,t);
[y5, t5] = step(sys5,opt,t);
[y59, t59] = step(sys59,opt,t);
[y6, t6] = step(sys6,opt,t);
[y7, t7] = step(sys7,opt,t);
[y15, t15] = step(sys15,opt,t);


%% plot all responses 
figure('DefaultAxesFontSize',14)
plot(t0,y0,'r','LineWidth',1)
hold on
plot(t5,y5,'c','LineWidth',1)
plot(t59,y59,'k','LineWidth',1)
plot(t6,y6,'g','LineWidth',1)
plot(t7,y7,'m','LineWidth',1)
plot(t15,y15,'b','LineWidth',1)

xlabel('Time [s]')
ylabel('Normal acceleration a_n [g]')

grid

legend('x_a = 0 ft', 'x_a = 5 ft', 'x_a = 5.9 ft', 'x_a = 6 ft', 'x_a = 7 ft', 'x_a = 15 ft');
