%% Reduced matrices and simulation

load Trim5000_300


A_red_lo = sel(A_longitude_lo,1:5,1:5);
B_red_lo = sel(A_longitude_lo,1:5,[6,7]);
C_red_lo = eye(5);
D_red_lo = sel(D_longitude_lo,1:5,1:2);

sys_lo=ss(A_red_lo,B_red_lo,C_red_lo,D_red_lo);


sim GlideslopeController_model


%% Figures
