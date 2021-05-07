clear; close all; clc;
%% MIP Hardware
Vb = 7.4; %V Battery Voltage
wf = 1643.5; %rad/s motor shaft free speed
sbar = .0036021; %Nm motor stall torque
G = 143; %gearRatio
Im = 6.5085e-8; %kg*m^2 motor inertia
r = 50e-3; %m wheel radius
mw = 40e-3; %kg wheel mass
mb = 276e-3; %kg body mass
l = 50e-3; %m length from wheel axis to body centroid
Ib =  0.0016; %kg*m^2 body inertia
g= 9.81; % gravity
k = 2.1918e-06;%  motor torque constant
Iw = 2*((mw*r^2)/2 + (Im * G^2)); % wheel inertia

sampleRate1 = 100;
sampleRate2 = 10;
%% Linearization Coeficients
A = (Iw + (mw+mb)*r^2);
B = mb*r*l;
C = (Ib + mb*l^2);
D = mb*g*l;
E = 2*G*sbar;
F = 2*G^2*k;

J1 = B - (A*(B+C))/(A+B);
J2 = F + (F*(B+C))/(A+B);
J3 = (A*D)/(A+B);
J4 = (D*F)/(A+B);

R1 = (A+B);
R2 = (B+C);
R3 = D;

s = tf('s'); %% s-domain variable

Theta = (E/J1)*s; %%define theta

U = s^3 - s^2*(J2/J1) + s*(J3/J1) + (J4/J1);%%define U

Phi = R3/R1 - s^2*(R2/R1); %%define Phi

G1 = minreal(Theta/U); %%define G1
G2 = minreal(Phi/s^2); %%define G2
%% Inner Loop Body Angle Control 
G1poles = pole(G1);
G1zeros = zero(G1);

%lead Controller z < p
z1lead = 5;
p1lead = 70;

%lag Controller z > p
z1lag = 55;
p1lag = 0; 

K1 = -6; %%negative gain
D1lead =(s+z1lead)/(s+p1lead);
D1lag =(s+z1lag)/(s+p1lag);

%include pade approximation to account for DAC h/2 delay
d1 = (1/sampleRate1)/2; 
padeApprox1 = (1-(d1*s/2) + ((d1*s)^2)/12)/(1+(d1*s/2) + ((d1*s)^2)/12);

D1 = K1 * D1lead * D1lag *padeApprox1;

L1 = minreal(G1*D1);
[GM_L1,PM_L1,WCg_L1,WCp_L1] = margin(L1);

[y1,t1] = step((L1)/(1+L1));

%calculate prescalar P from last 25 terms
P1 = 1/mean(y1(end-25 : end));
T1 = (G1*D1)/(1+G1*D1);
%% Outer Loop Wheel Position Control
G2poles = pole(G2);
G2zeros = zero(G2);

%lead Controller z < p
z2lead = .3;
p2lead = 7;

%lag Controller z < p
z2lag = .4;
p2lag = 0; 

K2 = .5; %% gain
D2lead = (s+z2lead)/(s+p2lead);
D2lag =(s+z2lag)/(s+p2lag);

d2 = (1/sampleRate2)/2; 
padeApprox2 = (1-(d2*s/2)+ ((d2*s)^2)/12)/(1+(d2*s/2)+ ((d2*s)^2)/12);

D2 = K2 * D2lead * D2lag *padeApprox2;

L2 = minreal(G2*D2);

[GM_L2,PM_L2,WCg_L2,WCp_L2] = margin(L2);

T2 = minreal((G2*D2)/(1+G2*D2));
%% Successive Loop Closure
L3 = minreal(D2*T1*G2);

Tsystem = minreal((D2*P1*T1*G2)/(1 + D2*P1*T1*G2));
[GM_Ts,PM_Ts,WCg_Ts,WCp_Ts] = margin(Tsystem);
%% Discrete Time Controller
h1 = 1 / sampleRate1;

% Convert to discrete time using tustin approximation with prewarping
%perwarp frequency designed around crossover frequency
opt1 = c2dOptions('Method','tustin','PrewarpFrequency',abs(WCp_L1));
Gz1 = c2d(G1,h1,'zoh');
Dz1 = c2d(D1,h1,opt1);
Tz1 = (Dz1*Gz1)/(1 + Dz1*Gz1);

%calculate discrete prescalar using last 5 terms
[yz1,tz1] = step(Tz1);
dz1Lim = find(tz1 >= .5);
Pz1 = 1/mean(yz1(dz1Lim(1)-5 : dz1Lim(1)));


h2 = 1/sampleRate2;

opt2 = c2dOptions('Method','tustin','PrewarpFrequency',abs(WCp_L2));
Gz2 = c2d(G2,h2,'zoh');
Dz2 = c2d(D2,h2,opt2);
Tz2 = (Dz2*Gz2)/(1 + Dz2*Gz2);

TzSystem = c2d(Tsystem,h2,'zoh');
%% Display Results
G1
G2
D1
P1
D2
Dz1
Pz1
Dz2
T1
T2
Tz1
Tz2
Tsystem
%% Plotting
% ROOT LOCUS PLOTS
figure(1)
subplot(2,1,1)
rlocus(G1)
title('G1')

subplot(2,1,2)
rlocus(G2)
title('G2')
sgtitle('     Plant Root Locus Plot')

figure(2)
subplot(2,1,1)
rlocus(L1)
title('L1=G1(s)*D1(s)')

subplot(2,1,2)
rlocus(L2)
title('L2=G2(s)*D2(s)')

sgtitle('     Loop-Gain Root Locus Plot')

figure(3)
rlocus(Tsystem,'b')
grid on
title('System Transfer Function Bode Plot')

% BODE PLOTS
figure(4)
subplot(2,1,1)
bode(G1)
grid on
title('L1=G1(s)*D1(s)')

subplot(2,1,2)
bode(G2)
grid on
title('L2=G2(s)*D2(s)')

sgtitle('     Plant Bode Plot')

figure(5)
subplot(2,1,1)
bode(L1)
grid on
title('L1=G1(s)*D1(s)')

subplot(2,1,2)
bode(L3)
grid on
title('L3=G2(s)*D2(s)*T1')

sgtitle('     Controller Bode Plot')

figure(6)
margin(Tsystem)
hold on
bode(Tsystem,'b')
grid on
title('System Transfer Function Bode Plot')


% NYQUIST PLOTS
figure(7)
subplot(2,1,1)
nyquist(L1)
title('L1=G1(s)*D1(s)')
subplot(2,1,2)
nyquist(L3)
xlim([-1.5 .8])
title('L2=G2(s)*D2(s)')
sgtitle('     Loop-Gain Nyquist Plot')

% STEP RESPONSES
figure(8)
step(P1 * T1,'b',Pz1 * Tz1,'r',1)
hold on
grid on
legend('Continuous','Discrete')
title('Inner-Loop Transfer Function Step Response')

figure(9)
step(T2,'b',Tz2,'r',5)
hold on
grid on
legend('Continuous','Discrete')
title('Outer-Loop Transfer Function Step Response')

figure(10)
step(Tsystem,'b',TzSystem,'r')
hold on
grid on
legend('Continuous','Discrete')
title('System Transfer Function Step Response')