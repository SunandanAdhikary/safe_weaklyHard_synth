clear all;
clc;
%{ esp_bicycle, dcmotor_pos, quadrotor, aircraft, ewb,
% suspension_control, fuel_injection, driveline_mngmnt, esp, ttc 
%}
system = "suspension_control"
format long;
% if system ~= systm
%     disp("here")
%     clear all;
%     system=sys;
% end
time = 40;
%% systems
%% esp_bicycle [Secure State Estimation with Cumulative Message Authentication]
if system =="esp_bicycle"
    m = 1573; % kg
    lf = 1.1; % m
    lr = 1.58; %m
    Iz = 2873; %kgm^2
    Caf = 80000;
    Car = 80000;
    vxref = 35; %m/s
    A = [0,1,0,0;...
        0,(-2*Caf*Car)/(m*vxref),(2*Caf*Car)/m,2*(-Caf*lf+Car*lr)/(m*vxref);...
        0,0,0,1;...
        0,(-2*Caf*lf-Car*lr)/(Iz*vxref),(-2*Caf*lf-Car*lr)/Iz,(-2*Caf*lf^2+Car*lr^2)/(Iz*vxref)];
    B = [0;2*Caf/m;0;2*Caf*lf/Iz];
    C = blkdiag(0,1,0,1);
    D = zeros(size(B));
    p = 0.00001;
    Q = p*(C'*C);
    R = 0.000001;
    [K,S,E] = dlqr(A,B,Q,R);
    sys_ss =ss(A-B*K,B,C,D,Ts);
    qweight= 1;
    rweight = 1;
    QN = 5000000;
    RN = 10*eye(1);
    [kalmf,L,P,M] = kalman(sys_ss,QN,RN,Ts);
    % K = [-0.0987 0.1420];
    L = [-0.0390;0.4339];
    settling_time = 6;
    proc_dev= 0.001; meas_dev=0.00001;
    safex=[-1,-2;1,2];
    ini= 0.2.*safex;
    perf=0.1.*safex;
    sensorRange = [-2.5;2.5] ;     % columnwise range of each y
    actuatorRange = [-0.8125;0.8125]; % columnwise range of each u
end
%% esp_bicycle
%{
if system=="esp_bicycle"
Ts=0.04;
A = [0.6278   -0.0259;
    0.4644    0.7071];

B = [0.1246   -0.00000028;
    3.2763    0.000016];

C = [0    1.0000
 -338.7813    1.1293];

D = [0         0;
  169.3907         0];

K = [5.261 -0.023;
    -414911.26, 57009.48];

L = [-0.00000000002708 -0.00000000063612;
    0.00000000033671  0.00000000556308];

safex = [1,2];
ini = 1;
perf = 0.2;
th = 4.35; 
settlingTime = 13 ;
sensorRange = [2.5;15];  % columnwise range of each y
actuatorRange = [0.8125;0.8125]; % columnwise range of each u
noisy_zvar=0.1;
noisy_zmean= 0.5;
proc_dev= 0.1; meas_dev=0.001;
end
%}
%% electronic stability program/vehicle lateral dynamic control(skip2secure,same as early)-- in Electronic Stability Program ECU
if system=="esp"
    Ts=0.04;
    A = [0.4450 -0.0458;1.2939 0.4402];
    B = [0.0550;4.5607];
    C = [0 1];
    D = [0];
    p = 0.00001;
    Q = p*(C'*C);
    R = 0.000001;
    [K,S,E] = dlqr(A,B,Q,R);
    sys_ss =ss(A-B*K,B,C,D,Ts);
    qweight= 1;
    rweight = 1;
    QN = 5000000;
    RN = 10*eye(1);
    [kalmf,L,P,M] = kalman(sys_ss,QN,RN,Ts);
    % K = [-0.0987 0.1420];
    L = [-0.0390;0.4339];
    settling_time = 6;
    proc_dev= 0.001; meas_dev=0.00001;
    safex=[-1,-2;1,2];
    ini= 0.2.*safex;
    perf=0.1.*safex;
    sensorRange = [-2.5;2.5] ;     % columnwise range of each y
    actuatorRange = [-0.8125;0.8125]; % columnwise range of each u
end

%% trajectory tracking/automatic cruise control (skip2secure,same as early)--in Engine Control Module/Unit ECU
if system=="ttc"
    Ts = 0.1;
    A = [1.0000    0.1000; 0    1.0000];
    B = [0.0050; 0.1000];
    C = [1 0];
    D = [0];
    K = [16.0302    5.6622];
    L = [0.9902; 0.9892];
    settling_time = 12;
    safex=[-25,-30;25,30];
    ini=0.1.*safex;
    perf=0.3.*safex;
    proc_dev= 0.01; meas_dev=0.0001;
    sensorRange = [-30;30];  % columnwise range of each y
    actuatorRange = [-36;36];   % columnwise range of each u
    Q = 300*C'*C/(max(safex(2,:))^2);
    R = blkdiag(1/(actuatorRange(2,:).^2));
    QN = 900000/(proc_dev*proc_dev)*(B*B');
    RN = 0.0001/(meas_dev*meas_dev);
% -----new values for better intermediate safety---
%     Q= eye(size(s.A,2));
%     R= eye(size(s.B,2));
%     [K,S,E] = dlqr(s.A,s.B,Q,R);
%     QN = 1500;
%     RN = eye(1);
%     sys_ss = ss(s.A,s.B,s.C,s.D,s.Ts);
%     [kalmf,L,P,M] = kalman(sys_ss,QN,RN);
%       K = [0.9171    1.6356];  
%       L = [0.8327;   2.5029];
%     actuatorRange = [-72;72];   % columnwise range of each u
% -------------------------------------------------
end

%% anti-braking system (incomplete)
if system=="abs"
 alpha=9.75;
 beta=0.1;
 Fn=4414;
 m=450;
 R=0.32;
 J=1;
 v= 30;% collect from ttc output

     A1c =[0 (-1*alpha*Fn)/m;
         (alpha*Fn*R*R)/(v*v*J) (-1*alpha*Fn*R*R)/(v*J)];
     A2c(:,:,i)=[0 Fn/(4*m);
         ((-1/4+3/4-0.2)*Fn*R*R)/(v(i)*v(i)*J) (Fn*R*R)/(4*v(i)*J)];
     B1c =[0;(-1*alpha*Fn*R*R)/(v*v*J)+(R*1200)/(J*v*v)];
     B2c(:,:,i)=[((-3/4-0.2)*Fn)/m;
                ((1/4-3/2-0.4)*Fn*R*R)/(v(i)*J)];
%  end
     
%  Bc=[0;
%     R/J];
 Cc=[1 0];
 Dc=0;
sys_ct= ss(Ac,Bc,Cc,Dc);
  sys_dt = c2d(sys_ct,Ts);
%     sys_dt = ss(Ac,Bc,Cc,Dc,Ts);
[A,B,C,D]= ssdata(sys_dt);
   
QN = 0.0025;
RN = 100;
end
%% electronic wedge brake (Multi-Objective Co-Optimization of FlexRay-based Distributed Control Systems)--in Chassis Control Unit/Module ECU
if system=="ewb"
    % states: wedge position, wedge velocity
    % output braking torque
    % input force by dc motor
    Ts = 0.005;
    Ac = [0  1;8395.1 0];
    Bc = [0; 4.0451];
    Cc = [7992 0];
    Dc = [0];
    sys_ct= ss(Ac,Bc,Cc,Dc);
    sys_dt = c2d(sys_ct,Ts);
%     sys_dt = ss(Ac,Bc,Cc,Dc,Ts);
    [A,B,C,D]= ssdata(sys_dt);
     % safety limits
    safex = [-0.15,0;0.15,1000];
    perf = 0.5.*safex;
    % safer region of this system to start from
    ini = 0.9.*safex;
    % for central chi2 FAR < 0.05
    th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
    sensorRange = [0;500000];  % columnwise range of each y
    actuatorRange = [-1500000;1500000];   % columnwise range of each u
%     Q= 0.000015*(C'*C);% blkdiag(1000,500);% [Q_1c_1 Q_12c_1;Q_12c_1' Q_2c_1];
%     Q(2,2)=250000
%     R= 5;% [R_1c_1 zeros(size(A,1),1);zeros(1,size(A,1)) R_2_1];
    proc_dev= 0.001; 
    meas_dev=0.0001;
%     QN = 5000*proc_dev*proc_dev*(B*B');
%     RN = 500*meas_dev*meas_dev; 
%     Q = 10000/max(sensorRange)^2*(C'*C);%0.0000000000015*(C'*C);
%     R = 1000000000/max(actuatorRange)^2;%10000;
    qweight = 1;
    rweight= 1;
%     using lqg ragulator
%     K= [3547.3    37.9];
%     L= [0.0004;0.0545];
    settling_time = 6;% 0.03/Ts; 
%     noisy_zvar= 0.14;
%     noisy_zmean= 0.52;
%     noisy_delta= 1.86;
%     nonatk_zvar= 12.6041;%15.8507
%     nonatk_zmean= 0.6064;
%     nonatk_delta= 0.0292;
    uatkon=[1];   % attack on which u
    yatkon=[1];   % attack on which y
    ref =59940;
end
%% vehicle suspension control (Multi-Objective Co-Optimization of FlexRay-based Distributed Control System)--in CCM ECU
if system=="suspension_control"
%   states: car position, car velocity, suspension load position, suspension velocity
%   input suspension load force
%   output car position
    Ts = 0.04;
    Ac = [0 1 0 0;-8 -4 8 4;0 0 0 1;80 40 -160 -60];
    Bc = [0;80;20;1120];
    Cc = [1,0,0,0];
    Dc = [0];    
    sys_ct= ss(Ac,Bc,Cc,Dc);
    sys_dt= c2d(sys_ct,Ts);
    [A,B,C,D]= ssdata(sys_dt);  
    % safery margin
    safex = [-20,-200,-100,-600;20,200,100,600]%[-15,-15,-20,-20;15,15,20,20];
    % safer region of this system to start from
    ini = 0.5.*safex;
    perf = 0.1.*safex;
    % for central chi2 FAR < 0.05
    th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
    sensorRange = [-20;20];  % columnwise range of each y
    actuatorRange = [-1000;1000];   % columnwise range of each u
    qweight= 1000;
    rweight = 0.00001;
    Q= qweight*(C'*C);
    Q(2,2)=10;Q(3,3)=1;Q(4,4)=10;
    R= rweight;
    proc_dev= 0.01; meas_dev=0.001;
    QN = 0.005*(B*B');
    RN = 0.00005;
%     % using lqg ragulator
%     K=[7.6110    0.3412    0.0186    0.0157];
%     L= [0.1298; 0.1642; 0.1312; -0.0622];
    settlingTime = 0.2800/Ts;
%     s.noisy_zvar= 2.6328;
%     s.noisy_zmean= 0.2719;
%     s.noisy_delta= 1.86;
%     s.nonatk_zvar= 12.6041;%15.8507
%     s.nonatk_zmean= 0.6064;
%     s.nonatk_delta= 0.0292;
    s.uatkon=[1];   % attack on which u
    s.yatkon=[1];   % attack on which y
end
%% flexible driveline (patterson_Driveline Modeling and Control, Balau Andreea_OVERALL POWERTRAIN MODELING AND CONTROL BASED ON DRIVELINE SUBSYSTEMS INTEGRATION)-- in ECM ECU
if system=="driveline_mngmnt" % linearized driveline
% theta= rotation angle, J= inertia, r= radius, c= rolling co-eff,
% _m= engine, _w= wheel, _f= final drive, _t= transmission

% states: drive shaft torsion angle difference, engine angular velocity, wheel angular velocity
% input: differnce between driving and friction torque i.e. effective engine torque (M_m-M_mfriction)
% output: wheel angular velocity
i_t= 3.5; % gearbox ratio(1st gear)
i_f= 3.7; % final drive ratio
J_m= 0.17; % (kg.m^2) engine inertia
J_t= 0.014; % (kg.m^2) transmission inertia
J_f= 0.031; % (kg.m^2) final drive inertia
J_w= 1; % (kg.m^2) wheel inertia
c_r1= 0.01; %(Nm/kg) rolling co-eff
c_w= 0.3; % 0.6 % (rad^-2) airdrag co-eff
c_r2= 0.36; % (Nms/rad) approx. co-eff
FA= 2.7; %9; % (m^2) front area
rho= 1.2; % (kg/m^3) air density
c= 65;% (Nms/rad) internal dampling of drive shaft
k= 5000;% (Nm/rad) drive shaft stiffness
b_t= 0.1; % (Nms/rad) viscous damping co-eff causing friction torque at transmission
b_f= 0.1; % (Nms/rad) viscous damping co-eff causing friction torque at final drive
b_w= 0.1; % (Nms/rad) viscous damping co-eff causing friction torque at wheel
r_w= 0.32;% 0.52 %(m) wheel radius
m= 1400;% 24000 %(kg) vehicle mass
g= 9.8; % (m/s^2) gravitational acceleration
alpha= 0; % (radian) road slope angle
load= r_w*m*(c_r1+g*sin(alpha)); 
i=i_f+i_t;
J1= J_m + J_t/i_t^2 + J_f/((i_t^2)*(i_f^2));
J2= J_w+m*r_w^2;
b1= b_t/i_t^2 + b_f/((i_t^2)*(i_f^2));
b2= b_w+c_w*FA*rho*r_w^2+m*c_r2*r_w^2;
Ts = 0.05;
Ac = [0, 1/i, -1;
    -k/(i*J1), -(b1+c/i^2)/J1, c/(i*J1);
    k/J2, c/(i*J2),-(c+b2)/J2];
Bc = [0;1/J1;0];
H = [0;0;-1/J2]; % Ax+Bu+H*load
% Bc = [0 0;1/J1 0;0 -1/J2]; % [B H]*[u;load]
Cc = [0,0,1];
Dc = [0];
% Dc = [0,0];
sys_ct =ss(Ac,Bc,Cc,Dc);
sys_dt= c2d(sys_ct,Ts);
[A,B,C,D]= ssdata(sys_dt);
proc_dev= 0.001;
meas_dev=0.0001;
qweight= 1;
rweight = 0.00001;
Q= 1/4500*(C'*C);
Q(2,2) = 1/6400;
R= 1/90000;% 5;
QN = 1/proc_dev^2*(B*B');%70*proc_dev^2*(B*B');
RN = 0.0000021/meas_dev^2;% 2.1*meas_dev^2;
% calculated using lqg regulator
% K= [-602.4541 ,   1.3191  , 25.1815];
% L= [-0.0050; -0.8285; 0.1233];
settling_time = 60; 
% safety limits
safex = [-6.2832,0,0;6.2832,850,50/r_w]; % [2pi, idle,max engine ang vel in [62.83,628.3] ,min max wheel ang vel;50/wheel rad]
% safer region of this system to start from
ini = [0,62.83,0;3.14,600,100];
perf = 0.1.*safex;
% for central chi2 FAR < 0.05
% th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
sensorRange = [-200;200]; % max ang vel of wheel> 50*0.32 % columnwise range of each y
actuatorRange = [-150000;150000];%[-300;300];   % columnwise range of each u
proc_dev= 0.01; meas_dev=0.001;
% s.noisy_zvar= 0.14;
% s.noisy_zmean= 0.52;
% s.noisy_delta= 1.86;
% s.nonatk_zvar= 12.6041;%15.8507
% s.nonatk_zmean= 0.6064;
% s.nonatk_delta= 0.0292;
s.uatkon=[1];   % attack on which u
s.yatkon=[1];   % attack on which y
end

%% driveline + clutch (Modeling and Control of an Engine Fuel Injection System)
%{
if system=="driveline_clutch_mngmnt" % linearized clutch+driveline
% states:
J1= J_m;
J2= J_t+J_f/i_f^2;
J3= J_w+m*r_w^2;
b2= b_t+b_f/i_f^2;
b3= b_w+c
Ts = 0.02;
A = [0,0,1,-i_t,0;
    0,0,0,1/i_f;-1;
    -k_c/J1,0,-cc/J1,cc*i_t/J1,0;
    k_c*i_t/J2,-k_d/(i_f*J2),cc*i_t/J2,-(cc*i_t*i_t+b2+cd/(i_f*i_f))/J2,cd/(i_f*J2);
    0,kd/J3,0,cd/(i_f*J3),-(b3+cd)/J3];
B = [0 0;0 0;1/J1 0;0 0;0 -1/J2];
C = [0,0,0,0,1];
D = [0,0];
Q= 0.1*eye(size(A,2));
R= 500*eye(size(B,2));
[K,S,E] = dlqr(A,B,Q,R);
QN = 1500;
RN = eye(1);
sys_ss = ss(A-B*K,B,C,D,Ts);
[kalmf,L,P,M] = kalman(sys_ss,QN,RN);
safex = [4,100,100];
% safer region of this system to start from
ini = 1;
% from perfReg.py with this system
perf = [-1.67,-1.47];
% for central chi2 FAR < 0.05
th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
sensorRange = [4];  % columnwise range of each y
actuatorRange = [36];   % columnwise range of each u
proc_dev= 1; meas_dev=0.01;
% s.settlingTime = 13; Ts = 0.01;
% from system_with_noise.m with this system
s.noisy_zvar= 0.14;
% from system_with_noise.m with this system
s.noisy_zmean= 0.52;
s.noisy_delta= 1.86;
s.nonatk_zvar= 12.6041;%15.8507
s.nonatk_zmean= 0.6064;
s.nonatk_delta= 0.0292;
s.uatkon=[1];   % attack on which u
s.yatkon=[1];   % attack on which y
end
%}
%% fuel_injection(Modeling and Control of an Engine Fuel Injection System)--in ECM ECU
if system=="fuel_injection"
% states: 
% inputs: inlet valve air flow, combustion torque
% output: AFR
Ts = 0.01;
A = [0.18734,0.13306,0.10468;
    0.08183,0.78614,-0.54529;
    -0.00054 0.10877,0.26882];
B = [0.00516,-0.0172;
    -0.00073,0.09841;
    -0.00011,0.13589];
C = [158.16,8.4277,-0.44246];
D = [0,0];
Q= blkdiag(0.1,0.1,1);%blkdiag(1,2.25,25);
R= blkdiag(1,500);
Pr= care(A,B,Q);
K = -inv(R)*B'*Pr;
qweight= 1;
rweight = 1;
proc_dev= 0.001; meas_dev=0.0001;
QN = eye(size(B,1));%proc_dev^2*(B*B');
RN = 1;%meas_dev^2;
safex = [-0.22,-1.5,-5;0.22,1.5,5];
% initial region of this system to safely start from
ini = 0.7.*safex;
perf = 0.1.*safex;
% for central chi2 FAR < 0.05
% th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
sensorRange = [-80;80];  % columnwise range of each y
actuatorRange = [-20,-20;20,20];   % columnwise range of each u
% ref= 14.7;
% F= [1; -1.5];
settling_time = 6; 
% noisy_zvar= 0.14;
% noisy_zmean= 0.52;
% noisy_delta= 1.86;
% nonatk_zvar= 12.6041;%15.8507
% nonatk_zmean= 0.6064;
% nonatk_delta= 0.0292;
uatkon=[1];   % attack on which u
yatkon=[1];   % attack on which y
end

%% for rl work comparative study
%{
%% quadrotor(Incomplete..Thesis_KTH-Francesco_Sabatino,data from Stabilization and Control of Unmanned Quadcopter by Thomas Jirinec)
if system=="quadrotor"
    Ts = 0.02;
%     A1 = [0,1,0,0;0,0,1,0;0,0,0,1;0,0,0,0];
%     A2 = [0,1;0,0];
%     A=blkdiag(A1,A1,A1,A2);
%     B1=zeros(4,4);
%     B1(4,1)=1;
%     B2=zeros(4,4);
%     B2(4,2)=1;
%     B3=zeros(4,4);
%     B1(4,3)=1;
%     B4=zeros(2,4);
%     B = [B1;B2;B3;B4];
%     C1 = [1;0;0;0];
%     C2 = [1;0];
%     C = blkdiag(C1',C1',C1',C2');
%     D = zeros(size(C,1),size(B,2));
    I_x= 0.0093; % kg m2
    I_y= 0.0092;
    I_z= 0.0151;
    m= 0.45; % kg
    A = zeros(12);
    A(1,4)=1;
    A(2,5)=1;
    A(3,6)=1;
    A(7,2)=-9.8; % -g in SI N/kg
    A(8,1)=9.8; % g in SI N/kg
    A(10,7)=1;
    A(11,8)=1;
    A(12,9)=1;
    BB= zeros(12,4);
    BB(4,2)=1/I_x;
    BB(5,3)=1/I_y;
    BB(6,4)=1/I_z;
    BB(9,1)=1/m;
    DD= zeros(12,6);
    DD(7,1)=1/m;
    DD(8,2)= 1/m;
    DD(1:12,3:6)=BB;
    B= [BB DD];
    C = eye(size(A));
    D = zeros(size(C,1),size(B,2));
    Q= eye(size(A,2));
    R= 0.01*eye(size(B,2));
%     [K,S,E] = dlqr(A,B,Q,R);
    QN = 1500;
    RN = eye(1);
%     sys_ss = ss(A-B*K,B,C,D,Ts);
%     [kalmf,L,P,M] = kalman(sys_ss,QN,RN);
    safex = [4,100,100];
    perf = [-1.67,-1.47];
    % safer region of this system to start from
    ini = perf;
    % for central chi2 FAR < 0.05
    th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
    sensorRange = [4];  % columnwise range of each y
    actuatorRange = [36];   % columnwise range of each u
    proc_dev= 1; meas_dev=0.01;
    settlingTime = 13; 
    % from system_with_noise.m with this system
    noisy_zvar= 0.14;
    % from system_with_noise.m with this system
    noisy_zmean= 0.52;
    noisy_delta= 1.86;
    nonatk_zvar= 12.6041;%15.8507
    nonatk_zmean= 0.6064;
    nonatk_delta= 0.0292;
    uatkon=[1];   % attack on which u
    yatkon=[1];   % attack on which y
end
%% dcmotor position(2020_RTSS_Real-Time Attack-Recovery for Cyber-Physical Systems Using Linear Approximations)
if system=="dcmotor_pos"
    % states: rotational ang., angular vel., armature current
    % output rotational angle
    % input armature voltage
    Ts = 0.1;
    % ctms
    J = 3.2284E-6;
    b = 3.5077E-6;
    KK = 0.0274;
    RR = 4;
    LL = 2.75E-6;

%     J = 0.01;
%     b = 0.1;
%     KK = 0.01;
%     RR = 1;
%     LL = 0.5;
    Ac = [0 1 0;
        0 -b/J KK/J;
        0 -KK/LL -RR/LL];
    Bc = [0; 0; 1/LL];
    Cc = [1 0 0];
    Dc = [0];
    
    % discretize
    [A,B,C,D]=ssdata(c2d(ss(Ac,Bc,Cc,Dc),Ts));
    
    Q= 10*eye(size(A,2));
    R= 0.01*eye(size(B,2));
    [K,S,E] = dlqr(A,B,Q,R);
    K
    proc_dev= 0.01; 
    meas_dev=0.001;
    QN = proc_dev*proc_dev;
    RN = meas_dev*meas_dev;
    sys_ss = ss(A-B*K,B,C,D,Ts);
    [kalmf,L,P,M] = kalman(sys_ss,QN,RN);
    safex = [-4,-20,0;4,20,20];
    % from perfReg.py with this system
    perf = [-1.67,-1.67,-1.67;-1.47,-1.47,-1.47];
    % safer region of this system to start from
    ini = perf;
    % for central chi2 FAR < 0.05
    th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
    sensorRange = [-4;4];  % columnwise range of each y
    actuatorRange = [-5;5];   % columnwise range of each u
    settlingTime = 13; 
    % from system_with_noise.m with this system
    noisy_zvar= 0.14;
    % from system_with_noise.m with this system
    noisy_zmean= 0.52;
    noisy_delta= 1.86;
    nonatk_zvar= 12.6041;%15.8507
    nonatk_zmean= 0.6064;
    nonatk_delta= 0.0292;
    uatkon=[1];   % attack on which u
    yatkon=[1];   % attack on which y
end
%% aircraft(ctms/Real-Time Attack-Recovery for Cyber-Physical Systems Using Linear Approximations)
if system=="aircraft"
    % states: attack angle, pitch rate, pitch angle
    % output pitch angle
    % input angle of deflection
    A = [-0.313 56.7 0;-0.0139 -0.426 0;0 56.7 0];
    B = [0.232;0.0203;0];
    C = [0 0 1];
    D = zeros(size(C,1),size(B,2));
    Ts = 0.02;  
        % safery constraints
    safex = [0 0 0; 2 2 2];
    % performance region of this system
    perf = [0.68 0.68 0.68;0.72 0.72 0.72];
    % safer region of this system to start from
    ini = 0.5*safex;
    % for central chi2 FAR < 0.05
    th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
    sensorRange = [-3;3];  % columnwise range of each y
    % columnwise range of each u
    actuatorRange = [-0.4363 ;0.4363 ];   % from ctms control tab
    Q= blkdiag(1/2,1/2,1/2);%6.8*eye(size(C,1));
    R= 1/min(abs(actuatorRange))^2;
%     [K,S,E] = dlqr(A,B,Q,R);
%     K
    proc_dev= 0.001;
    meas_dev= 0.0001;
    QN = 1/(proc_dev*proc_dev)*(B*B');
    RN = 1/(meas_dev*meas_dev);
%     sys_ss = ss(A-B*K,B,C,D,Ts);
%     [kalmf,L,P,M] = kalman(sys_ss,QN,RN);

    settling_time = 13; 
    noisy_zvar= 0.14;
    noisy_zmean= 0.52;
    noisy_delta= 1.86;
    nonatk_zvar= 12.6041;%15.8507
    nonatk_zmean= 0.6064;
    nonatk_delta= 0.0292;
    uatkon=[1];   % attack on which u
    yatkon=[1];   % attack on which y 
end
%}

%% control, estimator gain calc
sys_dt= ss(A,B,C,D,Ts);
xdim=size(A,1);
udim=size(B,2);
ydim=size(C,1);
if ~exist('K','var') || ~exist('L','var')
    if ~exist('Q','var') || ~exist('R','var')
        R = rweight.*diag(max(abs(actuatorRange)).^-2);
        Q = qweight*diag(max(abs(safex)).^-2);
    end
    N = zeros(xdim,udim);
    for i1=1:xdim
        for i2=1:udim
            umx = max(abs(actuatorRange));
            xmx = max(abs(safex));
            N(i1,i2)= 1/(xmx(i1)*umx(i2));
        end
    end
    if ~exist('QN','var') || ~exist('RN','var')
        QN = eye(size(B,1));%proc_dev^2*(B*B');
        RN = 1;%meas_dev^2;
    end
    [reg_ss,gain_mat]=lqg(sys_dt,blkdiag(Q,R),blkdiag(QN,RN));
    [reg_ss,gain_mat]=lqg(sys_dt,[Q,N;N',R],blkdiag(QN,RN));
    if ~exist('K','var')
        K= gain_mat.Kx
    end
    if ~exist('L','var')
        L=gain_mat.L   
    end
end
sys_ss = ss(A-B*K,B,C,D,Ts);
% [kalmf,L,P,M] = kalman(sys_ss,proc_dev^2,meas_dev^2);

%% feed-forward gain, settling time calc from step response
[step_out,step_t,step_x,step_ysd]= step(sys_ss);
F_sim= 1/step_out(end);
F_inv= C*((eye(size(A))-(A-B*K))\B);
if ~exist('F','var')
    if size(F_inv,1)==size(F_inv,2)
        F = -inv(F_inv);
    else
        F= -F_sim;
    end
end
if ~exist('ref','var')
    ref = C*(0.5.*(perf(2,:)+perf(1,:))');
end

settling_time = stepinfo(sys_ss).SettlingTime;
%% plot step response
figure;
title("step response")
hold on;
% plot(step_t,step_out);
stepplot(sys_ss);
hold off;
legend;

%% init 
% rng shuffle;
% x=(2*ini*safex*rand(xdim)-safex*ini)';
for k=1:xdim
%    x(k,1) = (2*ini(k)*rand-ini(k))';
        x(k,1) = ini(2,k);% max 
%         x(k,1) = ini(1,k);% min
%         x(k,1) =0;
        
end  
x0 = x %[0.0976827903;0.1724794346];
y = C*x;
z = zeros(xdim,1);
% u = -K*z;
u= -max(min(K*z+F*ref,actuatorRange(2,:)'),actuatorRange(1,:)');
max_x = abs(x);
max_u = abs(u);
state=zeros(xdim,time+1);
state(:,1)= x;
state_est=zeros(xdim,time+1);
out = zeros(ydim,time+1);
out(:,1)= y;
res = zeros(ydim,time+1);
est_err = zeros(xdim,time+1);
inp = zeros(udim,time+1);
inp(:,1) = u;

%% simulation
for i=2:time+1
%    y = max(min(C*x + meas_dev*(2.*ones(size(C,1),1)-ones(size(C,1),1)),sensorRange(2,:)'),sensorRange(1,:)'); % limiting sensor data, max noise 
% y = max(min(C*x + mvnrnd(0,meas_dev,1))); % meas_dev*(2.*ones(size(C,1),1)-ones(size(C,1),1)),sensorRange(2,:)'),sensorRange(1,:)'); % limiting sensor data, max noise 
      y = C*x + meas_dev*(2.*ones(size(C,1),1)-eye(size(C,1),1)); % max bounded noise
%    y = C*x + mvnrnd(0,meas_dev,ydim);% meas_dev*(2.*rand(size(C,1),1)-eye(size(C,1),1)); % random stochastic/bounded noise
   r = y - C*z;
   if system == "driveline_mngmnt"
        z = A*z + B*u + L*r + H*load;
   else
        z = A*z + B*u + L*r;
   end
   if system == "driveline_mngmnt"
        x = A*x + B*u + proc_dev*(2.*ones(size(A,1),1)-ones(size(A,1),1))+H*load;% max noise (+H*load for driveline)
    %    x = A*x + B*u + mvnrnd(0,proc_dev,xdim);% proc_dev*(2.*rand(size(C,1),1)-eye(size(C,1),1));% random stochastic/bounded noise
   else
       x = A*x + B*u + proc_dev*(2.*ones(size(A,1),1)-ones(size(A,1),1));%+H*load;% max noise (+H*load for driveline)
    %    u = -max(min(K*z+F*ref,actuatorRange(2,:)'),actuatorRange(1,:)'); % limiting actuation and adding F*ref to track o/p reference
   end
   u= -K*z-F*ref;
   for j=1:xdim
       if abs(x(j))>max_x(j)
           max_x(j) = abs(x(j));
       end
   end
   for jj=1:udim
       if abs(u(jj))>max_u(jj)
           max_u(jj) = abs(u(jj));
       end
   end   
   
   res(:,i)=r;
   est_err(:,i)=(x-z);
   state(:,i) = x;
   state_est(:,i) = z;
   out(:,i) = y;
   inp(:,i) = u;
   safe_up(:,i)= safex(2,:)';
   safe_dwn(:,i)= safex(1,:)';
   actLim_up(:,i)= actuatorRange(2,:)';
   actLim_dwn(:,i)= actuatorRange(1,:)';
   sensLim_up(:,i)= sensorRange(2,:)';
   sensLim_dwn(:,i)= sensorRange(1,:)';
end

%% plotting
for j=1:xdim
    figure
    title("state,inputs plot");
    hold on;
    plot(state(j,:)','b-');
    plot(state_est(j,:)','b-*');
    plot(safe_up(j,:)','r-.');plot(safe_dwn(j,:)','r-.');
    legendState=strcat({'x','z','upper safety','lower safety'},{num2str(j),num2str(j),num2str(j),num2str(j)}); 
    for jj=1:udim
        plot(inp(jj,:)','-g');
        plot(actLim_up(jj,:)','-.g');plot(actLim_dwn(jj,:)','-.g');
        legendCtrl=strcat({'u','upper actuator limit', 'lower actuator limit'},{num2str(jj),num2str(jj),num2str(jj)});
        axis([1 time min([safex(1,j),actuatorRange(1,jj),-max_x(j),-max_u(jj)]) max([safex(2,j),actuatorRange(2,jj),max_x(j),max_u(jj)])]);
    end
    legend([legendState legendCtrl]);
    hold off;
end
% output/ref plot
figure;
title("reference (*) tracking: output(blue) vs time,limit (green)");
for kk=1:ydim
    hold on;
    plot(out(kk,:)','-b');
    plot(ref(kk,1)*ones(size(out)),'*-b');
    plot(sensLim_up(kk,:)','-.g');plot(sensLim_dwn(kk,:)','-.g');
    % legend(['outputs''references','sensor limits']);
    hold off;
end
%% some data

if exist('res_cov','var')
    res_cov = max(res_cov,cov(res));
else
    res_cov = cov(res);
end
if exist('res_mean','var')
    res_mean = min(res_mean,mean(res));
else
    res_mean = mean(res);
end
chi_sq = mean(res)*inv(cov(res))*mean(res)';

% % to put in python code
% system, safex,ini,sensorRange,actuatorRange, proc_dev, meas_dev, ...
%     Q,R,QN,RN,settling_time,Ts,A,B,C,D,K,L,F,ref
% 
% %% cusum threshold Calc [from Characterization of a CUSUM Model-Based Sensor Attack Detector by ruth]
% false_alarm_rate = 1;   % Maximum false alarm rate
% uth = 1000;             % Upper bound on threshold
% lth = 1e-5;             % Lower bound on threshold
% hysteresis = 2;         % Minimum false alarm rate = Maximum/hysteresis
% sigma = sqrt(var(res))               % Standard deviation of residues
% b = sigma*(sqrt(2/pi))*ones(size(r))      % Compute bias
% th = FindThreshold(time, b, sigma, false_alarm_rate, uth, lth, hysteresis)    % Compute threshold
% rate = fRate(time, th, sigma, b)              % Print false alarm rate for given parameters