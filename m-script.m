clc
% Manipulator
%Pillar parameters
Pillar.c =  [0 0; 0.1 0; 0.29 0.04; 0.29 0.24; 0.1 0.34;-0.07 1.38;
    -0.27 1.38; -0.27 1.18; -0.1 0];
Pillar.l = 0.2;
Pillar.m = 110;
Pillar.cm = [-0.07, 0.56, 0];
Pillar.rgb = [0, 0, 0];

%Lift parameters
Lift.c = [0.1 -0.1; 0.2 0; 0.3 -0.1; 0.5 -0.1; 0.6 0; 3.69 0.182; 3.69 0.382;
    2.228 0.382; 2.128 0.65; 1.928 0.65; 1.828 0.382; 0.2 0.4; -0.1 0.1; -0.1 -0.1;];
Lift.l = 0.2;
Lift.m = 370;
Lift.cm = [1.4, 0.25, 0];
Lift.rgb = [0, 0, 0];
Lift.j = [-0.18, 1.274, 0]; 

%Piston2 parameters
Piston2.r = 0.035;
Piston2.l = 0.92;
Piston2.m = 25;
Piston2.cm = [0, 0, 0];
Piston2.rgb = [0.8, 0.8, 0.8];
Piston2.pos = [0.19, 0.14, 0];

% Cylinder2 parameters
Cylinder2.r = 0.06;
Cylinder2.l = 0.92;
Cylinder2.m = 72;
Cylinder2.cm = [0, 0, 0];
Cylinder2.rgb = [0.2, 0.2, 0.2];
Cylinder2.pos = [0.4, 0.016, 0];

%Link1 parameters
Link1.j = [3.43 0.295 0];
Link1.c = [-0.05 -0.05;0.43 -0.05;0.43 0.05;-0.05 0.05];% ok
Link1.l = 0.13;
Link1.m = 15; 
Link1.cm = [3.215, 0.295, 0];
Link1.rgb = [0.15, 0.15, 0.5];

%Link2 parameters
Link2.j = [0.43 0 0];
Link2.c = [-0.05 -0.05;0.452 -0.05;0.452 0.05;-0.05 0.05];
Link2.l = 0.13;

%Tilt parameters
Tilt.j = [3.59 0.282 0];
Tilt2.j = [0.452 0 0];
Tilt3.j = [-0.075 0.24 0];
Tilt.c = [-1.35 0.3; -0.075 0.24; 0 0; 0.6 0.23 ; 1.92 0.22 ; 1.8 0.638 ;
    -0.687 0.668; -0.737 0.708 ; -0.787 0.678; -1.3 0.688];
Tilt.l = 0.13;
Tilt.m = 170;
Tilt.cm = [0.5 0.45 0];
Tilt.rgb = [0, 0, 0];

% Cylinder3 parameters
Cylinder3.j = [2.028 0.550 0];
Cylinder3.l = 1.068;
Cylinder3.r = 0.0575;
Cylinder3.m = 65;
Cylinder3.cm = [0, 0, 0];

%Piston3 parameters
Piston3.l = 1.068;
Piston3.r = 0.03;
Piston3.m = 20;
Piston3.cm = [0, 0, 0];
Piston3.rgb = [0.8, 0.8, 0.8];

Piston31.pos = [0.43, 0, 0];
Piston32.pos = [0.452, 0, 0];

% Extention 1 parameters
Extention1.c = [-2.32 -0.12;0.177 -0.12;0.177 0.12;0.085 0.265;
    0.027 0.245; 0.088 0.12;-2.32 .12];
Extention1.l = 0.1;
Extention1.m = 90;
Extention1.cm = [-1.215, 0, 0];
Extention1.rgb = [1, 1, 0];
Extention1.j = [1.87, 0.468, 0];

%Extention 2 parameters
Extention2.j = [0.177, 0, 0];
Extention2.c = [-2.32 -0.07;0.199 -0.07;0.199 0.07;0.085 0.265;
    0.027 0.245; 0.088 0.07;-2.32 .07];
Extention2.l = 0.1;
Extention2.m = 90;
Extention2.cm = [-1.215, 0, 0];
Extention2.rgb = [0, 0, 0];

% Cylinder4 parameters
Cylinder4.pos = [-0.737 0.708 0];
Cylinder4.l = 2.640;
Cylinder4.r = 0.035;
Cylinder4.m = 50;
Cylinder4.cm = [0, 0, 0];
Cylinder4.rgb = [0.2, 0.2, 0.2];

% Piston4 parameters
Piston4.r = 0.0225;
Piston4.l = 2.64;
Piston4.m = 20;
Piston4.cm = [0, 0, 0];
Piston4.pos = [0.055 0.235 0];

% pinion parameters
Pinion.j = [0 0.525 0];
Pinion.radius = 0.1;
Pinion.thickness = 0.5650;
Pinion.mass = 5;
Pinion.rgb = [0.8, 0.4, 0];

% rack parameters
Rack.length = 0.8;
Rack.height = 0.545;
Rack.thickness = Pinion.thickness;
Rack.mass = 10;
Rack.rgb = [0.2, 0.4, 0.7];

%Fluid Power
%4/3 valve

%Valve Dynamics
%2nd order transfer function
Valve.secondorder_delay = 0.03;
Valve_omega = 40.5;
Valve_eta = 0.989;

%RELATIVE OPENINGS
%Look-up Tabel PA
Valve.lookupopenPA = [5.86e-4 5.86e-4 5.86e-4 0.0329 0.1003 0.1600 0.6635 1];
Valve.lookupspoolPA = [-1 -0.5 0 0.2 0.4 0.5 0.8 1];

%Look-up Tabel PB
Valve.lookupopenPB = [1 0.4951 0.1599 0.0997 0.0324 5.86e-4 5.86e-4 5.86e-4];
Valve.lookupspoolPB = [-1 -0.7 -0.5 -0.4 -0.2 0 0.5 1];

%Look-up Tabel BT
Valve.lookupopenBT = [0.00115 0.00115 0.00115 0.0347 0.1102 0.2088 0.6821 1];
Valve.lookupspoolBT = [-1 -0.5 0 0.2 0.4 0.5 0.8 1];

%Look-up Tabel AT
Valve.lookupopenAT = [1 0.5189 0.1990 0.1118 0.0343 0.00115 0.00115 0.00115];
Valve.lookupspoolAT = [-1 -0.7 -0.5 -0.4 -0.2 0 0.5 1];


%Orifice/Flowpath Parameters
Valve1.QN.PA = 16.5500e-05;             % Nominal flow rate [m3/s]
Valve1.QN.BT = 16.5500e-05;
Valve1.QN.PB = 16.5500e-05;
Valve1.QN.AT = 16.5500e-05;
Valve1.dpN = 0.4e6;                  % Nominal pressure difference [Pa]
Valve1.ptr = 0.1e6;                  % Transition pressure [Pa]

Valve2.QN.PA = 15.8333e-05;             % Nominal flow rate [m3/s]
Valve2.QN.BT = 15.8333e-05;
Valve2.QN.PB = 15.8333e-05;
Valve2.QN.AT = 15.8333e-05;
Valve2.dpN = 0.4e6;                  % Nominal pressure difference [Pa]
Valve2.ptr = 0.1e6;                  % Transition pressure [Pa]

Valve3.QN.PA = 15.8333e-05;             % Nominal flow rate [m3/s]
Valve3.QN.BT = 15.8333e-05;
Valve3.QN.PB = 15.8333e-05;
Valve3.QN.AT = 15.8333e-05;
Valve3.dpN = 0.4e6;                  % Nominal pressure difference [Pa]
Valve3.ptr = 0.1e6;                  % Transition pressure [Pa]

Valve4.QN.PA = 15.8333e-05;             % Nominal flow rate [m3/s]
Valve4.QN.BT = 15.8333e-05;
Valve4.QN.PB = 15.8333e-05;
Valve4.QN.AT = 15.8333e-05;
Valve4.dpN = 0.4e6;                  % Nominal pressure difference [Pa]
Valve4.ptr = 0.1e6;   

%Supply Pressure
pr_Ps = 215e5;               % Pressure [Pa]
                         
% Actuator
%Cylinder Parameters
Cyl.V0A = 0.2e-3;                           % Dead volume at A-side [m3]
Cyl.V0B = 0.2e-3;                           % Dead volume at B-side [m3]
Cyl.B = 1300e6;                             % Effective bulk modulus [Pa]
%Cyl.pa_init = 4450249.10234683;             % [Pa]
%Cyl.pb_init = 5202222.16271062;             % [Pa]
Cyl1.pa_init = 4.4e6;
Cyl1.pb_init = 4.438e6;
Cyl2.pa_init = 2.134e7;
Cyl2.pb_init = 2.201e6;
Cyl3.pa_init = 8.35e6;
Cyl3.pb_init = 1.86e6;
Cyl4.pa_init = 2.94e6;
Cyl4.pb_init = 6.19e6;

%Cylinder1
Cyl1.D = 115e-3;                             % Piston diameter A-side [m]
Cyl1.d = 0;                              % Piston diameter B-side [m]
Cyl1.A_A = pi*(Cyl1.D)^2/4;                   % Piston area A-side [m2]
Cyl1.A_B = pi*((Cyl1.D)^2-(Cyl1.d)^2)/4;       % Piston area B-side area [m2]
Cyl1.xmax = 0.72;                           % Stroke Length [m]
v1=0.1e-3;
Cyl1.m = 100+110+370+170+90+90+15+15+100+72+65+50;
k_end1=(pr_Ps*Cyl1.A_A)/v1;
b_end1=0.5*(k_end1*Cyl1.m)^0.5;
Cyl1.x_min = 0;
%Friction Model
Cyl1_K = 20000;
Cyl1_Fs = 22360;                            % Static friction force [N]
Cyl1_Fc = 20514;                            % Coulombic friction force [N]
Cyl1_b = 40000;                              % Viscous friction coefficient [Ns/m]
Cyl1_vs = 0.8;                              % Parameter related to minimum friction vel. [m/s]


%Cylinder2
Cyl2.D = 120e-3;                             % Piston diameter A-side [m]
Cyl2.d = 70e-3;                              % Piston diameter B-side [m]
Cyl2.A_A = pi*(Cyl2.D)^2/4;                   % Piston area A-side [m2]
Cyl2.A_B = pi*((Cyl2.D)^2-(Cyl2.d)^2)/4;       % Piston area B-side area [m2]
Cyl2.xmax = 0.61;                           % Stroke Length [m]
Cyl2.m = 370+170+90+90+15+15+72+65+50;
v2=0.5e-3;
k_end2 = (pr_Ps*Cyl2.A_A)/v2;
b_end2 = 0.5*(k_end2*Cyl2.m)^0.5;
Cyl2.x_min = 0.920;
%Friction Model
Cyl2_K = 20000;
Cyl2_Fs = 16125;                            % Static friction force [N]
Cyl2_Fc = 14950;                            % Coulombic friction force [N]
Cyl2_b = 30000;                              % Viscous friction coefficient [Ns/m]
Cyl2_vs = 0.8;                              % Parameter related to minimum friction vel. [m/s]

%Cylinder3
Cyl3.D = 115e-3;                             % Piston diameter A-side [m]
Cyl3.d = 60e-3;                              % Piston diameter B-side [m]
Cyl3.A_A = pi*(Cyl3.D)^2/4;                   % Piston area A-side [m2]
Cyl3.A_B = pi*((Cyl3.D)^2-(Cyl3.d)^2)/4;       % Piston area B-side area [m2]
Cyl3.xmax = 0.725;                           % Stroke Length [m]
Cyl3.m = 170+90+90+15+15+65+50;
v3 = 0.5e-3;
k_end3 = (pr_Ps*Cyl3.A_A)/v3;
b_end3 = 0.5*(k_end3*Cyl3.m)^0.5;
Cyl3.x_min = 1.068;
%Friction Model
Cyl3_K = 20000;
Cyl3_Fs = 16340;                            % Static friction force [N]
Cyl3_Fc = 14996;                            % Coulombic friction force [N]
Cyl3_b = 30000;                              % Viscous friction coefficient [Ns/m]
Cyl3_vs = 0.8;                              % Parameter related to minimum friction vel. [m/s]

%Cylinder4
Cyl4.D = 70e-3;                             % Piston diameter A-side [m]
Cyl4.d = 45e-3;                              % Piston diameter B-side [m]
Cyl4.A_A = pi*(Cyl4.D)^2/4;                   % Piston area A-side [m2]
Cyl4.A_B = pi*((Cyl4.D)^2-(Cyl4.d)^2)/4;       % Piston area B-side area [m2]

Cyl4.xmax = 1.850;                           % Stroke Length [m]
Cyl4.m = 90+90+50;
v4 = 5e-3;
k_end4 = (pr_Ps*Cyl4.A_A)/v4;
b_end4 = 0.5*(k_end4*Cyl4.m)^0.5;
Cyl4.x_min = 2.640;
%Friction Model
Cyl4_K = 20000;
Cyl4_Fs = 4945;                            % Static friction force [N]
Cyl4_Fc = 4570;                            % Coulombic friction force [N]
Cyl4_b = 8500;                              % Viscous friction coefficient [Ns/m]
Cyl4_vs = 0.8;                              % Parameter related to minimum friction vel. [m/s]

sampleTime = 0.001;
