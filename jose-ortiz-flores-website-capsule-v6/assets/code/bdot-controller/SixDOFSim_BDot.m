%% B-Dot Detumbling Simulation
% Bang-bang B-dot controller (discrete, 10 Hz)
%
% Magnetorquer:
% GomSpace NanoTorque GST-600 Mk2

close all
clear
clc
clear mex

%% Constants

mu = 398600; % Earth's gravitational parameter [km^3/s^2]
Re = 6378; % Earth radius [km]
h  = 200; % Orbit altitude [km]

RAAN = 30; % [deg]
inc  = 50; % [deg]
TA   = 0;  % [deg]

% Earth's magnetic dipole model
mu0 = 4*pi*1e-7; % [kg*m/(s^2*A^2)]
m_e = 8e22; % Earth's magnetic dipole moment [A*m^2]

R_m = Re*1000; % Earth's radius [m]

B0 = (mu0*m_e)/(4*pi*R_m^3); % [T]


%% Spacecraft

% Spacecraft inertia
I = diag([35.5 33.5 7.9]); % [kg*m^2]


%% GomSpace NanoTorque GST-600 Mk2

% Maximum magnetic dipole moments
m_max = [0.40;0.40;0.50]; % [A*m^2]


%% Controller specs

fc = 10; % Controller frequency [Hz]
dt_control = 1/fc; % Controller period [s]


%% Initial Orbit

r0mag = Re + h; % [km]
v0mag = sqrt(mu/r0mag); % [km/s]

r0_O = [0;0;r0mag];

v0_O = [v0mag;0;0];

% DCM: orbit relative to inertial frame

c1 = cosd(RAAN);
c2 = cosd(inc);
c3 = cosd(TA);

s1 = sind(RAAN);
s2 = sind(inc);
s3 = sind(TA);

ON0 = [c3*c1-s3*c2*s1, c3*s1+s3*c2*c1, s3*s2;...
       -s3*c1-c3*c2*s1, -s3*s1+c3*c2*c1, c3*s2;...
       s2*s1, -s2*c1, c2];

r0_N = ON0'*r0_O;
v0_N = ON0'*v0_O;


%% Orbital Period

Torbit = 2*pi*sqrt(r0mag^3/mu);


%% Initial Attitude

% 3-1-3 Euler angles B relative to N

theta01 = 5; % [deg]
theta02 = 7; % [deg]
theta03 = 4; % [deg]

c1 = cosd(theta01);
c2 = cosd(theta02);
c3 = cosd(theta03);

s1 = sind(theta01);
s2 = sind(theta02);
s3 = sind(theta03);

BN0 = [c3*c1-s3*c2*s1, c3*s1+s3*c2*c1, s3*s2;...
       -s3*c1-c3*c2*s1, -s3*s1+c3*c2*c1, c3*s2;...
       s2*s1, -s2*c1, c2];


%% Initial Quaternion

q4 = 0.5*sqrt(1 + trace(BN0));

q1 = (BN0(2,3)-BN0(3,2))/(4*q4);
q2 = (BN0(3,1)-BN0(1,3))/(4*q4);
q3 = (BN0(1,2)-BN0(2,1))/(4*q4);

q0 = [q1;q2;q3;q4];


%% Initial Tumble Rate

w0 = [0.01;0.01;0.01]; % [rad/s]


%% Initial State

x = [r0_N;v0_N;q0;w0];


%% Simulation Time

tf = 10*Torbit;                 

time = 0:dt_control:tf;

N = length(time);


%% Array initialization

x_hist = zeros(N,13);

B_hist = zeros(N,3);

m_hist = zeros(N,3);
tau_hist = zeros(N,3);


%% Main Simulation Loop

for k = 1:N

    t = time(k);

    % Store current state
    x_hist(k,:) = x';


    %% Current magnetic field measurement

    B_current = magneticFieldBody(x,R_m,B0);

    B_hist(k,:) = B_current';

    %% C++ B-dot controller
    
    m_cmd = bdot_mex(B_current,dt_control);


    %% Magnetorquer torque

    tau_mtq = cross(m_cmd,B_current);


    %% Save controller data

    m_hist(k,:) = m_cmd';
    tau_hist(k,:) = tau_mtq';


    %% Propagate spacecraft until next controller update

    if k < N

        t_interval = [time(k) time(k+1)];

        options = odeset('RelTol',1e-10,'AbsTol',1e-10);

        [~,x_temp] = ode45( ...
            @(t,x) spacecraftDynamics( ...
                t,x,mu,I,m_cmd,R_m,B0), ...
            t_interval,x,options);

        % Take state at end of integration interval
        x = x_temp(end,:)';

        % Normalize quaternion
        x(7:10) = x(7:10)/norm(x(7:10));

    end

end


%% Extract Results

r = x_hist(:,1:3);
v = x_hist(:,4:6);
q = x_hist(:,7:10);
w = x_hist(:,11:13);

w_mag = vecnorm(w,2,2);


%% Plot Angular Velocity

figure('Name','Angular Velocity','Color','w')

plot(time/60,rad2deg(w),'LineWidth',1.5)

grid on

xlabel('Time [min]')
ylabel('Angular velocity [deg/s]')

title('Spacecraft Angular Velocity')

legend('\omega_x','\omega_y','\omega_z')


%% Angular Velocity Magnitude

figure('Name','Detumbling Performance','Color','w')

plot(time/60,rad2deg(w_mag),'LineWidth',1.8)

grid on

xlabel('Time [min]')
ylabel('||\omega|| [deg/s]')

title('B-Dot Detumbling Performance')


%% Magnetic Field

figure('Name','Body Magnetic Field','Color','w')

plot(time/60,B_hist*1e6,'LineWidth',1.3)

grid on

xlabel('Time [min]')
ylabel('Magnetic field [\muT]')

title('Magnetic Field in Body Frame')

legend('B_x','B_y','B_z')


%% Magnetorquer Commands

figure('Name','Magnetorquer Commands','Color','w')

plot(time/60,m_hist,'LineWidth',1.2)

grid on

xlabel('Time [min]')
ylabel('Magnetic dipole [A m^2]')

title('Bang-Bang Magnetorquer Commands')

legend('m_x','m_y','m_z')


%% Magnetic Control Torque

figure('Name','Magnetic Torque','Color','w')

plot(time/60,tau_hist,'LineWidth',1.2)

grid on

xlabel('Time [min]')
ylabel('Torque [N m]')

title('Magnetorquer Control Torque')

legend('\tau_x','\tau_y','\tau_z')



%% SPACECRAFT DYNAMICS

function dx = spacecraftDynamics(t,x,mu,I,m_cmd,R_m,B0)

    %% States

    r = x(1:3); % position N-frame [km]
    v = x(4:6); % velocity N-frame [km/s]

    q = x(7:10); % q_BN

    w = x(11:13); % omega_BN in B-frame [rad/s]


    %% Normalize quaternion

    q = q/norm(q);


    %% Orbit Dynamics

    rmag = norm(r);

    a = -mu*r/rmag^3;

    %% Quaternion -> DCM

    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    q4 = q(4);

    BN = [q1^2-q2^2-q3^2+q4^2, 2*(q1*q2+q3*q4), 2*(q1*q3-q2*q4);...
        2*(q1*q2-q3*q4), -q1^2+q2^2-q3^2+q4^2, 2*(q2*q3+q1*q4);...
        2*(q1*q3+q2*q4), 2*(q2*q3-q1*q4), -q1^2-q2^2+q3^2+q4^2];


    %% Magnetic Field Dipole Model

    r_m = 1000*r;

    xN = r_m(1);
    yN = r_m(2);
    zN = r_m(3);

    rmag_m = norm(r_m);

    Bx = (3*xN*zN*R_m^3*B0)/(rmag_m^5);

    By = (3*yN*zN*R_m^3*B0)/(rmag_m^5);

    Bz = ((3*zN^2-rmag_m^2)*R_m^3*B0)/(rmag_m^5);

    B_N = [Bx;By;Bz];

    B_B = BN*B_N;


    %% Magnetorquer Torque

    tau_mtq = cross(m_cmd,B_B);


    %% Rotational Dynamics

    wdot = I \ (-cross(w,I*w)+tau_mtq);


    %% Quaternion Kinematics

    Bq = [ q4, -q3,  q2;...
           q3,  q4, -q1;...
          -q2,  q1,  q4;...
          -q1, -q2, -q3];

    qdot = 0.5*Bq*w;


    %% State Derivative

    dx = [v;a;qdot;wdot];

end



%% BODY-FRAME MAGNETIC FIELD DIPOLE MODEL

function B_B = magneticFieldBody(x,R_m,B0)

    r = x(1:3);
    q = x(7:10);
    q = q/norm(q);


    %% Quaternion -> DCM

    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    q4 = q(4);

    BN = [q1^2-q2^2-q3^2+q4^2, 2*(q1*q2+q3*q4), 2*(q1*q3-q2*q4);...
        2*(q1*q2-q3*q4), -q1^2+q2^2-q3^2+q4^2, 2*(q2*q3+q1*q4);...
        2*(q1*q3+q2*q4), 2*(q2*q3-q1*q4), -q1^2-q2^2+q3^2+q4^2];


    %% Earth's magnetic field

    r_m = 1000*r;

    xN = r_m(1);
    yN = r_m(2);
    zN = r_m(3);

    rmag_m = norm(r_m);

    Bx = (3*xN*zN*R_m^3*B0)/(rmag_m^5);

    By = (3*yN*zN*R_m^3*B0)/(rmag_m^5);

    Bz = ((3*zN^2-rmag_m^2)*R_m^3*B0)/(rmag_m^5);

    B_N = [Bx;By;Bz];

    B_B = BN*B_N;

end