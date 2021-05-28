%% ReactionWheelDesign.m
clc; clearvars; format shortEng; close all
% Rhett A. Smith
% Use bang-bang control theory (lowest possible time case) to design a reaction wheel.

%% Known Values
rho_PLA = 1.01E3;                           % density of PLA (kg/m^3)
theta_max = pi;                             % max rotation (rad)
T_rated = 0.0147099;                        % motor rated torque at 12v (N*m)

w_max_wheel = 1000 .* (2*pi/60);            % max unloaded rotational rate (deg/s)

%% Satellite Moment of Inertia
a = 0.095;                                  % length (m)
b = 0.085;                                  % width (m)
M_sat = 0.675;                              % mass of satellite(kg)

I_sat = (M_sat*(a^2+b^2))/12;               % moment of inertia of satellite (kg*m^2)

disp(['Calculated moment of inertia of the sat | I_sat = ',num2str(I_sat,3),' kg*m^2'])

%% Satellite bang-bang simulation
T_actual = T_rated*0.5;                     % approximate load torque {running at half load} (N*m)
alpha_sat = T_actual/I_sat;                 % angular acceleration of satellite {20% loss} (m/s^2)
max_time = 120;                              % maximum time I am willing to let the system run (s)
dt = 0.001;                                 % time step (s)
t = [0:dt:max_time];                        % time vector (s)
theta(1) = 0;                               % starting angle (rad)
w_sat(1) = 0;                               % starting velocity (rad/s)
theta_target = pi;                          % target angle (rad)

% Begin loop
% ~~ Goes either until the angle is met or max_time as passed
for i = 1:length(t)
    if theta(i) <= theta_target/2                           % if the satellite is < theta_target/2, torque up
        T(i) = T_actual;                                    % torque up (N*m)
        w_sat(i) = alpha_sat*t(i);                          % calculate new angular velocity (rad/s)
        t_half = t(i);                                      % time when sat reaches theta_target/2 (s)
    elseif theta(i) >= theta_target/2                       % if the satellite is > theta_target/2, torque down
        T(i) = -T_actual;                                   % torque down (N*m)
        w_sat(i) = max(w_sat) - alpha_sat*(t(i)-t_half);    % calculate new angular velocity (rad/s)
        t_total = t(i);                                     % total time needed (s)
    end
    theta(i+1) = theta(i) + w_sat(i)*dt;                    % assign new theta value (rad)
    if theta(i+1) >= theta_target*0.999                     % it should asymptote just before 180 with this model
        break
    end
end

% Plot
figure('Name','Satellite bang-bang simulation','NumberTitle',1)

subplot(3,1,1)
plot(t(1:length(T)),T)
title('Torque')
xlim([0,t(length(theta))])
ylim([-T_actual-0.003, T_actual+0.003])
xlabel('Time (s)')
ylabel('Torque (N*m)')

subplot(3,1,2)
plot(t(1:length(w_sat)),w_sat*I_sat)
title('Momentum')
xlim([0,t(length(theta))])
xlabel('Time (s)')
ylabel('Momentum (rad*kg*m^2/s)')

subplot(3,1,3)
plot(t(1:length(theta)),theta*180/pi)
title('Angle')
xlim([0,t(length(theta))])
ylim([0,theta_max*180/pi+20])
xlabel('Time (s)')
ylabel('Angle (deg)')

%% Required Moment of Inertia of the Flywheel
H_max = max(w_sat)*I_sat;               % maximum momentum in the system (rad*kg*m^2/s)
I_wheel_req = H_max/(w_max_wheel*0.5);  % moment of inertia of wheel {w_max_wheel assumed to have 50% losses} (kg*m^2)
disp(['Required moment of inertia | I_wheel_req = ',num2str(I_wheel_req,3),' kg*m^2'])

%% Wheel Moment of Inertia 
R1 = 0.040;                             % inner radius (m)
R2 = 0.050;                             % outer radius (m)
h1 = 0.008;                             % height of disk (m)
h2 = 0.018;                             % height of ring (m)

M1 = pi*R1^2*h1*rho_PLA;                % mass of disk (kg)
M2 = pi*h2*(R2^2 - R1^2) * rho_PLA;     % mass of ring (kg)

I_disk = 0.5*M1*R1^2;                   % moment of inertia of disk (kg*m^2)
I_ring = 0.5*M2*(R1^2+R2^2);            % moment of inertia of ring (kg*m^2)

I_wheel_calc = I_disk + I_ring;         % moment of inertia of wheel (kg*m^2)

disp(['Calculated moment of inertia | I_wheel_calc = ',num2str(I_wheel_calc,3),' kg*m^2'])

% CHECK
H_max = I_wheel_calc*w_max_wheel*0.5;   % maximum momentum, accounting for 50% max wheel speed loss
w_max_sat = H_max/I_sat;                % satellite max rotational rate (deg/s)
t_saturation = H_max/T_rated;           % time before the wheel reaches saturation (s)

theta_max = (T_rated*t_saturation^2)/I_sat; % should be greater than 180
