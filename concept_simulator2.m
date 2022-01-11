function [x_end,v_end,v_average,v_terminal,a_end,t_end,n,m] = concept_simulator2(r,m,n,I,gt_eff,torque_bearing_loss,torque_rolling_resistance,n_batteries,inclination,time_step,Ff)
% Subteam 1A: Anmol Gill, Christine de Guzman, Morteza Rahimi-Mediseh, 
%             Muhammad Muztahidul Hakim Zareer
% Inputs: r - radius of spool
%         m - mass of body
%         n - gear ratio
%         omega_data - array of omega values from motor graph
%         torque_data - array of torque values from motor graph
%         inclination - angle of inclination of cable with respect to the
%                       horizontal
%         miu - coefficient of rolling resistance
%         time_step - time increments for which calculations are carried out
%         F - friction between cable and spool
% Outputs: x_end - position of body at finishing end of cable
%          v_end - velocity of body at finishing end of cable
%          a_end - acceleration of body at finishing end of cable
%          Graphs of x vs t, v vs t, and a vs t

% Create an array of time values
t = 0:time_step:120;
% Index for the for loop
i_max = length(t);

% create array to store values of acceleration, velocity, position,
% torque applied on spool, angular velocity and angular acceleration of
% spool
a = zeros(1,i_max);
v = zeros(1,i_max);
x = zeros(1,i_max);
torque = zeros(1,i_max);
omega = zeros(1,i_max);
alpha = zeros(1,i_max);

% Initialize gravitational acceleration, position, anglar velocity, velocity
g = 9.81;
torque_max = n_batteries*1.5/4.5*11.5*10^-3;
omega_max = n_batteries*1.5/4.5*9840*2*pi/60;

x(1) = 0;
omega(1) = 0;


% find max power point (just set it for now)
torque_BEP = torque_max/2;
omega_BEP = omega_max/2;

if n == 0 && m
    % find gear ratio for max power and terminal velocity
    n = (r*m*g*sind(inclination) + torque_bearing_loss + torque_rolling_resistance)/(gt_eff*torque_BEP);
else
    if m == 0 && n
        m = (gt_eff*n*torque_BEP - torque_bearing_loss - torque_rolling_resistance)/(r*g*sind(inclination));
    end
end

v_terminal = omega_BEP*r/n;

% find initial torque through interpolation
torque(1) = torque_max - torque_max/omega_max*omega(1);

% find linear acceleration of device and its angular velocity
v(1) = omega(1)/n*r;
a(1) = (n*gt_eff*torque(1)/r - torque_bearing_loss/r - m*g*sind(inclination) + torque_rolling_resistance)/(m + I/r^2);
alpha(1) = a(1)/r*n;

% initialize indexing variable
i_counter = 1;
for i = 2:i_max
    i_counter = i_counter + 1;
    % calculate new angular velocity after increase in time step
    omega(i) = omega(i-1) + alpha(i-1)*time_step;
    % calculate new velocity after increase in time step
    v(i) = omega(i)/n*r;
    % find new torque through interpolation
    if omega(i) < 0 % if the device is rolling backwards
        torque(i) = torque_max; % torque = stall torque
    else % if moving forwards use motor curve
        torque(i) = torque_max - torque_max/omega_max*omega(i);
    end
    % check if torque applied on spool exceeds that which is allowed by
    % friction
    if torque(i) > Ff*r
        torque(i) = Ff*r;
    end
    % calculate acceleration, angular acceleration and position
    a(i) = (n*gt_eff*torque(i)/r - torque_bearing_loss/r - m*g*sind(inclination) - torque_rolling_resistance)/(m + I/r^2);
    alpha(i) = a(i)/r;
    x(i) = x(i-1) + v(i-1)*time_step + 1/2*a(i-1)*time_step^2;
    % break loop if position exceeds 2.5 meters of cable
    if x(i) >= 2.5
        break
    end
end

% Calculate Scores


% Record values at the end of the cable
a_end = a(i_counter);
v_end = v(i_counter);
x_end = x(i_counter);
t_end = t(i_counter);

v_average = x_end/t_end;

% Plot results
% x vs t graph
subplot(1,3,1)
plot(t,x)
title('x versus t')
xlabel('time (s)')
ylabel('position (m)')
axis square
xlim([0 t_end])

% v vs t graph
subplot(1,3,2)
plot(t,v)
title('v versus t')
xlabel('time (s)')
ylabel('velocity (m/s)')
axis square
xlim([0 t_end])

% a vs t graph
subplot(1,3,3)
plot(t,a)
title('a versus t')
xlabel('time (s)')
ylabel('acceleration (m/s^2)')
axis square
xlim([0 t_end])