clc; clf; clear; close all;


%% Importing & Initial Conditions
% This is conversion code for thrust, NEED 'reduceTDMS.m' and 'convertTDMS.m' and 'Motor_1.tdms' files in you workspace.
% This code outputs Thust and time matrices
testid = 1;
saveData = true;
dataFileNameMB = 'Motor_1.tdms'; %ENSURE .tdms IS AT END OF FILE
LFmatFilename = sprintf('Test_%d_Data',testid);
LFMB = reduceTDMS(dataFileNameMB,1,nan);
 
% Package and Save
if saveData% && ZeroSave
    % fprintf('Saving Low Frequency Data...\n')
    save([pwd,'\',LFmatFilename],'LFMB')
    % fprintf('Low Frequency Data Saved.\n\n')
end

allData.LFMB = LFMB;
time = LFMB.time.Value;
Thrust = LFMB.cc_lc_02.Value;
indices = find(Thrust >= 0.265);
F = Thrust(indices) * 4.448;

t_rel = time(indices);
tb = t_rel(end)- t_rel(1);
% dt = t_rel(2)-t_rel(1);
t_test = linspace(0,tb,length(t_rel));
thrust_N = Thrust(indices) * 4.448;
F = trapz(t_test,F);

% %% Inital conditions
rho = 1.225;
g = 9.81;
mo = 0.891; % Initial mass kg
mp = 0.0433; % Prop mass kg
mi = 0.0442; % Inert mass kg
mbs = mp/tb; % Mass loss slope
mc = mo-mp;
mD_thrust = mp / F;
mDotArr = mD_thrust .* thrust_N;
mDotTot = trapz(t_test, mDotArr);
% Isp = (thrust_N ./ mDotArr) / g;
An = 0.00486945887412729; % Noze area m^2
A_chute = pi*(0.9144/2)^2;
Aw = 0.050895; % Wing area m^2 (3 wings)
Lr = 1.3716; % Rocket length (m)
m = @(t) max(mo - mbs*t, mo-mp); % Funcion for mass over time
Cd_rocket = .49; % just a relativley reasonable value form the Mach_Cd table colin made
%Cl_rocket = 1.25; % gross assumption we can deal with it later again from colins work what a beast
Cd_chute = 0.73; %actual values is 0.73
Cg = 0.5715; % Center of gravity from base (m) measured by balancing at a point
% Center of pressure calculation
rad = 0.0381; % m
Lb = 1.0922; % m
Ln = 0.2794; % m
db = 0.5461; % m
dn = 1.2319; % m
dw = 0.1397; % m
Ab = 2*pi*rad*Lb; % Body area m^2
As = pi*rad*sqrt((Ln^2)+(rad^2)); % Nose area m^2
At = Ab + As + Aw; % Total Surface Area m^2
Cp = (db*Ab + dn*As + dw*Aw)/At; % Center of pressure from base m

v_term = -sqrt(2*mc*g/(rho*A_chute*Cd_chute));
wind = 3.3; %wind in m/s
%% End Importing & Initial Conditions


%% Main
%% Convergence Plots
[time1,Vx1,Vy1,Vrel1,x_pos1,y_pos1,theta1] = launchSim(90,0.05,thrust_N,mo,mp,mc,g,rho,An,Aw,A_chute,Cd_rocket,Cd_chute,t_test,mDotArr,wind,v_term,Cp,Cg,Lr);
[time2,Vx1,Vy1,Vrel1,x_pos2,y_pos2,theta2] = launchSim(90,0.005,thrust_N,mo,mp,mc,g,rho,An,Aw,A_chute,Cd_rocket,Cd_chute,t_test,mDotArr,wind,v_term,Cp,Cg,Lr);
[time3,Vx1,Vy1,Vrel1,x_pos3,y_pos3,theta3] = launchSim(90,0.001,thrust_N,mo,mp,mc,g,rho,An,Aw,A_chute,Cd_rocket,Cd_chute,t_test,mDotArr,wind,v_term,Cp,Cg,Lr);
[time4,Vx1,Vy1,Vrel1,x_pos4,y_pos4,theta4] = launchSim(90,0.0005,thrust_N,mo,mp,mc,g,rho,An,Aw,A_chute,Cd_rocket,Cd_chute,t_test,mDotArr,wind,v_term,Cp,Cg,Lr);

figure;
plot(x_pos1,y_pos1);
hold on;
plot(x_pos2,y_pos2);
plot(x_pos3,y_pos3);
plot(x_pos4,y_pos4);
title('Position At Varying Timestep');
xlabel('Horizontal Position (m)');
ylabel('Altitude (m)');
legend('dt=0.05','dt=0.005','dt=0.001','dt=0.0005');
hold off;

figure;
plot(time1,theta1);
hold on;
plot(time2,theta2);
plot(time3,theta3);
plot(time4,theta4);
title('Angle At Varying Timestep');
xlabel('Time (s)');
ylabel('Theta (deg)');
legend('dt=0.05','dt=0.005','dt=0.001','dt=0.0005');
hold off;
%% End Convergence Plots


%% Finding Best Angle
wind = wind; % Set to current wind speed m/s
xf = -100;
thetam = 90;
while(xf < 0)
    thetam = thetam - 10;
    [time5,Vx5,Vy5,Vrel5,x_pos5,y_pos5,theta5] = launchSim(thetam,0.005,thrust_N,mo,mp,mc,g,rho,An,Aw,A_chute,Cd_rocket,Cd_chute,t_test,mDotArr,wind,v_term,Cp,Cg,Lr);
    xf = x_pos5(end);
    if(thetam<=0); xf=100; thetam=0;end;
end
xf = -100;
thetam = thetam + 10;
while(xf < 0)
    thetam = thetam - 1;
    [time5,Vx5,Vy5,Vrel5,x_pos5,y_pos5,theta5] = launchSim(thetam,0.005,thrust_N,mo,mp,mc,g,rho,An,Aw,A_chute,Cd_rocket,Cd_chute,t_test,mDotArr,wind,v_term,Cp,Cg,Lr);
    xf = x_pos5(end);
    if(thetam<=0); xf=100; thetam=0;end;
end
thetap = thetam + 1;
distmin = 1000; angbest = 90;
for(j=thetam:0.1:thetap)
    [time5,Vx5,Vy5,Vrel5,x_pos5,y_pos5,theta5] = launchSim(j,0.005,thrust_N,mo,mp,mc,g,rho,An,Aw,A_chute,Cd_rocket,Cd_chute,t_test,mDotArr,wind,v_term,Cp,Cg,Lr);
    if(abs(x_pos5(end))<=distmin)
        distmin = x_pos5(end); angbest = j;
    end
end
fprintf('The best launch angle is theta=%2.1fdeg from horizontal.\n',angbest);
[time5,Vx5,Vy5,Vrel5,x_pos5,y_pos5,theta5] = launchSim(angbest,0.005,thrust_N,mo,mp,mc,g,rho,An,Aw,A_chute,Cd_rocket,Cd_chute,t_test,mDotArr,wind,v_term,Cp,Cg,Lr);
plotTraj(x_pos5,y_pos5,angbest);

%% End Finding Best Angle
%% End Main


%% Launch Simulation
function [time,Vx,Vy,Vrel,x_pos,y_pos,theta] = launchSim(theta0,dt,thrust_N,mo,mp,mc,g,rho,An,Aw,A_chute,Cd_rocket,Cd_chute,t_test,mDotArr,wind,v_term,Cp,Cg,Lr)
    theta(1) = theta0; % Initial angle from horizontal
    time(1) = 0;
    i = 1; 
    thrustStepper = dt * 2000;
    thrustStep = 1;
    Vy(1) = 0;
    Vx(1) = 0;
    y = 0;
    mass(1) = mo;
    y_pos(1) = 0;
    x_pos(1) = 0;
    Fdrag(1) = 0;
    thetad(1) = 0;
    Vrel(1) = 0;
    
    % Launch Rail Phase
    while (y <= 1.8288)
        Stepper_over = thrustStep + thrustStepper;
        Fthrust(i) = (mean(thrust_N(thrustStep:Stepper_over)));
        Fg(i) = -g * mass(i); % Grav Force N
        Fdrag(i) = fDrag(rho,An,Cd_rocket,Vrel(i)); % Drag Force N
        dMass(i) = (trapz(t_test(thrustStep:Stepper_over), mDotArr(thrustStep:Stepper_over))); % mdot kg/s
        mass(i+1) = mass(i) - dMass(i); % Mass at next step Kg
        Fnetx = Fthrust(i)*cosd(theta(i)) + Fdrag(i)*cosd(theta(i)); % Forces in x direction
        Fnety = Fthrust(i)*sind(theta(i)) + Fdrag(i)*sind(theta(i)) + Fg(i); % Forces in y direction
        Vx(i+1) = Vx(i) + (Fnetx/mass(i))*dt; % Velocity in next time step depends on forces now
        Vy(i+1) = Vy(i) + (Fnety/mass(i))*dt; % Velocity in next time step depends on forces now
        Vrel(i+1) = sqrt((Vx(i+1)^2)+(Vy(i+1)^2)); % Relative Velocity m/s
        theta(i+1) = theta0;
        x_pos(i+1) = x_pos(i) + Vrel(i+1)*cosd(theta(i+1))*dt;
        y_pos(i+1) = y_pos(i) + Vrel(i+1)*sind(theta(i+1))*dt;
        y = sqrt((y_pos(i+1)^2)+(x_pos(i+1)^2));
        thrustStep = Stepper_over;
        time(i+1) = time(i) + dt;
        thetad(i+1) = 0;
        i = i + 1;
    end

    % Powered Ascent Phase
    while (Stepper_over < (2640 - thrustStepper))
        Stepper_over = thrustStep + thrustStepper;
        Fthrust(i) = (mean(thrust_N(thrustStep:Stepper_over)));
        Fg(i) = -g * mass(i); % Grav Force N
        Vrel(i) = sqrt(((Vx(i)+wind)^2)+(Vy(i)^2)); % Relative Velocity m/s
        Fdrag(i) = fDrag(rho,An,Cd_rocket,Vrel(i)); % Drag Force N
        dMass(i) = (trapz(t_test(thrustStep:Stepper_over), mDotArr(thrustStep:Stepper_over))); % mdot kg/s
        mass(i+1) = mass(i) - dMass(i); % Mass at next step Kg
        aoa = theta(i) - atand(Vy(i)/(Vx(i)+wind));
        Flift(i) = fLift(rho,Aw,Cl_rocket(aoa),Vrel(i)); % Lift force for torqueing
        Fnetx = Fthrust(i)*cosd(theta(i)) - Flift(i)*sind(theta(i)) + Fdrag(i)*cosd(theta(i)); % Forces in x direction
        Fnety = Fthrust(i)*sind(theta(i)) + Flift(i)*cosd(theta(i)) + Fdrag(i)*sind(theta(i)) + Fg(i); % Forces in y direction
        [theta(i+1),thetad(i+1)] = fTh(Flift(i),dt,mass(i),Lr,Cp,Cg,theta(i),thetad(i)); % Weathercocking
        Vx(i+1) = Vx(i) + (Fnetx/mass(i))*dt; % Velocity in next time step depends on forces now
        Vy(i+1) = Vy(i) + (Fnety/mass(i))*dt; % Velocity in next time step depends on forces now
        x_pos(i+1) = x_pos(i) + Vx(i+1)*dt;
        y_pos(i+1) = y_pos(i) + Vy(i+1)*dt;
        thrustStep = Stepper_over;
        time(i+1) = time(i) + dt;
        i = i + 1;
    end
    
    % Coast Phase
    mass(i) = mc;
    while (time(i) <= 6)
        Fg(i) = -g * mass(i); % Grav Force N
        Vrel(i) = sqrt(((Vx(i)+wind)^2)+(Vy(i)^2)); % Relative Velocity m/s
        Fdrag(i) = fDrag(rho,An,Cd_rocket,Vrel(i)); % Drag Force N
        mass(i+1) = mass(i); % Mass at next step Kg
        aoa = theta(i) - atand(Vy(i)/(Vx(i)+wind));
        Flift(i) = fLift(rho,Aw,Cl_rocket(aoa),Vrel(i)); % Lift force for torqueing
        Fnetx = -Flift(i)*sind(theta(i)) + Fdrag(i)*cosd(theta(i)); % Forces in x direction
        Fnety = Flift(i)*cosd(theta(i)) + Fdrag(i)*sind(theta(i)) + Fg(i); % Forces in y direction
        [theta(i+1),thetad(i+1)] = fTh(Flift(i),dt,mass(i),Lr,Cp,Cg,theta(i),thetad(i)); % Weathercocking
        Vx(i+1) = Vx(i) + (Fnetx/mass(i))*dt; % Velocity in next time step depends on forces now
        Vy(i+1) = Vy(i) + (Fnety/mass(i))*dt; % Velocity in next time step depends on forces now
        x_pos(i+1) = x_pos(i) + Vx(i+1)*dt;
        y_pos(i+1) = y_pos(i) + Vy(i+1)*dt;
        thrustStep = Stepper_over;
        time(i+1) = time(i) + dt;
        i = i + 1;
    end
    
    % Recovery Phase
    while (y_pos(i) > 0)
        Vx(i) = -wind;
        Vy(i) = v_term;
        dx = -wind * dt;
        dy = v_term * dt;
        x_pos(i+1) = x_pos(i) + dx;
        y_pos(i+1) = y_pos(i) + dy;
        time(i+1) = time(i) + dt;
        mass(i+1) = mc;
        theta(i+1) = 90;
        thetad(i+1) = 0;
        i = i + 1;
    end
end
%% End Launch Simulation


%% Trajectory Plot Function
function plotTraj(x_pos,y_pos,th)
    figure;
    plot(x_pos, y_pos);
    hold on;
    grid on;
    xlabel('Horizontal Position (m)');
    ylabel('Altitude (m)');
    title(['Theta0=' num2str(th) 'deg | Rocket Position in Space Through Flight']);
end
%% End Trajectory Plot Function


%% Drag Force Function
function fd = fDrag(rho,A,Cd,vrel)
    fd = -0.5*Cd*rho*A*vrel^2; return;
end
%% End Drag Force Function


%% Lift Force Function
function fl = fLift(rho,A,Cl,vrel)
    fl = 0.5*Cl*rho*A*vrel^2; return;
end
%% End Lift Force Function


%% Weathercock Function
function [th,thetad] = fTh(FL,dt,m,L,Cp,Cg,tht,thtd)
    tht_rad = tht * (pi/180);
    thtd_rad = thtd * (pi/180);
    I = (m*L^2)/12; % Moment of inertia Kg*m^2 (assuming its a rod)
    T = FL*(Cp-Cg); % Torque N*m
    thetadd = T/I; % Angular acceleration rad/s^2
    thetad = thtd_rad + thetadd*dt;
    th = tht_rad + thtd_rad*dt + 0.5*thetadd*dt^2;
    thetad = thetad * (180/pi);
    th = th * (180/pi);
    return;
end
%% End Weathercock Function


%% Coefficient Of Lift Function
function cl = Cl_rocket(a)
    cl = (2/13)*a; return;
end
%% Coefficient Of Lift Function