function [] = driver()
    clc; close all;
    % CONSTANTS
    % Earth Properties
    earth.R = 6378.1;           % Radius [km]
    earth.mu = 398600.436233;   % Gravitational Parameter [km^3/s^2]
    earth.J2 = 1082.63;         % J2 effects

    % Moon
    moon.R = 1737.4;            % Radius [km]
    moon.mu = 4902.800076;      % Gravitational Parameter [km^3/s^2]
    moon.J2 = 202.7;            % J2 effects
    moon.a = 0.3844e6;          % Semi-major axis [km]     
    moon.e = 0.0549;            % Eccentricity [-]
    moon.i = 5.145;             % Inclination [°]
    moon.Om = 0.0;              % RAAN [°]
    moon.w = 0.0;               % Argument of perigee [°]
    moon.phi = 0.0;             % True anomaly [°]
    
    % Satellite    
    sat.r_0(1:3) = [247.122; 0.0; -4493.209];
    sat.r_0(4:6) = [0.0; 1.44467; 0.0];
    
    % INITIAL CONDITIONS - Earth-fixed frame to be considered inertial
    moon.r_0 = kep2cart(earth, moon);
    moon.t_0 = [0 120*24*3600];
%     sat.r_0 = rel2in(sat, moon);
    
%     r_0 = [moon.r_0(1:3); sat.r_0(1:3); moon.r_0(4:6); sat.r_0(4:6)];
    
    % ORBIT PROPAGATION
    opts = odeset('RelTol',1e-10,'AbsTol',1e-12);
%     [t, r] = ode45(@(t,r) dr_m(t,r, earth, moon), moon.t_0, r_0, opts);
    [t, r] = ode45(@(t,r) dr_s(t,r, earth, moon), moon.t_0, sat.r_0, opts);
    
    % ORBIT PLOT
    figure
    plot3(r(:,1), r(:,2), r(:,3));
    grid on
    hold on
%     plot3(r(:,4), r(:,5), r(:,6));
end

function [ R ] = kep2cart(mainBody, orbitObject)
    % Unpack object
    mu = mainBody.mu;
    a = orbitObject.a;
    e = orbitObject.e;
    i = orbitObject.i;
    Om = orbitObject.Om;
    w = orbitObject.w;
    phi = orbitObject.phi; 
    
    p = a*(1 - e^2);
    r = p/(1 + e*cosd(phi));
    phi_d = (1 + e*cosd(phi))^2*sqrt(mu/p^3);
    r_d = p*e*sind(phi)*sqrt(mu/p^3);
    
    r1 = r*cosd(phi);
    r2 = r*sind(phi);
    r3 = 0.0;
    
    v1 = r_d*cosd(phi) - r*phi_d*sind(phi);
    v2 = r_d*sind(phi) + r*phi_d*cosd(phi);
    v3 = 0.0;
    
    r_vec = C_ck(w, i, Om)*[r1; r2; r3];
    v_vec = C_ck(w, i, Om)*[v1; v2; v3];
    
    x = r_vec(1);
    y = r_vec(2);
    z = r_vec(3);
    u = v_vec(1);
    v = v_vec(2);
    w = v_vec(3);
    
    R = [x, y, z, u, v, w]';
end

function [ R ] = rel2in(sat, moon)
    R = sat.r_0;

    % Rotation Matrix
    L = cross( moon.r_0(1:3), moon.r_0(4:6) );
    e_mz = L/norm(L);
    e_mx = cross(e_my
    
    e_my = -moon.r_0(1:3)/norm(moon.r_0(1:3));
    
    R(1:3) = R(1)*e_mx + R(2)*e_my + R(3)*e_mz + moon.r_0(1:3);
    R(4:6) = R(4)*e_mx + R(5)*e_my + R(6)*e_mz + moon.r_0(4:6); 
    R = R';
end

function [ C ] = C_kc(w, i, Om)
    % Rotation matrix from Cartesian to Kepplerian
    c11 = cosd(Om)*cosd(w) - sind(Om)*sind(w)*cosd(i);
    c12 = sind(Om)*cosd(w) + cosd(Om)*sind(w)*cosd(i);
    c13 = sind(w)*sind(i);
    c21 = -cosd(Om)*sind(w) - sind(Om)*cosd(w)*cosd(i);
    c22 = -sind(Om)*sind(w) + cosd(Om)*cosd(w)*cosd(i);
    c23 = cosd(w)*sind(i);
    c31 = sind(Om)*sind(i);
    c32 = -cosd(Om)*sind(i);
    c33 = cosd(i);

    C =  [ c11 c12 c13;
           c21 c22 c23;
           c31 c32 c33 ];
end

function [ C ] = C_ck(w, i, Om)
    % Rotation matrix from Kepplerian to Cartesian
    C = C_kc(w, i, Om);
    C = C';
end

function [ dr ] = dr_m(t,r, earth, moon)
    % - Kinematics -
    % Gravitational Parameters
    mu_e = earth.mu;
    mu_m = moon.mu;
    
    % Locations
    r_m = r(1:3);               % Position of the moon wrt earth
    r_s = r(4:6);               % Position of the satellite wrt earth
    r_rel = r(4:6) - r(1:3);    % Position of the satellite wrt the moon
    
    % Magniture of distances
    R_m = norm( r_m );
    R_s = norm( r_s );
    R_sm = norm( r_rel );
    
    % Equations of motion
    dr(1) = r(7);
    dr(2) = r(8);
    dr(3) = r(9);
    dr(4) = r(10);
    dr(5) = r(11);
    dr(6) = r(12);
    dr(7:9)   = - mu_e/R_m^3 * r_m;
    dr(10:12) = - mu_e/R_s^3 * r_s - mu_m/R_sm^3 * r_rel;
%     dr(10:12) = - mu_m/R_sm^3 * r_rel;
    
    dr = dr';
end

function [ dr ] = dr_s(t,r, earth, moon)
    % - Kinematics -
    % Gravitational Parameters
    mu_e = earth.mu;
    mu_m = moon.mu;
    
    % Locations
    r_s = r(1:3);               % Position of the satellite wrt the moon
    
    % Magniture of distances
    R_s = norm( r_s );
    
    % Equations of motion
    dr(1) = r(4);
    dr(2) = r(5);
    dr(3) = r(6);
    dr(4:6) = - mu_m/R_s^3 * r_s;
    
    dr = dr';
end

