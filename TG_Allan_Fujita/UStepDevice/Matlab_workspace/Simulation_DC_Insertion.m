%% Simulation of needle insertion with Arc-Based RRT Path Planning
%% PARAMETRIZATION IN LENGTH!

function Simulation_DC_Insertion
clear all; close all; clc
global kappa; global ds;

ds = 0.001;
kappa = 1/0.05;
Ltotal = 10;

theta = pi/2;
q = DQ([cos(theta/2);sin(theta/2)*[1;0;0]]); 
t1 = DQ([0;0;0.6;0]); 

dqa1 = q + DQ.E*0.5*t1*q;    

L1 = 0;

figure; hold on; grid on; axis equal; %view(-35,45);
grid off
axis(2*[-0.01 0.5 -0.5 0.5 -0.01 0.01]);
xlabel('X'); ylabel('Y'); zlabel('Z'); 
  
% Simulate Insertion
t = translation(dqa1);
tip1(:,1) = t.q(2:4);
h1 = plotTip(tip1, dqa1, 'r');

k = 1;
to = 0;
% while(L1 < Ltotal)  
for k = 1:1000
    v = 0.001;
    w = 2*pi;
%     k = k+1;
    [dqa1 L1 to] = needleUpdate(dqa1, w, v, 0.0, L1, to);
    t = translation(dqa1);
    tip1(:,k) = t.q(2:4);
    h1 = plotTip(tip1, dqa1, 'r', h1);

end;

save('simulation_needle_dc2');


function h = plotTip(tip, dq, color, h)
if (nargin < 3)
    error('To few input data in function "plotTip"');
elseif (nargin == 3) 
    h = [];
end;
if(isempty(h))
    h.trace = plot3(tip(1,1), tip(2,1), tip(3,1),color);  
    h.dq = plot(dq,'scale',0.1);
else
    set(h.trace,'XData',tip(1,:),'YData',tip(2,:),'ZData',tip(3,:));
    h.dq = plot(dq,'erase',h.dq,'scale',0.1);
end

%% Needle Simulation

function [dq_res L_res t_res] = needleUpdate(dqa, w, v, DC, L, to)
% Update Needle Position
global kappa; global ds;
% 
% T = ds/v;
% Trot = DC*T;
% Tins = T - Trot;

if DC == 0
    Trot = 0; 
    Tins = 1; 
else
    Trot = 2*pi/w;
    Tins = Trot*(1-DC)/DC;
end;
T = Trot+Tins;
v = ds/T;

% % Test axis inversion
if (abs(dq_roll(dqa)) > 0.5*pi)
    s = -1;
else
    s = 1;
end
if (sign(DC*s) < 0)
    dqinv = DQ([cos(pi*0.5);sin(pi*0.5)*[1;0;0]]);
else
    dqinv = DQ(1);
end;

% With rotation in x
    W = sqrt(w^2+kappa^2*v^2); n = [w/W;0;(kappa*v)/W];
    q = DQ([cos(W*Trot*0.5);sin(W*Trot*0.5)*n]);
    t = DQ([1;0;0;0; 0;0.5*v*Trot;0;0]);
    dqi1 = t*q;    
% No rotation in x
    W = kappa*v; n = [0;0;1];
    q = DQ([cos(W*Tins*0.5);sin(W*Tins*0.5)*n]);
    t = DQ([1;0;0;0; 0;0.5*v*Tins;0;0]);
    dqi2 = t*q;
dq_res = dqa*dqinv*dqi1*dqi2;
L_res = L + v*T;
t_res = to + T;
    

function res = dq_roll(dq)
    res = atan2(2*(dq.q(1)*dq.q(2)+dq.q(3)*dq.q(4)),1-2*(dq.q(2)^2+dq.q(3)^2));





