%
% vehicle.m
%
% created: Jan 2017
%  author: Matthias Rungger
%
% see readme file for more information on the vehicle example
%
% you need to run ./vehicle binary first 
% so that the following files are creatd
%
%   controller.scs
%   controller.bdd
%   obstacles.scs
%   obstacles.bdd
%   obstacles.scs
%   target.scs 
%   target.bdd 
%

function vehicle
clear all
close all


%% simulation

% target set
lb=[9 0];
ub=lb+0.5;
% initial state
x0=[0.6 0.6 0];


% load controller from file
controller=SymbolicSet('determinized');

% simulate closed loop system
y=x0;
v=[];
loop=3000;

% tau
tau=0.3;

while(loop>0)
    loop=loop-1;
  
  if (lb(1) <= y(end,1) & y(end,1) <= ub(1) &&...
      lb(2) <= y(end,2) & y(end,2) <= ub(2))
    break;
  end 
  
  u=controller.restriction(y(end,:));
  % pick a valid input
  u=u(1,:)

  [t x]=ode45(@unicycle_ode,[0 tau], y(end,:), odeset('abstol',1e-12,'reltol',1e-12),u);
  
  v=[v; u];
  y=[y; x(end,:)];

end

%% plot the vehicle domain
% colors
colors=get(groot,'DefaultAxesColorOrder');

% load the symbolic set containig obstacles
obs=SymbolicSet('obstacles');
obs=obs.points([1 2]);
plot(obs(:,1),obs(:,2),'.');
hold on

target=SymbolicSet('target');
target=target.points([1 2]);
plot(target(:,1),target(:,2),'.','color',colors(2,:));
hold on

% plot the real obstacles and target set
plot_domain

% plot initial state  and trajectory
plot(y(:,1),y(:,2),'k.-')
plot(y(1,1),y(1,2),'.','color',colors(5,:),'markersize',20)

box on
axis([-.5 10.5 -.5 10.5])

%set(gcf,'paperunits','centimeters','paperposition',[0 0 16 10],'papersize',[16 10])

end

function dxdt = unicycle_ode(t,x,u)

  dxdt = zeros(3,1);
  c=atan(tan(u(2))/2);

  dxdt(1)=u(1)*cos(c+x(3))/cos(c);
  dxdt(2)=u(1)*sin(c+x(3))/cos(c);
  dxdt(3)=u(1)*tan(u(2));

end

function plot_domain

colors=get(groot,'DefaultAxesColorOrder');

alpha=0.2;

v=[9 0; 9.5  0; 9 0.5; 9.5 .5];
patch('vertices',v,'faces',[1 2 4 3],'facea',alpha,'facec',colors(2,:),'edgec',colors(2,:));


v=[1     0  ;1.2  0   ; 1     9    ; 1.2 9   ];
patch('vertices',v,'faces',[1 2 4 3],'facea',alpha,'facec',colors(1,:),'edgec',colors(1,:));
v=[2.2   0  ;2.4  0   ; 2.2   5    ; 2.4 5   ];
patch('vertices',v,'faces',[1 2 4 3],'facea',alpha,'facec',colors(1,:),'edgec',colors(1,:));
v=[2.2   6  ;2.4  6   ; 2.2   10   ; 2.4 10  ];
patch('vertices',v,'faces',[1 2 4 3],'facea',alpha,'facec',colors(1,:),'edgec',colors(1,:));
v=[3.4   0  ;3.6  0   ; 3.4   9    ; 3.6 9   ];
patch('vertices',v,'faces',[1 2 4 3],'facea',alpha,'facec',colors(1,:),'edgec',colors(1,:));
v=[4.6   1  ;4.8  1   ; 4.6   10   ; 4.8 10  ];
patch('vertices',v,'faces',[1 2 4 3],'facea',alpha,'facec',colors(1,:),'edgec',colors(1,:));
v=[5.8   0  ;6    0   ; 5.8   6    ; 6   6   ];
patch('vertices',v,'faces',[1 2 4 3],'facea',alpha,'facec',colors(1,:),'edgec',colors(1,:));
v=[5.8   7  ;6    7   ; 5.8   10   ; 6   10  ];
patch('vertices',v,'faces',[1 2 4 3],'facea',alpha,'facec',colors(1,:),'edgec',colors(1,:));
v=[7     1  ;7.2  1   ; 7     10   ; 7.2 10  ];
patch('vertices',v,'faces',[1 2 4 3],'facea',alpha,'facec',colors(1,:),'edgec',colors(1,:));
v=[8.2   0  ;8.4  0   ; 8.2   8.5  ; 8.4 8.5 ];
patch('vertices',v,'faces',[1 2 4 3],'facea',alpha,'facec',colors(1,:),'edgec',colors(1,:));
v=[8.4   8.3;9.3  8.3 ; 8.4   8.5  ; 9.3 8.5 ];
patch('vertices',v,'faces',[1 2 4 3],'facea',alpha,'facec',colors(1,:),'edgec',colors(1,:));
v=[9.3   7.1;10   7.1 ; 9.3   7.3  ; 10  7.3 ];
patch('vertices',v,'faces',[1 2 4 3],'facea',alpha,'facec',colors(1,:),'edgec',colors(1,:));
v=[8.4   5.9;9.3  5.9 ; 8.4   6.1  ; 9.3 6.1 ];
patch('vertices',v,'faces',[1 2 4 3],'facea',alpha,'facec',colors(1,:),'edgec',colors(1,:));
v=[9.3   4.7;10   4.7 ; 9.3   4.9  ; 10  4.9 ];
patch('vertices',v,'faces',[1 2 4 3],'facea',alpha,'facec',colors(1,:),'edgec',colors(1,:));
v=[8.4   3.5;9.3  3.5 ; 8.4   3.7  ; 9.3 3.7 ];
patch('vertices',v,'faces',[1 2 4 3],'facea',alpha,'facec',colors(1,:),'edgec',colors(1,:));
v=[9.3   2.3;10   2.3 ; 9.3   2.5  ; 10  2.5 ];
patch('vertices',v,'faces',[1 2 4 3],'facea',alpha,'facec',colors(1,:),'edgec',colors(1,:));


end
