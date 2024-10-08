%%%% Parameter's for Dubin's Car %%%%
ang = pi/6;
S = 1;  % 1
L = 2.8; % 2.8
phi = pi/4;
q = [-8 -6 0];

%%%% Parameters for Runge Kutta %%%%
a = 0;                   % Interval intial value
b = 50;                  % Interval final value
N = 100;                 % No of samples
h = (b-a)/N;             % Step size for values between interval

q_list = [];
ctr = 0;
for i = 1:N;
    ctr = 1 + ctr;
    k1 = [h*S*cos(q(3)),h*S*sin(q(3)),h*((S*tan(phi))/L)];
    k2 = [h*S*cos(q(3)+(0.5*h)),h*S*sin(q(3)+(0.5*h)),h*((S*tan(phi+(k1(3)+0.5*h)))/L)];
    k3 = [h*S*cos(q(3)+(0.5*h)),h*S*sin(q(3)+(0.5*h)),h*((S*tan(phi+(k2(3)+0.5*h)))/L)];
    k4 = [h*S*cos(q(3)+h),h*S*sin(q(3)+h),h*((S*tan(phi+(k3(3)*h)))/L)];
    q_list(ctr,:) = q;    
    
    x_new = q(1) + (k1(1) + 2*k2(1) + 2*k3(1) + k4(1))*h*(1/6);
    y_new = q(2) + (k1(2) + 2*k2(2) + 2*k3(2) + k4(2))*h*(1/6);
    t_new = q(3) + (k1(3) + 2*k2(3) + 2*k3(3) + k4(3))*h*(1/6);
    q = [x_new, y_new, t_new];
end

X = q_list(:,1);
Y = q_list(:,2);
T = q_list(:,3);

plot(goal(:,1),goal(:,2),'gp', 'Linewidth', 3); hold on;
plot(origin(:,1),origin(:,2),'mp', 'Linewidth', 3); hold on;
plot3(X,Y,T,'b')
txt3 = 'Dubins Car';
text(-8, -6, txt3,'HorizontalAlignment','right')
title('WORKSPACE')
hold on