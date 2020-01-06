clc
x_0= [-2;0;1;0;1;0;0;0;0;0;0;0];
flag = 3;
tspan=0:1:200;

[t,x]=ode45(@(t,x)luenberger_non_linear(flag,t,x),tspan,x_0);
figure('Name','Luenberger for Non Linear')
hold on
subplot(3,1,1);
plot(t,x(:,1),'r')
title('Position of the cart')
subplot(3,1,2);
plot(t,x(:,2),'b')
title('Angle of first pendulum')
subplot(3,1,3);
plot(t,x(:,3),'g')
title('Angle of second pendulum')
hold off
grid