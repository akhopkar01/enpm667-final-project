%Given Parameters
M=1000;
m1=100;
m2=100;
l1=20;
l2=10;
g=9.81;
%initialization
A = [0 1 0 0 0 0 ;
    0 0 ((-m1*g)/M) 0 (-m2*g)/M 0;
    0 0 0 1 0 0;
    0 0 ((-g*(m1+M))/(M*l1)) 0 (-g*(m2)/(M*l1)) 0;
    0 0 0 0 0 1;
    0 0 (-g*(m1))/(M*l2) 0 (-g*(m2+M)/(M*l2)) 0];
B = [0; 1/M; 0 ; 1/(M*l1); 0 ; 1/(M*l2)];
C = eye(6);
D = zeros(6,1);
%Setting cost functions
Q = C'*C;
Q(1,1) = 500;
Q(2,2) = 50000000;
Q(3,3) = 500;
Q(4,4) = 500000000;
Q(5,5) = 5000;
Q(6,6) = 500000000;  
R = 2;
%Finding closed loop optimal gain
[K,S,P] = lqr(A,B,Q,R);
%State space model:
sys1 = ss(A-B*K,B,C,D);
%Initial State
X0 = [-2;0;1;0;1;0];
E = eig(A-B*K);
%setting time vector 
[y,T] = initial(sys1,X0);

figure('Name','LQR Linear')
hold on
subplot(3,1,1);
plot(T,y(:,1),'r')
title('Position of the cart')
subplot(3,1,2);
plot(T,y(:,2),'b')
title('Angle of first pendulum')
subplot(3,1,3);
plot(T,y(:,3),'g')
title('Angle of second pendulum')
hold off
grid

X0 = [-2;0;1;0;1;0];
tspan = (0:1:100);
[t,x]=ode45(@(t,x)nonlinear(x,K),tspan,X0);
figure('Name','LQR for Non Linear')
subplot(3,1,1);
plot(t,x(:,1),'r')
title('Position of the cart')
subplot(3,1,2);
plot(t,x(:,2),'b')
title('Angle of first pendulum')
subplot(3,1,3);
plot(t,x(:,3),'g')
title('Angle of second pendulum')
grid



