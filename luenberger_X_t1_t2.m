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
C1 = C(1,:);
C2 = C([1,5],:);
C3 = C([1,3,5],:);
poles = [-5;-0.3;-0.4;-1;-0.5;-0.2];
L = place(A',C3',poles).';
Al = (A-L*C3);
Bl = B;
Cl = C;
Dl = D;
sys = ss(Al,Bl,Cl,Dl);
[y,T,x] = initial(sys,X0,80);
figure('Name','State Estimator for x,,t1,t2 linear initial')
subplot(3,1,1);
plot(T,y(:,1),'r')
title('Position of the cart')
subplot(3,1,2);
plot(T,y(:,2),'b')
title('Angle of first pendulum')
subplot(3,1,3);
plot(T,y(:,3),'g')
title('Angle of second pendulum')
grid
[x,T] = step(sys,80);
figure('Name','State Estimator for x,,t1,t2 linear step')
subplot(3,1,1);
plot(T,x(:,1),'r')
title('Position of the cart')
subplot(3,1,2);
plot(T,x(:,2),'b')
title('Angle of first pendulum')
subplot(3,1,3);
plot(T,x(:,3),'g')
title('Angle of second pendulum')
grid