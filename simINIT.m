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
[Kr,S,P] = lqr(A,B,Q,R);
%  Augment system with disturbances and noise
Vd = .001*eye(6);  % disturbance covariance
Vn = .001;       % noise covariance

BF = [B Vd 0*B];  % augment inputs to include disturbance and noise

%  Build Kalman filter
C = [1 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];
Kf = (lqr(A',C',Vd,Vn))';   % design using "LQR" code

sysKF = ss(A-Kf*C,[B Kf],eye(6),0*[B Kf]);  % Kalman filter estimator