function dx=luenberger_non_linear(flag,t,x)
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
C= eye(6);
D = [0];
Q = (C'*C);
Q(1,1) = 500;
Q(2,2) = 50000000;
Q(3,3) = 500;
Q(4,4) = 500000000;
Q(5,5) = 500;
Q(6,6) = 500000000;
R = 2;
p=[-5;-0.3;-0.4;-1;-0.5;-0.2];
if flag == 1
    %for x
    C(2:6,:) = 0;
end
if flag == 2
    %for x t2
    C([2:4,6],:) = 0;
end
if flag == 3
    %for x t1 t2
    C([2,4,6],:) = 0;
end
disp(C);
Kr=lqr(A,B,Q,R); %lqr gain
u=-Kr*x(1:6);
L=place(A',C',p);%pole placement
L=L';
x_ = (u - m1*g*sin(x(3))*cos(x(3))-m2*g*sin(x(5))*cos(x(5))-l1*m1*(x(4)^2)*sin(x(3))-l2*m2*(x(6)^2)*sin(x(5)))/(M+m1*(sin(x(3))^2)+m2*sin(x(5)^2));
xe_dot = (A-L.*C)*x(7:12);
%{
 State space by seperation principle for stability
Xdot     =    A+BK    -BK    *  X
Xedot           0     A-LC      Xe

from Xe we find X^ : Xe =  X-X^
%}
dx1(1,1)= x(2);
dx1(2,1)= x_ ;
dx1(3,1)= x(4);
dx1(4,1)= (x_*cos(x(3))-g*sin(x(3)))/l1;
dx1(5,1)= x(6);
dx1(6,1)= (x_*cos(x(5))-g*sin(x(5)))/l2;
%estimator matrix
dx1(7,1)=x(2)-x(8);
dx1(8,1)= dx1(2,1)-xe_dot(2); %Xe = X-X^ -> X^ = X-Xe                      
dx1(9,1)=x(4)-x(10);
dx1(10,1)= dx1(4,1)-xe_dot(4);
dx1(11,1)= x(6)-x(12);
dx1(12,1)=dx1(6,1)-xe_dot(6);
%{
Acc=[(A-B*Kr) (B*Kr); zeros(size(A)) (A-L*C)];
Bcc=[B;zeros(size(B))];
Ccc=[C zeros(size(C))];
Dcc=[0];
Qcc=eye(12)*10;
Rcc=0.01;
Ksyslqr=lqr(Acc,Bcc,Qcc,Rcc);
%}
dx=dx1;

end