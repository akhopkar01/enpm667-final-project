
function dx = nonlinear(x,K)
M = 1000;
m1 = 100;
m2 = 100;
l1 = 20;
l2 = 10;
g = 9.81;
u = -K*x;
disp(u);
x_ = (u - m1*g*sin(x(3))*cos(x(3))-m2*g*sin(x(5))*cos(x(5))-l1*m1*(x(4)^2)*sin(x(3))-l2*m2*(x(6)^2)*sin(x(5)))/(M+m1*(sin(x(3))^2)+m2*sin(x(5)^2));
dx(1,1)= x(2);
dx(2,1)= x_;
dx(3,1)=x(4);
dx(4,1)= (x_*cos(x(3))-g*sin(x(3)))/l1;
dx(5,1)=x(6);
dx(6,1)= (x_*cos(x(5))-g*sin(x(5)))/l2;
end

