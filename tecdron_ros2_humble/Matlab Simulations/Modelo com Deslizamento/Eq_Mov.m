function y=Eq_Mov(V)
global g vlimite r m uc_x uc_y

u = [uc_x;uc_y];

w = V(1);
v = [V(2);V(3)];
theta = V(4);

TF_wheel_roller = [cos(theta) -sin(theta);
                   sin(theta) cos(theta)];

Vd = -(TF_wheel_roller')*v+[w*r;0];

Fw = ((m*g/4)/(vlimite))*[Vd(1)*u(1);Vd(2)*u(2)];

F = TF_wheel_roller*Fw;
y = F;

end