function y=Esforcos(V)
global Iz m l w

F1=[V(1);V(2);0];
F2=[V(3);V(4);0];
F3=[V(5);V(6);0];
F4=[V(7);V(8);0];

xp=V(9);
psip=V(10);
yp=V(11);

F=F1+F2+F3+F4;
M=tilde([l;w;0])*F1+tilde([l;-w;0])*F2+tilde([-l;-w;0])*F3+tilde([-l;w;0])*F4;

Acel=F/m-tilde([0;0;psip])*[xp;yp;0];
x2p=Acel(1);
y2p=Acel(2);
psi2p=M(3)/Iz;

y=[x2p;y2p;psi2p];

end