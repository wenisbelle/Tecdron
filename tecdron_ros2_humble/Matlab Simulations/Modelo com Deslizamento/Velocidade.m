function y=Velocidade(V)
global l w
xp=V(1);
yp=V(2);
psip=V(3);

V1=[xp;yp;0]+tilde([0;0;psip])*[l;-w;0];
V1=V1(1:2);

V2=[xp;yp;0]+tilde([0;0;psip])*[l;w;0];
V2=V2(1:2);

V3=[xp;yp;0]+tilde([0;0;psip])*[-l;w;0];
V3=V3(1:2);

V4=[xp;yp;0]+tilde([0;0;psip])*[-l;-w;0];
V4=V4(1:2);

y=[V1;V2;V3;V4];

end