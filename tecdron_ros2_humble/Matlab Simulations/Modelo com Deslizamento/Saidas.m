function y=Saidas(V)

yaw=V(1);
vx=V(2);
vy=V(3);
Tz=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)];

y=Tz*[vx;vy];

end