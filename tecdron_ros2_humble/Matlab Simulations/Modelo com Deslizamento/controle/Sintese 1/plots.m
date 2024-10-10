%colocar os gráficos

%Definir a planta
%tenho que já estar com o controlador colocado

 g=9.81;
 uc=0.7;
 vlimite=0.1;
 r=0.06;
 l=0.187;
 w=0.203;
 Iz=0.0338;
 m=3.044;
 Cr=0;
 Cm=0;
 reducao=30;
 J=0.0000331;
 Kt=0.05;

 A=[0 0 0 0 uc*r*g/4*vlimite uc*r*g/4*vlimite uc*r*g/4*vlimite uc*r*g/4*vlimite;
    1 0 0 0 0 0 0 0;
    0 0 0 0 -r*g*m*uc*w/(8*Iz*vlimite) r*g*m*uc*w/(8*Iz*vlimite) -r*g*m*uc*w/(8*Iz*vlimite) r*g*m*uc*w/(8*Iz*vlimite);
    0 0 1 0 0 0 0 0;
    0 0 0 0 -Cm*reducao/J-Cr/J-r^2*g*m*uc/(4*vlimite*J) 0 0 0;
    0 0 0 0 0 -Cm*reducao/J-Cr/J-r^2*g*m*uc/(4*vlimite*J) 0 0;
    0 0 0 0 0 0 -Cm*reducao/J-Cr/J-r^2*g*m*uc/(4*vlimite*J) 0;
    0 0 0 0 0 0 0 -Cm*reducao/J-Cr/J-r^2*g*m*uc/(4*vlimite*J)];
 
 B=[zeros(4,4);Kt/J*eye(4)];
 
 Cp=[1 0 0 0 0 0 0 0;0 0 0 1 0 0 0 0];
 Dp=zeros(2,4);

 Plant=ss(A,B,Cp,Dp);
s = zpk('s');

%Ponderações
w_e1 = 0.5*(s+1)/(s+10);
w_e2 = 0.5*(s+1)/(s+10);

w_u1= 0.8*(s)/(s+5);
w_u2= 0.8*(s)/(s+5);
w_u3= 0.8*(s)/(s+5);
w_u4= 0.8*(s)/(s+5);

Ai=zeros(2);
Bi=eye(2);
Ci=eye(2);
Di=zeros(2);
In=ss(Ai,Bi,Ci,Di);

 
%Nomeando
In.InputName='e';
In.OutputName='ie';


K.InputName='ie';
K.OutputName='u';

Plant.InputName='u';
Plant.OutputName='y';

w_e1.InputName='ie(1)';
w_e1.OutputName='pond_e1';
w_e2.InputName='ie(2)';
w_e2.OutputName='pond_e2';

w_u1.OutputName='pond_u1';
w_u1.InputName='u(1)';
w_u2.OutputName='pond_u2';
w_u2.InputName='u(2)';
w_u3.OutputName='pond_u3';
w_u3.InputName='u(3)';
w_u4.OutputName='pond_u4';
w_u4.InputName='u(4)';

Feedback = sumblk('e = r-y',2);

Planta=connect(Plant,In,K,w_e1,w_e2,w_u1,w_u2,w_u3,w_u4,Feedback,{'r'},{'pond_e1','pond_e2','pond_u1','pond_u2','pond_u3','pond_u4'});

%% Vamos agora salvar os modulos das saidas de cada ponderação
[mag phase]=bode(Planta);
pond_e1e1=squeeze(mag(1,1,:));
pond_e1e2=squeeze(mag(1,2,:));
pond_e2e1=squeeze(mag(2,1,:));
pond_e2e2=squeeze(mag(2,2,:));
pond_u1e1=squeeze(mag(3,1,:));
pond_u1e2=squeeze(mag(3,2,:));
pond_u2e1=squeeze(mag(4,1,:));
pond_u2e2=squeeze(mag(4,2,:));
pond_u3e1=squeeze(mag(5,1,:));
pond_u3e2=squeeze(mag(5,2,:));
pond_u4e1=squeeze(mag(6,1,:));
pond_u4e2=squeeze(mag(6,2,:));

%Calcular o inverso das conderações
f=logspace(-8,8,285);
[e pe]=bode((w_e1)^-1,f);
[u pu]=bode((w_u1)^-1,f);
e=squeeze(e);
u=squeeze(u);


%% Plots
%erro 1
figure
semilogx(f,20*log10(pond_e1e1))
hold on
semilogx(f,20*log10(e))
semilogx(f,20*log10(pond_e1e2))
hold off

%erro 2
figure
semilogx(f,20*log10(pond_e2e1))
hold on
semilogx(f,20*log10(e))
semilogx(f,20*log10(pond_e2e2))
hold off

%sinal de controle 1
figure
semilogx(f,20*log10(pond_u1e1))
hold on
semilogx(f,20*log10(u))
semilogx(f,20*log10(pond_u1e2))
hold off

%sinal de controle 2
figure
semilogx(f,20*log10(pond_u2e1))
hold on
semilogx(f,20*log10(u))
semilogx(f,20*log10(pond_u2e2))
hold off

%sinal de controle 3
figure
semilogx(f,20*log10(pond_u3e1))
hold on
semilogx(f,20*log10(u))
semilogx(f,20*log10(pond_u3e2))
hold off

%sinal de controle 4
figure
semilogx(f,20*log10(pond_u4e1))
hold on
semilogx(f,20*log10(u))
semilogx(f,20*log10(pond_u4e2))
hold off





