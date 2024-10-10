clc 
clear all

global r m g J Iz Cm uc c vlimite w l L R Kt Kbm Cm reducao Ap Bp Cp Dp Q_k R_k
 
 g=9.81;
 uc=0.7;
 vlimite=0.1;
 r=0.06;
 l=0.38/2;
 w=0.203/2;
 Iz=6.125;
 m=5.0;
 c=0.015;
 Cm=0;
 reducao=30;
 J=0.000331;
 Kt=0.05;



%dados do motor
L=0.001;
R=40;
Kt=0.05;
Kbm=0.05;
Cm=0;
reducao=30;

%% Linear model fori Kalman Filter
Ap=[-g*uc/vlimite 0 0;
    0 g*uc/vlimite 0;
    0 0 -g*m*uc*(l^2+w^2)/(Iz*vlimite)];

   
Bp=[r*g*uc/(4*vlimite) r*g*uc/(4*vlimite) r*g*uc/(4*vlimite) r*g*uc/(4*vlimite);
    0 0 0 0;
   g*m*uc*r*w/(4*Iz*vlimite) -g*m*uc*r*w/(4*Iz*vlimite) -g*m*uc*r*w/(4*Iz*vlimite) g*m*uc*r*w/(4*Iz*vlimite)];

Cp= eye(3);
Dp=zeros(3,4);

Q_k = 0.2*eye(3);
R_k = 0.2*eye(3);