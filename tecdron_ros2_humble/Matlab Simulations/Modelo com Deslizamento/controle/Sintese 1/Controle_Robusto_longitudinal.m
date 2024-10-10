%% Primeiramente, vamos criar a planata do sistema
clear
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
 
 %representação nos espaços de estados
 
A=[-g*uc/vlimite 0 0 0;
   1 0 0 0;
   0 0 -g*m*uc*(l^2+w^2)/(Iz*vlimite) 0;
   0 0 1 0];

   
B=[r*g*uc/(4*vlimite) r*g*uc/(4*vlimite) r*g*uc/(4*vlimite) r*g*uc/(4*vlimite);
    0 0 0 0;
   g*m*uc*r*w/(4*Iz*vlimite) -g*m*uc*r*w/(4*Iz*vlimite) -g*m*uc*r*w/(4*Iz*vlimite) g*m*uc*r*w/(4*Iz*vlimite);
   0 0 0 0];

m=0.05*m;
uc = 0.05*uc;

A_in= [-g*uc/vlimite 0 0 0;
   0 0 0 0;
   0 0 -g*m*uc*(l^2+w^2)/(Iz*vlimite) 0;
   0 0 0 0];
%Matriz B das incertezas
B_in=B;

%Vamos partir para a análise da malha
s = tf('s');

% Definição das variáveis incertezas e as matrizes incertas
delta1 = ureal('delta1',0);
delta2 = ureal('delta2',0);
Ap = A+delta1*A_in;
%Bp = B+delta2*B_in;
Bp=B;
Cp=[1 0 0 0;0 0 1 0];
Dp=zeros(2,4);
 
 %Construção da Planta
 Plant = uss(Ap,Bp,Cp,Dp); %Planta Final
 
 
 %% 
 %Análise do controlador
s = zpk('s');
K = tunableSS('K',3,4,2);
K.D.Value = 0;      % set D = 0
K.D.Free = false;   % fix D to zero

%Ponderações
%V1
% w_e1 = 0.8*(s+1)/(s+10);
% w_e2 = 0.8*(s+1)/(s+10);
% 
% w_u1= 0.5*(s+1)/(s+10);
% w_u2= 0.5*(s+1)/(s+10);


% %V2
w_e1 = 0.2*(s+1)/(s+2);
w_e2 = 0.2*(s+1)/(s+20);

w_u1= 0.1*(s+5)/(s+100);
w_u2= 0.1*(s+5)/(s+100);
w_u3= 0.1*(s+5)/(s+100);
w_u4= 0.1*(s+5)/(s+100);



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

Planta=connect(Plant,In,K,w_e1,w_e2,w_u1,w_u2,w_u3,w_u4,Feedback,{'r'},{'pond_e1','pond_e2','pond_u1','pond_u2','pond_u3','pond_u4','u(1)', 'u(2)','u(3)','u(4)','y'});

Req = TuningGoal.Gain({'r'},{'pond_e1','pond_e2','pond_u1','pond_u2','pond_u3','pond_u4'},1); %ganho de r para a ponderação igual a 1
options = systuneOptions('RandomStart',10,'SoftTarget',1);

Req3=TuningGoal.StepTracking('r(2)','y(2)',0.25);
Req2=TuningGoal.StepTracking('r(1)','y(1)',0.25);

Req4 = TuningGoal.StepRejection('r(2)','y(1)',0.05,0.5);
Req5= TuningGoal.StepRejection('r(1)','y(2)',0.05,0.5);%Quanto mais coloco restrições de rejeição mais as correntes se comportam de forma simétrica

Req6 = TuningGoal.LQG('r',{'u(1)', 'u(2)','u(3)','u(4)'}, 10, 0.01);

[CL,fSoft] = systune(Planta,[Req,Req2,Req3, Req4, Req5, Req6],[],options);
K = ss(CL.Blocks.K) % representação do espaço de estado controlador

 figure
 viewGoal(Req,CL)

 figure
 viewGoal(Req2,CL)

 figure
 viewGoal(Req3,CL)

 figure
 viewGoal(Req4,CL)

  figure
 viewGoal(Req5,CL)



