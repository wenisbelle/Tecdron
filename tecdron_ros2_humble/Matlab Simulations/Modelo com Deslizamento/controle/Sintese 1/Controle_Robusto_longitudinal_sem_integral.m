%% Primeiramente, vamos criar a planata do sistema
clear
g=9.81;
uc_x=1.0;
uc_y=0.03;
theta=pi/4;
vlimite = 0.5;
l=0.70;
w=0.50;
m=150;
Iz = (1/12)*m*(l^2+w^2);
r=0.12;


A= [-(g*(4*uc_x + 4*uc_y))/(8*vlimite), 0, 0;
    0, -(g*(4*uc_x + 4*uc_y))/(8*vlimite),0;
    0, 0, -(g*m*(4*l^2*uc_x + 4*l^2*uc_y + 4*uc_x*w^2 + 4*uc_y*w^2 - 8*l*uc_x*w + 8*l*uc_y*w))/(8*Iz*vlimite)];
    
B = [(2^(1/2)*g*r*uc_x)/(8*vlimite), (2^(1/2)*g*r*uc_x)/(8*vlimite),(2^(1/2)*g*r*uc_x)/(8*vlimite),(2^(1/2)*g*r*uc_x)/(8*vlimite);
    (2^(1/2)*g*r*uc_x)/(8*vlimite),-(2^(1/2)*g*r*uc_x)/(8*vlimite),(2^(1/2)*g*r*uc_x)/(8*vlimite),-(2^(1/2)*g*r*uc_x)/(8*vlimite);
    (g*m*(2^(1/2)*l*r*uc_x - 2^(1/2)*r*uc_x*w))/(8*Iz*vlimite), -(g*m*(2^(1/2)*l*r*uc_x - 2^(1/2)*r*uc_x*w))/(8*Iz*vlimite), -(g*m*(2^(1/2)*l*r*uc_x - 2^(1/2)*r*uc_x*w))/(8*Iz*vlimite), (g*m*(2^(1/2)*l*r*uc_x - 2^(1/2)*r*uc_x*w))/(8*Iz*vlimite)];
   
B(3,:) = 7.5*B(3,:);

C = eye(3);
D = zeros(3,4);


m=0.5*m;
uc_x = 0.05*uc_x;
uc_y = 0.05*uc_y;

A_in= [-(g*(4*uc_x + 4*uc_y))/(8*vlimite), 0, 0;
        0, -(g*(4*uc_x + 4*uc_y))/(8*vlimite),0;
        0, 0, -(g*m*(4*l^2*uc_x + 4*l^2*uc_y + 4*uc_x*w^2 + 4*uc_y*w^2 - 8*l*uc_x*w + 8*l*uc_y*w))/(8*Iz*vlimite)];

%Matriz B das incertezas
B_in= [(2^(1/2)*g*r*uc_x)/(8*vlimite), (2^(1/2)*g*r*uc_x)/(8*vlimite),(2^(1/2)*g*r*uc_x)/(8*vlimite),(2^(1/2)*g*r*uc_x)/(8*vlimite);
        (2^(1/2)*g*r*uc_x)/(8*vlimite),-(2^(1/2)*g*r*uc_x)/(8*vlimite),(2^(1/2)*g*r*uc_x)/(8*vlimite),-(2^(1/2)*g*r*uc_x)/(8*vlimite);
        (g*m*(2^(1/2)*l*r*uc_x - 2^(1/2)*r*uc_x*w))/(8*Iz*vlimite), -(g*m*(2^(1/2)*l*r*uc_x - 2^(1/2)*r*uc_x*w))/(8*Iz*vlimite), -(g*m*(2^(1/2)*l*r*uc_x - 2^(1/2)*r*uc_x*w))/(8*Iz*vlimite), (g*m*(2^(1/2)*l*r*uc_x - 2^(1/2)*r*uc_x*w))/(8*Iz*vlimite)];

%Vamos partir para a análise da malha
s = tf('s');

% Definição das variáveis incertezas e as matrizes incertas
delta1 = ureal('delta1',0);
delta2 = ureal('delta2',0);
Ap = A+delta1*A_in;
Bp = B+delta2*B_in;
Cp=eye(3);
Dp=zeros(3,4);
 
%Construção da Planta
Plant = uss(Ap,Bp,Cp,Dp); %Planta Final
 
 
 %% 
 %Análise do controlador
s = zpk('s');
K = tunableSS('K',5,4,3);
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
w_e2 = 0.2*(s+1)/(s+2);
w_e3 = 0.2*(s+1)/(s+20);

w_u1= 0.1*(s+5)/(s+100);
w_u2= 0.1*(s+5)/(s+100);
w_u3= 0.1*(s+5)/(s+100);
w_u4= 0.1*(s+5)/(s+100);


Ai=zeros(3);
Bi=eye(3);
Ci=eye(3);
Di=zeros(3);
In=ss(Ai,Bi,Ci,Di);

 

K.InputName='e';
K.OutputName='u';

Plant.InputName='u';
Plant.OutputName='y';

w_e1.InputName='e(1)';
w_e1.OutputName='pond_e1';
w_e2.InputName='e(2)';
w_e2.OutputName='pond_e2';
w_e3.InputName='e(3)';
w_e3.OutputName='pond_e3';


w_u1.OutputName='pond_u1';
w_u1.InputName='u(1)';

w_u2.OutputName='pond_u2';
w_u2.InputName='u(2)';

w_u3.OutputName='pond_u3';
w_u3.InputName='u(3)';

w_u4.OutputName='pond_u4';
w_u4.InputName='u(4)';

% PRECISO APLICAR O zoh APÓS AS SAÍDAS DO CONTROLADOR. LER O LIVRO TAMBÉM
% aproximar por padé (1-0.5*ts)/(1+0.5*ts)

Feedback = sumblk('e = r-y',3);

Planta=connect(Plant,K,w_e1,w_e2,w_e3,w_u1,w_u2,w_u3,w_u4,Feedback,{'r'},{'pond_e1','pond_e2','pond_e3','pond_u1','pond_u2','pond_u3','pond_u4','u(1)', 'u(2)','u(3)','u(4)','y'});

Req = TuningGoal.Gain({'r'},{'pond_e1','pond_e2','pond_e3','pond_u1','pond_u2','pond_u3','pond_u4'},1); %ganho de r para a ponderação igual a 1
options = systuneOptions('RandomStart',10,'SoftTarget',1);

Req2=TuningGoal.StepTracking('r(1)','y(1)',0.25);
Req3=TuningGoal.StepTracking('r(2)','y(2)',0.25);
Req4=TuningGoal.StepTracking('r(3)','y(3)',0.50);

Req5 = TuningGoal.StepRejection('r(1)','y(2)',0.05,0.5);
Req6 = TuningGoal.StepRejection('r(1)','y(3)',0.05,0.5);

Req7= TuningGoal.StepRejection('r(2)','y(1)',0.05,0.5);%Quanto mais coloco restrições de rejeição mais as correntes se comportam de forma simétrica
Req8= TuningGoal.StepRejection('r(2)','y(3)',0.05,0.5);

Req9 = TuningGoal.StepRejection('r(3)','y(1)',0.05,0.5);
Req10 = TuningGoal.StepRejection('r(3)','y(2)',0.05,0.5);

Req11 = TuningGoal.LQG('r',{'u(1)', 'u(2)','u(3)','u(4)'}, 10, 0.01);

%[CL,fSoft] = systune(Planta,[Req,Req2,Req3, Req4, Req5, Req6],[],options);

[CL,fSoft] = systune(Planta,[Req,Req2,Req3,Req4,Req5,Req6,Req7,Req8,Req9,Req10,Req11],[],options);
K = ss(CL.Blocks.K) % representação do espaço de estado controlador

 figure
 viewGoal(Req,CL)

 figure
 viewGoal(Req2,CL)
  
 figure
 viewGoal(Req3,CL)
 % 
 figure
 viewGoal(Req4,CL)
 % 
 figure
 viewGoal(Req5,CL)

 figure
 viewGoal(Req6,CL)

 figure
 viewGoal(Req7,CL)

 figure
 viewGoal(Req8,CL)

  figure
 viewGoal(Req9,CL)

 figure
 viewGoal(Req10,CL)

 % figure
 % viewGoal(Req11,CL)
%% Controle digital
T = 0.005;
[KA,KB,KC,KD] = c2dm(K.A,K.B,K.C,K.D,T,'tustin')
