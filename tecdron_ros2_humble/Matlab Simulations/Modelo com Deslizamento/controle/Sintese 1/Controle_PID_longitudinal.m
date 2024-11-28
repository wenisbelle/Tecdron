%% Primeiramente, vamos criar a planata do sistema
clear
g=9.81;
uc_x=1.0;
uc_y=0.03;
theta=pi/4;
vlimite = 0.5;
l=0.70;
w=0.50;
m=100;
Iz = (1/12)*m*(l^2+w^2);
r=0.12;


A= [-(g*(4*uc_x + 4*uc_y))/(8*vlimite), 0, 0;
    0, -(g*(4*uc_x + 4*uc_y))/(8*vlimite),0;
    0, 0, -(g*m*(4*l^2*uc_x + 4*l^2*uc_y + 4*uc_x*w^2 + 4*uc_y*w^2 - 8*l*uc_x*w + 8*l*uc_y*w))/(8*Iz*vlimite)];
    
B = [(2^(1/2)*g*r*uc_x)/(8*vlimite), (2^(1/2)*g*r*uc_x)/(8*vlimite),(2^(1/2)*g*r*uc_x)/(8*vlimite),(2^(1/2)*g*r*uc_x)/(8*vlimite);
    (2^(1/2)*g*r*uc_x)/(8*vlimite),-(2^(1/2)*g*r*uc_x)/(8*vlimite),(2^(1/2)*g*r*uc_x)/(8*vlimite),-(2^(1/2)*g*r*uc_x)/(8*vlimite);
    (g*m*(2^(1/2)*l*r*uc_x - 2^(1/2)*r*uc_x*w))/(8*Iz*vlimite), -(g*m*(2^(1/2)*l*r*uc_x - 2^(1/2)*r*uc_x*w))/(8*Iz*vlimite), -(g*m*(2^(1/2)*l*r*uc_x - 2^(1/2)*r*uc_x*w))/(8*Iz*vlimite), (g*m*(2^(1/2)*l*r*uc_x - 2^(1/2)*r*uc_x*w))/(8*Iz*vlimite)];
   
B(3,:) = 7.5*B(3,:);

Ap = A;
Bp = B;
Cp=eye(3);
Dp=zeros(3,4);
 
%Construção da Planta
Plant = ss(Ap,Bp,Cp,Dp); %Planta
 
 %% 
 %Análise do controlador
s = zpk('s');

PI_vx = tunablePID('PI_vx','PID');
PI_vx.InputName='e(1)';
PI_vx.OutputName='c_vx';
PI_vx.Tf.Value = 0.01;
PI_vx.Tf.Free = false;
PI_vx.Kp.Value = 0;

PI_vy = tunablePID('PI_vy','PID');
PI_vy.InputName='e(2)';
PI_vy.OutputName='c_vy';
PI_vy.Tf.Value = 0.01;
PI_vy.Tf.Free = false;
PI_vy.Kp.Value = 0;


PI_vtta = tunablePID('PI_vtta','PID');
PI_vtta.InputName='e(3)';
PI_vtta.OutputName='c_vtta';
PI_vtta.Tf.Value = 0.01;
PI_vtta.Tf.Free = false;
PI_vtta.Kp.Value = 0;


Atf=zeros(1);
Btf=zeros(1,3);
Ctf=eye(4,1);
Dtf= [1 -1 -(l+w);
      1 1 (l+w);
      1 1 -(l+w);
      1 -1 (l+w)];

TF=ss(Atf,Btf,Ctf,Dtf);
TF.InputName={'c_vx','c_vy','c_vtta'};
TF.OutputName='u';


Plant.InputName='u';
Plant.OutputName='y';

Feedback = sumblk('e = r-y',3);

Planta=connect(Plant,TF,PI_vx,PI_vy,PI_vtta,Feedback,{'r'},{'u(1)', 'u(2)','u(3)','u(4)','y'});

options = systuneOptions('RandomStart',20,'SoftTarget',1);

Req2=TuningGoal.StepTracking('r(1)','y(1)',0.5);
Req3=TuningGoal.StepTracking('r(2)','y(2)',0.5);
Req4=TuningGoal.StepTracking('r(3)','y(3)',1.0);

Req5 = TuningGoal.StepRejection('r(1)','y(2)',0.05,0.5);
Req6 = TuningGoal.StepRejection('r(1)','y(3)',0.05,0.5);

Req7= TuningGoal.StepRejection('r(2)','y(1)',0.05,0.5);%Quanto mais coloco restrições de rejeição mais as correntes se comportam de forma simétrica
Req8= TuningGoal.StepRejection('r(2)','y(3)',0.05,0.5);

Req9 = TuningGoal.StepRejection('r(3)','y(1)',0.05,0.5);
Req10 = TuningGoal.StepRejection('r(3)','y(2)',0.05,0.5);

Req11 = TuningGoal.LQG('r',{'u(1)', 'u(2)','u(3)','u(4)'}, 10, 0.01);

[CL,fSoft] = systune(Planta,[Req2,Req3,Req4, Req11],[options]);


showTunable(CL)

% D = ss(CL.Blocks.D);
% PI_vx = ss(CL.Blocks.PI_vx);
% PI_vy = ss(CL.Blocks.PI_vy);
% PI_vtta = ss(CL.Blocks.PI_vtta); 
% 
% 
% 
%% 

  figure
  viewGoal(Req2,CL)
% 
  figure
  viewGoal(Req3,CL)
%  % 
  figure
  viewGoal(Req4,CL)
%  % 
%  figure
%  viewGoal(Req5,CL)
% 
%  figure
%  viewGoal(Req6,CL)
% 
%  figure
%  viewGoal(Req7,CL)
% 
%  figure
%  viewGoal(Req8,CL)
% 
%   figure
%  viewGoal(Req9,CL)
% 
%  figure
%  viewGoal(Req10,CL)
% 
%  figure
%  viewGoal(Req11,CL)

