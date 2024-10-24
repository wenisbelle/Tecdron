clear
%declaração das variáveis simbólicas
syms Kt Cm c J reducao uc m g w l vlimite r Iz iR iL i3 i4 x y xp yp psi w1 w2 w3 w4 psip K 'real'
syms uc_x uc_y 'real'
syms A B 'array'

theta = pi/4;

u = [uc_x;uc_y];
% 1 and 3
Mecanum_FronLeft_RearRight= [cos(theta) -sin(theta);
                             sin(theta) cos(theta)];
% 2 and 4
Mecanum_FrontRight_RearLeft = [cos(theta) sin(theta);
                               -sin(theta) cos(theta)];

%Matriz de rotação do referencial do CG do chassi para o inercial
T=[cos(psi) -sin(psi) 0;sin(psi) cos(psi) 0;0 0 1];

%Velocidade de cada roda no referencial do carro, sendo xp e yp no
%referencial do veículo
V1=[xp;yp;0]+tilde([0;0;psip])*[l;w;0];
V1=V1(1:2);

V2=[xp;yp;0]+tilde([0;0;psip])*[l;-w;0];
V2=V2(1:2);

V3=[xp;yp;0]+tilde([0;0;psip])*[-l;-w;0];
V3=V3(1:2);

V4=[xp;yp;0]+tilde([0;0;psip])*[-l;w;0];
V4=V4(1:2);

%velocidades de deslizamento no referencial do da roda
Vd1=simplify(-Mecanum_FronLeft_RearRight'*V1+[w1*r;0]);
Vd2=simplify(-Mecanum_FrontRight_RearLeft'*V2+[w2*r;0]);
Vd3=simplify(-Mecanum_FronLeft_RearRight'*V3+[w3*r;0]);
Vd4=simplify(-Mecanum_FrontRight_RearLeft'*V4+[w4*r;0]);

%Matrizes da direção da força das rodas
 %%
%Força de atrito em cada ums das rodas no referencial da roda

F1w=((m*g/4)/(vlimite))*(Vd1.*u);
F2w=((m*g/4)/(vlimite))*(Vd2.*u);
F3w=((m*g/4)/(vlimite))*(Vd3.*u);
F4w=((m*g/4)/(vlimite))*(Vd4.*u);

%Forca no referencial do veiculo
F1 = Mecanum_FronLeft_RearRight*F1w;
F2 = Mecanum_FrontRight_RearLeft*F2w;
F3 = Mecanum_FronLeft_RearRight*F3w;
F4 = Mecanum_FrontRight_RearLeft*F4w;

%%
%Cálculo da força resultante no referencial momento 
%resultante no referencial do carro

F=simplify([F1;0]+[F2;0]+[F3;0]+[F4;0]);
M=tilde([l;w;0])*[F1;0]+tilde([l;-w;0])*[F2;0]+tilde([-l;-w;0])*[F3;0]+tilde([-l;w;0])*[F4;0];
M=simplify(M(3));

%Cálculo das acelerações no referencial do veículo
Acel=F/m-tilde([0;0;psip])*[xp;yp;0];
x2p=simplify(Acel(1));
y2p=simplify(Acel(2));
psi2p=simplify(M/Iz);

%Equação diferencial
EQDIFF_nlinear=[x2p;y2p;psi2p];
  
%% 
var = [xp;yp;psip;w1;w2;w3;w4];

for i=1:1:length(EQDIFF_nlinear)
    for j=1:1:length(var)
        aux=coeffs(EQDIFF_nlinear(i),var(j));
        naux=length(aux);
        if j<=3
            if naux==1
                A(i,j) = 0;
            else
                A(i,j) = aux(2);
            end
        else
            if naux==1
                B(i,(j-3)) = 0;
            else
                B(i,(j-3)) = aux(2);
            end
        end
    end
end

%% Descrevendo os valores
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


