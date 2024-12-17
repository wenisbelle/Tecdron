function [y1, y2, y3, y4, X_k] = test(u,X_k)

dt = 0.005;
i_error = [u(1);u(2);u(3)];

A = [-807.7,  222.7,    0,      0,       0;
          190.6, -531,  -13.15,    0,       0;
             0,  -9.577, -900.4,  234.2,     0;
             0,     0,    -69,   -364.2,  4.553;
             0,     0,     0,   0.9319,  -429.4];

B = [-1073,  -1684,  824.5;
           53.94,  897.4, -418.5;
          670.5,   449,  -195.4;
          576.1,  10.26, -30.86;
          502.8, -59.79, -2.678];

C = [21.93,  -18.87,  144.4,  -100.7,  13.85;
         75.43,  157.8,  -14.99,  -84.24,  278.4;
        -294.9, -514.5,  -39.34, -383.5,  -58.98;
         389.3,  648.5,  171.8,   170.8,   375.7];

i_X_k1 = A*X_k + B*i_error;
X_k1 = X_k + i_X_k1*dt;

Y = C*X_k1;
X_k = X_k1;

y1 =Y(1);
y2 =Y(2);
y3 =Y(3);
y4 =Y(4);