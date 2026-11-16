%% Control Moderno
clear all;close all;clc
s = tf('s');
Ts = 20e-3;
%% 1a. Sistema
ceros = [0, 90.01];
polos = [-0.4850 + 9.5599i, -0.4850 - 9.5599i];
ganancia = 73.03/1.2390e+03;
 
%         73.03 s (s-90.01)
%  --------------------------------
%  (s+35.2)^2 (s^2 + 0.97s + 91.63)

Phi = zpk([], [-35.2, -35.2],  1.2390e+03);
Theta = zpk(ceros, polos,  ganancia);

P= Phi*Theta
[Aa, Ba, Ca, Da] = ssdata(P);

%Transformación a forma canonica observable
T = [Ca;          % fila 1: phi
     Ca * Aa;     % fila 2: phi_dot
     Ca * (Aa^2); % fila 3: theta
     Ca * (Aa^3)];% fila 4: theta_dot


% Nuevos estados fisicos: x = T*z  =>  z = T^-1*x
% Nueva dinamica: x_dot = T*Aa*T^-1*x + T*Ba*u ; y = Ca*T^-1*x
Ac = (T * Aa) / T;
Bc = T * Ba;
Cc = Ca / T; % equivalente a Ca*inv(T), forma numericamente mas estable
Dc = Da;

disp('--- Matrices en estados fisicos [phi, phi_prima, theta, theta_prima] ---');
disp('A ='); disp(Ac);
disp('B ='); disp(Bc);
disp('C ='); disp(Cc);
n = size(Ac,1);  % orden del sistema

%% 1b. Test de controlabilidad y observabilidad
Co = ctrb(Ac, Bc);
Ob = obsv(Ac, Cc);

fprintf('Rango de la matriz de controlabilidad: %d\n', rank(Co));
fprintf('Rango de la matriz de observabilidad:  %d\n', rank(Ob));

if rank(Co) == size(Ac,1)
    disp('✅ La planta es completamente controlable.');
else
    disp('❌ La planta NO es completamente controlable.');
end

if rank(Ob) == size(Ac,1)
    disp('✅ La planta es completamente observable.');
else
    disp('❌ La planta NO es completamente observable.');
end


%% 1c. Sistema Discreto

%      0.0052981 (z-6.051) (z-1)
% ------------------------------------
% (z-0.4946)^2 (z^2 - 1.945z + 0.9808)


 %Pd = zpk([6.051,  1], [0.4946, 0.4946, 0.9725 + 0.1872i, 0.9725 - 0.1872i], 0.0052981, Ts)

 %[Ab, Bb, Cb, Db] = ssdata(Pd)

 %Transformación a forma canonica observable
%Td = [Cb;          % fila 1: phi
 %    Cb * Ab;     % fila 2: phi_dot
  %   Cb * (Ab^2); % fila 3: theta
   %  Cb * (Ab^3)];% fila 4: theta_dot

%Td = 1;

% Nuevos estados fisicos: x = T*z  =>  z = T^-1*x
% Nueva dinamica: x_dot = T*Aa*T^-1*x + T*Ba*u ; y = Ca*T^-1*x
%Ad = Td * Ab * inv(Td); %#ok<MINV>
%Bd = Td * Bb;
%Cd = Cb / Td; % equivalente a Ca*inv(T), forma numericamente mas estable
%Dd = Db;

%Pd = c2d(ss(Ac, Bc, Cc, Dc), Ts);
%Pd = c2d(P,Ts,'zoh');
%[Ad, Bd, Cd, Dd] = ssdata(Pd)

%Usamos directamente la planta identificada discreta
load('planta_identificada.mat')
Pd = sys_ideal;
[Ad, Bd, Cd, Dd] = ssdata(sys_ideal)

%% 2 Ley de control calculo de K continuo y discreto

% polinomio deseado controlador en continuo 
% polinomio deseado controlador en discreto z=exp(sTs)
p1c=-35 ;p2c=-35.2; p3c=-5.5 + 9.5599i ; p4c=-5.5 - 9.5599i;
p1= exp(p1c*Ts) ;p2= exp(p2c*Ts) ; p3= exp(p3c*Ts) ;p4= exp(p4c*Ts) ;
polyc =[p1c ;p2c;p3c;p4c] ;%polos deseados continuo
polyd =[p1 ;p2;p3;p4] ;%polos deseados discretos

% Realimentacion de estados
Kc = place(Ac,Bc,polyc);
Kd = place(Ad,Bd,polyd);

%tA=Ac-Bc*Kc; % Nueva matriz A del sistema controlado
%tC=Cc-Dc*Kc; % Nueva matriz C del sistema controlado

disp('Polos controlador discreto:'); disp(eig(Ad - Bd*Kd));

% 3a. polinomio del observador en cont y disc poly = (s+45)^4  z=exp(sTs)
pec = -45;

polyec =[pec pec pec pec] ;
Lc = acker(Ac.',Cc.',polyec).';

polyed = exp([-35, -36, -37, -38] * Ts);
Ld = place(Ad.', (Cd * Ad).', polyed).';

x0=[0 0 0 0];

disp('Polos observador discreto actual:'); disp(eig(Ad - Ld*Cd*Ad));

% 3b. Armado del observador
Boc =[Bc Lc];       
Aoc =[Ac - Lc*Cc];
Bo =[Bd Ld];
Ao =[Ad - Ld*Cd];
Chat=eye(length(Ac));
Do =[0 0;0 0;0 0;0 0];
Bhat =eye(length(Ac));
observadorc=ss(Aoc,Boc,Chat,Do);%observador continuo
[Aocc,Bocc,Cocc,Docc]= ssdata(observadorc);
%observadord=ss(Ao,Bo,Chat,Do,Ts);%observador discreto

Ao = Ad - Ad*Ld*Cd;           
Bo = [Bd, Ad*Ld];                
Co = eye(size(Ad)) - Ld*Cd;
Do = [zeros(size(Bd)), Ld];
observadord = ss(Ao, Bo, Co, Do, Ts);

% 3c. Armado del observador para lo utilizado en arduino
I = eye(size(Ad));

Aodd = (I - Ld * Cd) * Ad;
Bodd = [(I - Ld * Cd) * Bd, Ld];   % Columna 1: multiplica a u[k-1] | Columna 2: multiplica a y[k] (theta)
Codd = Cd;
Dodd = [0, 0];

% 3.d Exportar valores a ESP32
fprintf('\n// --- PEGAR EN ARDUINO IDE ---\n');

for i = 1:4
    fprintf('float A_fila%d[4] = {%.10g, %.10g, %.10g, %.10g};\n', ...
            i, Aodd(i,1), Aodd(i,2), Aodd(i,3), Aodd(i,4));
end

fprintf('\n');
for i = 1:4
    fprintf('float B_fila%d[2] = {%.10g, %.10g};\n', ...
            i, Bodd(i,1), Bodd(i,2));
end

fprintf('\nfloat Cd[4] = {%.10g, %.10g, %.10g, %.10g};\n', ...
        Cd(1), Cd(2), Cd(3), Cd(4));

fprintf('float K[4]  = {%.10g, %.10g, %.10g, %.10g};\n', ...
        Kd(1), Kd(2), Kd(3), Kd(4));

fprintf('// ----------------------------------\n\n');

%% 4. Respuestas temporales

figure;
step(P);
title("Respuesta al escalón continua");

figure;
step(Pd);
title("Respuesta al escalón discreta");

figure;
impulse(P);
title("Respuesta al impulso continua");

figure;
impulse(Pd);
title("Respuesta al impulso discreta");

%% 5. Respuesta del sistema controlado

A = Ad;
B = Bd;
C = Cd;
D = Dd;
kI = 0;
K = Kd;
L = Ld;
kr = 1;

A_total = [ A,       -B*kI,         -B*K;
           -C,         0,      zeros(1,n);
           L*C,     -B*kI,   A - B*K - L*C ];

B_total = [ B*kr;
            1;
            B*kr ];

C_total = [ C, 0, zeros(1,n) ];
D_total = 0;

%Sistema en espacio de estados cerrado
sys_cl = ss(A_total, B_total, C_total, D_total, Ts);

%Simulación
t = 0:Ts:10;
r = ones(size(t));  % entrada escalón

[y, t_out] = lsim(sys_cl, r, t);

%Gráfica
figure;
plot(t_out, y, 'LineWidth', 2);
grid on;
xlabel('Tiempo (s)');
ylabel('Salida y(t)');
title('Respuesta con observador y seguimiento');


%% 
I4 = eye(4);
A_cl = [ Ad,          -Bd*Kd;
         Ld*Cd*Ad,    (I4-Ld*Cd)*Ad - Bd*Kd ];
eig(A_cl)

B1 = (I4-Ld*Cd)*Bd;  B2 = Ld;
Acx = Aodd - B1*Kd;
Ctrl = ss(Acx, B2, -Kd*Acx, -Kd*B2, Ts);   % theta[k] -> u[k]
bode(Ctrl); grid on

close all
