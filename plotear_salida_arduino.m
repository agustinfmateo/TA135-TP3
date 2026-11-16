datos = readmatrix('datos.txt');

Ts=20e-3;

x = datos(:, 1);
y = datos(:, 2);
z = datos(:, 3);
t = (0:length(x)-1)*Ts ;

figure;
hold on
plot(t,x,LineWidth=2)
plot(t,y,LineWidth=2,LineStyle="--")
plot(t,z,LineWidth=2)
legend('Theta','Theta_e','u')
hold off
grid on