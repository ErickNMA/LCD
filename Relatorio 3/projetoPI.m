%Sistema linerarizado em torno do ponto de operação:
Glin = tf([8.74e5], [1 1802 5.842e5 4.173e6 7.389e6]);
poles = pole(Glin);
Gsimp = tf(Glin.num, conv([1 -poles(3)], [1 -poles(4)]));
Gsimp = Gsimp*dcgain(Glin)/dcgain(Gsimp);
figure
hold
step(Glin);
step(Gsimp);

%Sistema contínuo, em malha aberta:
[z,p,k] = zpkdata(Gsimp); %formato zpk
Gzpk = zpk(z,p,k,'DisplayFormat','timeconstant'); %formato em constante de tempo
figure
step(Gsimp); %degrau em malha aberta
Ginfo = stepinfo(Gsimp); %características do transitório
disp(Ginfo);

%Especificações de projeto:
disp('Especificações do Projeto:');
ST = 0.75*Ginfo.SettlingTime;
fprintf('ST < %g\n', ST); %redução do tempo de acomodação em 25% da malha aberta
disp('OS < 5%'); %overshoot ótimo de 5%
T = 1/(5*max(abs(p{1})));
fprintf('T = %g\n', T); %intervalo de amostragem: 1/5 da constante de tempo mais rápida

%Discretização do sistema, com ZOH:
Gz = c2d(Gsimp, T, 'zoh');

%Projeto de controlador proporcional (tempo de acomodação):
%sisotool(Gz);
load('PI.mat');