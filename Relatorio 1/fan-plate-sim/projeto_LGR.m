%Função de transferência do sistema no tempo continuo 
Gc=tf(1,[1,1]);

%Taxa de amostragem
Ts=0.1;

%Função de transferência do sistema no tempo discreto 
Gd=c2d(Gc,Ts);

%Comparação via degrau
step(Gc,Gd);
%%
%Toolbox de controle
sisotool(Gd)