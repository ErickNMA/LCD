%Fun��o de transfer�ncia do sistema no tempo continuo 
Gc=tf(1,[1,1]);

%Taxa de amostragem
Ts=0.1;

%Fun��o de transfer�ncia do sistema no tempo discreto 
Gd=c2d(Gc,Ts);

%Compara��o via degrau
step(Gc,Gd);
%%
%Toolbox de controle
sisotool(Gd)