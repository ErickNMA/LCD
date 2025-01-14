X0=[deg2rad(50);0.215;371.6;0];%vetor de estados, deve ser coluna
IX=[1;2;3;4];%vetor que for�a que o trim seja feito exatamente sobre o valores dos estados fornecidos
U0=[];
IU=[];
Y0=[0];% informando um palpite inicial para sa�da, isso porque IY est� vazio
IY=[];

[sizes,x0,xstring] = modelo_linearizacao;% o vetor xstring contem o nome dos blocos associado a cada estado

[X,U,Y,DX]=trim('modelo_linearizacao',X0,U0,Y0,IX,IU,IY); %encontra todos os estados e valores de entrada e sa�da do ponto de opera��o

[A,B,C,D]=linmod('modelo_linearizacao',X,U);% execulta a lineariza��o
% %%
% % Matriz de observabilidade
% Ob=obsv(A,C)
% 
% V=eig(A);
% 
% for i=1:length(V)
%     if(V(i)>0)
%         disp('O sistema � instavel em malha aberta')
%         break
%     end
%     if(i==length(V))
%         disp('O sistema � estavel em malha aberta')
%     end
% end
% 
% % O sistema � Observavel?
% if(length(A)-rank(Ob))
%     disp('O sistema N�O � observavel');
% else
%     disp('O sistema � observavel');
% end
% 
% % Matriz de controlabilidade
% Co=ctrb(A,B)
% 
% % O sistema � controlavel?
% if( length(A)-rank(Co))
%     disp('O sistema N�O � controlavel');
% else
%     disp('O sistema � controlavel');
% end
% % http://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlStateSpace
% % http://ctms.engin.umich.edu/CTMS/index.php?example=AircraftPitch&section=ControlDigital
% % http://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlDigital
% 
% %% Controle em SS com apena ganho de realimenta��o
% 
% po_des= [-5;-5.1];
% 
% disp('Ganho de realimenta��o usando place')
% K = place(A,B,po_des)
% 
% disp('Ganho de realimenta��o usando Acker')
% Ka= acker(A,B,po_des)
% 
% disp('Ganho de realimenta��o aplicado')
% Ku = K
% 
% %Nbar= rscale(sys,K)
% Nbar= -inv(C*inv(A-B*Ku)*B)
% 
% %% Controle SS com ganho de realimenta��o e integrador
% 
% disp('Matriz A aumentada')
% 
% Aa= [A, zeros(size(A,1),size(C,1)); C zeros(size(C,1),size(C,1))]
% 
% Ba = [B;0] 
% 
% po_des_i= [-5;-5.1; -6];
% 
% disp('Ganho de realimenta��o usando Acker')
% Ka= acker(Aa,Ba,po_des_i)
% 
% Kui= Ka(1:2)
% 
% Ki= Ka(3)
% %% Controle Digital
% Ts=0.1;
% 
% sys= ss(A,B,C,D);
% 
% sysd= c2d(sys,Ts);
% 
% Ad=sysd.A
% Bd=sysd.B
% Cd=sysd.C
% 
% Aad= [Ad, zeros(size(Ad,1),size(Cd,1)); Ts*Cd eye(size(Cd,1),size(Cd,1))]
% 
% Bad = [Bd;0] 
% 
% %po_des_d=[0.5,0.51,0.52];
% po_des_d=exp(po_des_i*Ts)
% 
% disp('Ganho de realimenta��o usando Acker')
% Kad= acker(Aad,Bad,po_des_d)
% 
% Kuid= Kad(1:2)
% 
% Kid= Kad(3)
% 
% %%
% Ca=[C,0];
% sysa=ss(Aa,Ba,Ca,D)
% sysad=c2d(sysa,Ts)
