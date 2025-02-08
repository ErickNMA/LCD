load('var.mat')
load('PI.mat')
load('PID.mat')
load('T')
kpid1 = pid(C, 'IFormula', 'Backward');
kp1 = kpid1.kp;
ki1 = kpid1.ki;
kd1 = kpid1.kd;
kpid2 = pid(Cpid, 'IFormula', 'Backward');
kp2 = kpid2.kp;
ki2 = kpid2.ki;
kd2 = kpid2.kd;
umin = 0;
umax = 16;