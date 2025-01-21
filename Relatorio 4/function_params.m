load('var.mat')
load('PI.mat')
load('T')
kpid = pid(C, 'IFormula', 'Backward');
kp = kpid.kp;
ki = kpid.ki;
kd = kpid.kd;
umin = 0;
umax = 14;