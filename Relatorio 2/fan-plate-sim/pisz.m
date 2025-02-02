Pu = 131e-3;
kc = 27;
kp = 0.6*kc*10;
ki = 100*2/Pu;
kd = 0.125*Pu;
Gpid = tf([kd kp ki], [1 10000 0]);
T = 2*Pu/10;
Gpidz = c2d(Gpid, T);
kpid = pid(Gpid);
kpidz = pid(Gpidz);