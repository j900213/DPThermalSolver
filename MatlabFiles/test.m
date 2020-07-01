Cth = 113e3;
Rth = 15e-6;
Qsun = 469.05;
Qpas = 416;
Cp = 1.0035e3;
rho = 1.1839;
mDot = 0.0842;
CoP_pos = 2.14;
CoP_neg = -2.14;
Tamb = 28;

dt = 0.43;

Xin = 27.959184;
Uin = -60000;

Xnext = Xin + (dt / Cth) * (Uin + Qsun + Qpas + (Tamb - Xin) / Rth);

Phvac = Uin / -2.14;