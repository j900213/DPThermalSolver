Cth = 1.1347e5;
Rth = 0.015;
Qsun = 150*1.7725;
Qpas = 4*104.2975;
Cp = 1.0035e3;
rho = 1.1839;
mDot = 0.0842;
CoP_pos = 2.1379;
CoP_neg = -2.1379;
Tamb = 28;
T_required = 26;

dt = 4.339624;

Xin = 25.058322;
Uin = 494.3322;

Xnext = Xin + (dt / Cth) * (Uin + Qsun + Qpas + (Tamb - Xin) / Rth);

Phvac = Uin / -2.14;
ArcCost = 10e6 * (Xnext - T_required)^2;

Tinlet = Xin + Uin / (Cp * rho * mDot);