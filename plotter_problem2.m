% problem #01
%solved
%sign switching in dd neccessary for the calculation of t1 and t2
%solution: Indroduced special test which takes care of the switching.
figure;
hold off;
newplot;
subplot(1,3,1);
calc7st(2, 3, 3, 3, -2, 2, 0,true);
calc7st(2.5, 3, 3, 3, -2, 2, 0,true);
calc7st(3, 3, 3, 3, -2, 2, 0,true);
axis([0, 3, -3.5, 3.5]);grid on;
subplot(1,3,2);
calc7st(1.5, 3, 3, 3, -2, 2, 0,true);
axis([0, 3, -3.5, 3.5]);grid on;
subplot(1,3,3);
calc7st(0.88, 3, 3, 3, -2, 2, 0,true);
calc7st(0.75, 3, 3, 3, -2, 2, 0,true);
calc7st(0, 3, 3, 3, -2, 2, 0,true);
axis([0, 3, -3.5, 3.5]);grid on;

% problem #02
%solved
% more than one real positive solution to the dd-polinomial.
% solution: test for the right one
%calc7st(2, 1.5, 8, 20, -5, -3, 0,true)
%calc7st(-3, 1, 9, 20, 4, 3, 0,true)

% problem #03
% solved
% v0 > vmax, no cruise
% solution: introduced a test and a recursive call after some slowing down
% calc7st(-121.168044, 2.206288, 15, 10.290294, 0, -25.7449, -11.0832,true)
% calc7st(-121.168044, 2.206288, 4.75, 10.290294, 0, -25.7449, -11.0832,true)

% problem #04
% solved
%v > vmax between  t0 and  t1
% Es ist folgendes problem aufgetreten:
% in calc7st_nocruise wird sofort der DD fall angenommen, obwohl eigentlich TD voliegt.
% Grund ist, dass das v zwischen 0 und t1 vmax überschreitet und somit beide dreiecke im
% TT profil oberhalb der x-achse liegen.
% solution: introduced a test and a recursive call after some slowing down
%calc7st(-121.168044, 2.206288, 9.957211, 10.290294, -9.915842, -3.462254, 71.241503,true)

% problem #05
% solved
% dd-gleichung liefert keine positiven, reellen Ergebnisse
% solution: don't call dd, when overshooting and an imidiate halt already
% leads to a trapezoid profile. (We are sure already thats not DD then and
% trying to solve DD could lead to no solution, because the dir-flag might
% switch sign).
% calc7st(-29, 2.1, 3, 16, 0, -11, 0,true)
% calc7st(-28.05, 2.1, 3, 16, 0, -11, 0,true)
% calc7st(-27.5, 2.1, 3, 16, 0, -11, 0,true)
% calc7st(-26.5, 2.1, 3, 16, 0, -11, 0,true)
% calc7st(-25.5, 2.1, 3, 16, 0, -11, 0,true)
% calc7st(-25, 2.1, 3, 16, 0, -11, 0,true)
% calc7st(-24, 2.1, 3, 16, 0, -11, 0,true)

% problem #06
% solved
% at v0>vmax and double downside profile case the second triangle should be
% a trapez instead --> amax limit is broken!!!
% The source of the problem: Although the v0->vmax->0 profile is a DD
% profile, in the end we have a TD profile.
% calc7st(4.228053, 0.857550, 1.727873, 3.105438, -0.405021, 5.763450, -10.239185,true)

% minor problem #07
% open
% rounding error: 10e-4
% calc7st(73.808957, 1.560508, 5.376940, 8.423299, -7.342873, 7.172009, 51.499119,true)
% calc7st(4,1.5,2,2,1.5,1.5,-4,true)

% problem #8
% solved
% not optimal profile because calc3st was given back the time intervalls in
% a format different than expected
% solution: Use shrink_t_j to cut out zero-length time intervalls and then
% check for the length.
% calc7st(4.228053, 0.857550, 1.727873, 3.105438, 0, 5, -10.239185,true)

% problem #9
% open
% negative time intervalls given back
calc7st(-0.607396, 4.091768, 0.827334, 0.201081, 1.055669, -0.264881, -0.543558, true)