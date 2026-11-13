function ctrl = param_control()
%param_control  Parametros del control de bajo nivel (P11) de cada articulacion.
%
%   ctrl = param_control()
%
%   Ley de control por articulacion (discreta, periodo Ts):
%
%       V = sat_{+-Vmax} [ Kp*(theta_ref - theta_meas) + Kff*omega_ref ]
%
%   El primer termino reproduce el lazo proporcional de posicion que ya trae
%   el servo S3003. El segundo es una prealimentacion de velocidad (Kff = Ke)
%   que compensa la fuerza contraelectromotriz y reduce el error en rampa.
%
%   Sintonia: con L = B = 0 la planta es theta/V = K / ( s (tau_m s + 1) ).
%   En lazo cerrado con Kp:   tau_m s^2 + s + K*Kp = 0, de donde
%       wn   = sqrt(K*Kp/tau_m)
%       zeta = 1/(2*sqrt(K*Kp*tau_m))
%   Se elige K*Kp = 40 1/s  ->  Kp = 43.5 V/rad, zeta = 1.23 (sobreamortiguado).
%
%   Offset de montaje q_off: el servo gira +-90 deg alrededor de su centro,
%   pero la trayectoria de P7 exige q2 hasta 128.9 deg (q1: 43.7 a 76.4 deg).
%   Por eso el horn de cada servo se monta de modo que el centro del servo
%   coincida con q_off (centro del rango de la trayectoria, redondeado a 5 deg):
%       theta_servo = q_articulacion - q_off
%   El modelo Simulink trabaja en coordenadas del servo (theta_servo).

    mo = param_motor();
    se = param_sensores();

    ctrl.Ts   = se.Ts;                 % s
    ctrl.KKp  = 40;                    % 1/s  (K*Kp)
    ctrl.Kp   = ctrl.KKp / mo.K;       % V/rad  (~43.5)
    ctrl.Kff  = mo.Ke;                 % V/(rad/s)
    ctrl.Vmax = mo.Vmax;               % V
    ctrl.zeta = 1 / (2*sqrt(ctrl.KKp * mo.tau_m));
    ctrl.wn   = sqrt(ctrl.KKp / mo.tau_m);   % rad/s

    ctrl.q_off = deg2rad([60; 95]);    % rad (Joint1; Joint2)
end
