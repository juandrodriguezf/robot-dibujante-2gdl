function motor = param_motor()
%param_motor  Parametros del Futaba S3003 para el modelo de P10.
%
%   motor = param_motor()
%
%   Modelo de motor DC de primer orden, referido al eje de salida del servo:
%
%       i      = (V - Ke*w) / R
%       J*dw/dt = Kt*i - tau_load
%       dtheta/dt = w
%
%   La inductancia L y el amortiguamiento viscoso B se desprecian para
%   obtener el modelo de primer orden aprobado para simulacion. El servo real
%   ya incluye su lazo de control; en la parte experimental se usa tal como se
%   compro (entrada PWM), y esa caracteristica se documenta en el informe.
%
%   Datos de la hoja de datos a 6 V:
%       par de bloqueo        tau_s = 4.1 kg.cm = 0.402 N.m
%       velocidad sin carga   w_nl  = 0.19 s/60 deg = 5.516 rad/s
%       tension nominal       V     = 6 V
%
%   En unidades SI se cumple Kt = Ke. Se toma Ke = V/w_nl y, para reproducir
%   el par de bloqueo, R = Kt*V/tau_s. La inercia no aparece en la hoja de
%   datos y se adopta un valor tipico de un servo pequeno (documentado abajo).
%
%   Salida: estructura motor con parametros electricos, mecanicos y limites.

    V     = 6.0;                    % V   (tension nominal)
    tau_s = 4.1 * 0.0980665;        % N.m (par de bloqueo, 4.1 kg.cm)
    w_nl  = (pi/3) / 0.19;          % rad/s (velocidad sin carga, 0.19 s/60 deg)

    motor.nombre = 'Futaba S3003';
    motor.Vmax   = V;
    motor.Ke     = V / w_nl;        % V.s/rad (~1.089)
    motor.Kt     = motor.Ke;        % N.m/A   (Kt = Ke en SI)
    motor.R      = motor.Kt * V / tau_s;   % ohm (~16.25)
    motor.L      = 0;               % H   (despreciada: modelo de primer orden)
    motor.J      = 3e-4;            % kg.m^2 (rotor + engranajes, estimado)
    motor.B      = 0;               % N.m.s/rad (despreciado)

    motor.qmin = -pi/2;             % rad (recorrido del servo, -90 deg)
    motor.qmax =  pi/2;             % rad (recorrido del servo, +90 deg)

    % Especificaciones derivadas
    motor.i_stall   = V / motor.R;                  % A   (~0.369)
    motor.tau_stall = motor.Kt * motor.i_stall;     % N.m (~0.402)
    motor.w_noload  = w_nl;                         % rad/s (~5.516)
    motor.K         = 1 / motor.Ke;                 % rad/s/V (~0.919)
    motor.tau_m     = motor.J * motor.R / ...
                      (motor.Kt * motor.Ke);        % s   (~4.1 ms)
end
