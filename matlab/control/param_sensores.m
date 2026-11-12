function sens = param_sensores()
%param_sensores  Parametros de los sensores del modelo de P10.
%
%   sens = param_sensores()
%
%   El S3003 no expone su potenciometro ni la corriente del motor; estos
%   modelos representan sensores genericos de posicion y corriente para el
%   lazo de control de P11. La resolucion, el ruido y el ancho de banda son
%   supuestos documentados, no datos publicados del servo.
%
%   Salida: estructura sens con el periodo de muestreo y los parametros de
%   los sensores de posicion, corriente y velocidad.

    sens.Ts = 0.01;                 % s (periodo de muestreo, igual a P7/P8)

    % Sensor de posicion (potenciometro/encoder generico)
    sens.pos_min   = -pi/2;         % rad
    sens.pos_max   =  pi/2;         % rad
    sens.pos_res   = 0.5*pi/180;    % rad (resolucion supuesta, 0.5 deg)
    sens.pos_noise = 0.05*pi/180;   % rad (desviacion estandar supuesta)
    sens.pos_offset = 0;            % rad

    % Sensor de corriente (shunt/Hall generico)
    sens.cur_gain   = 1;            % A/A
    sens.cur_offset = 0;            % A
    sens.cur_noise  = 0.002;        % A (desviacion estandar supuesta)
    sens.cur_bw     = 100;          % Hz (ancho de banda supuesto)
    sens.cur_min    = -1;           % A
    sens.cur_max    =  1;           % A

    % Velocidad derivada de la posicion (filtro pasa-bajos)
    sens.vel_fc = 20;               % Hz (frecuencia de corte supuesta)
end
