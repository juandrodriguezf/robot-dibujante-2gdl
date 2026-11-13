function build_control_model()
%build_control_model  Construye motor_2gdl_ctrl.slx (Punto 11).
%
%   Toma el modelo de P10 (planta DC equivalente del S3003 + sensores) y le
%   agrega, DENTRO de cada articulacion, el control de bajo nivel:
%
%       V = sat_{+-Vmax} [ Kp*(theta_ref - theta_meas) + Kff*omega_ref ]
%
%   con muestreo Ts = 0.01 s (bloques Zero-Order Hold) y la posicion MEDIDA
%   (cuantizada, con ruido y saturada) como realimentacion. Ver param_control.m.
%
%   Entradas (From Workspace), en COORDENADAS DEL SERVO (q - q_off):
%       q1_ref, q2_ref   : referencia de posicion [rad]
%       w1_ref, w2_ref   : referencia de velocidad [rad/s]
%       tau1, tau2       : par de carga de P8 [N.m]
%
%   Salidas (To Workspace, timeseries), 7 columnas por articulacion:
%       y1, y2 = [theta_true omega_true i_true theta_meas omega_meas i_meas V]
%
%   Uso:
%       build_control_model;      % genera motor_2gdl_ctrl.slx y hace una prueba
%       test_control_bajo_nivel;  % pruebas individuales completas

    aqui = fileparts(mfilename('fullpath'));
    addpath(aqui);
    mo = param_motor();
    se = param_sensores();
    ct = param_control();

    mdl  = 'motor_2gdl_ctrl';
    ruta = fullfile(aqui, [mdl '.slx']);

    if bdIsLoaded(mdl)
        close_system(mdl, 0);
    end
    if exist(ruta, 'file')
        delete(ruta);
    end

    new_system(mdl);
    set_param(mdl, 'StopTime', '20', 'Solver', 'ode45', 'MaxStep', '1e-3');

    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Joint1'], ...
              'Position', [300 70 700 430]);
    construir_joint([mdl '/Joint1'], mo, se, ct, 12345);

    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Joint2'], ...
              'Position', [300 490 700 850]);
    construir_joint([mdl '/Joint2'], mo, se, ct, 67890);

    % Entradas: referencias y carga
    nombres = {'q1_ref', 'w1_ref', 'tau1'; 'q2_ref', 'w2_ref', 'tau2'};
    y0 = [100 420];
    for j = 1:2
        for k = 1:3
            add_block('simulink/Sources/From Workspace', ...
                [mdl '/' nombres{j,k}], ...
                'VariableName', nombres{j,k}, 'SampleTime', '0', ...
                'Interpolate', 'on', ...
                'OutputAfterFinalValue', 'Holding final value', ...
                'Position', [30 y0(j)+60*(k-1) 110 y0(j)+30+60*(k-1)]);
            add_line(mdl, [nombres{j,k} '/1'], ...
                     sprintf('Joint%d/%d', j, k), 'autorouting', 'on');
        end
    end

    % Registro de las siete salidas por articulacion
    add_block('simulink/Signal Routing/Mux', [mdl '/Mux1'], 'Inputs', '7', ...
        'Position', [760 100 765 400]);
    add_block('simulink/Signal Routing/Mux', [mdl '/Mux2'], 'Inputs', '7', ...
        'Position', [760 520 765 820]);
    add_block('simulink/Sinks/To Workspace', [mdl '/y1'], ...
        'VariableName', 'y1', 'SaveFormat', 'Timeseries', ...
        'Position', [820 220 880 260]);
    add_block('simulink/Sinks/To Workspace', [mdl '/y2'], ...
        'VariableName', 'y2', 'SaveFormat', 'Timeseries', ...
        'Position', [820 640 880 680]);
    for k = 1:7
        add_line(mdl, sprintf('Joint1/%d', k), sprintf('Mux1/%d', k), ...
                 'autorouting', 'on');
        add_line(mdl, sprintf('Joint2/%d', k), sprintf('Mux2/%d', k), ...
                 'autorouting', 'on');
    end
    add_line(mdl, 'Mux1/1', 'y1/1', 'autorouting', 'on');
    add_line(mdl, 'Mux2/1', 'y2/1', 'autorouting', 'on');

    save_system(mdl, ruta);
    fprintf('Modelo guardado: %s\n', ruta);

    % ------------------------------------------------ Verificacion rapida
    % Escalon de 30 deg en Joint1 a t = 0.2 s (Joint2 en reposo).
    tt  = [0; 0.1999; 0.2; 1.5];
    ref = deg2rad(30) * [0; 0; 1; 1];
    z   = zeros(4, 1);

    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('q1_ref', timeseries(ref, tt));
    in = in.setVariable('w1_ref', timeseries(z, tt));
    in = in.setVariable('tau1',   timeseries(z, tt));
    in = in.setVariable('q2_ref', timeseries(z, tt));
    in = in.setVariable('w2_ref', timeseries(z, tt));
    in = in.setVariable('tau2',   timeseries(z, tt));
    in = in.setModelParameter('StopTime', '1.5');
    out = sim(in);

    y1 = out.y1.Data;
    fprintf('\n--- Verificacion P11 (escalon 30 deg en Joint1) ---\n');
    fprintf('theta_true final = %+.3f deg (esperado +30)\n', rad2deg(y1(end, 1)));
    fprintf('V max            = %.2f V (limite %.1f)\n', max(abs(y1(:, 7))), mo.Vmax);
    fprintf('i_true max       = %.4f A (i_stall = %.4f)\n', max(abs(y1(:, 3))), mo.i_stall);
    fprintf('Salidas finitas  = %d\n', all(isfinite(y1(:))) && all(isfinite(out.y2.Data(:))));

    close_system(mdl, 0);
end


% --------------------------------------------------------------- FUNCIONES
function construir_joint(sys, mo, se, ct, seed)
%construir_joint  Planta + sensores (P10) + controlador (P11) de una articulacion.

    g  = @(x) num2str(x, '%.12g');
    dx = 460;                              % desplazamiento de la planta
    P  = @(p) p + [dx 0 dx 0];

    delete_block([sys '/In1']);
    delete_block([sys '/Out1']);

    % Entradas: theta_ref (1), omega_ref (2), tau_load (3)
    add_block('simulink/Sources/In1', [sys '/theta_ref'], ...
              'Position', [20 60 50 80]);
    add_block('simulink/Sources/In1', [sys '/omega_ref'], ...
              'Position', [20 170 50 190]);
    add_block('simulink/Sources/In1', [sys '/tau_load'], ...
              'Position', [20 330 50 350]);
    set_param([sys '/theta_ref'], 'Port', '1');
    set_param([sys '/omega_ref'], 'Port', '2');
    set_param([sys '/tau_load'],  'Port', '3');

    % Salidas: theta_true, omega_true, i_true, theta_meas, omega_meas, i_meas, V
    salidas = {'theta_true', 'omega_true', 'i_true', ...
               'theta_meas', 'omega_meas', 'i_meas', 'V'};
    ypos = [100 180 260 360 440 520 600];
    for k = 1:numel(salidas)
        add_block('simulink/Sinks/Out1', [sys '/' salidas{k}], ...
                  'Position', [1120+dx ypos(k) 1150+dx ypos(k)+20]);
        set_param([sys '/' salidas{k}], 'Port', num2str(k));
    end

    % ---------------- Controlador discreto (P + prealimentacion), P11
    add_block('simulink/Discrete/Zero-Order Hold', [sys '/zoh_ref'], ...
        'SampleTime', g(ct.Ts), 'Position', [100 55 150 85]);
    add_block('simulink/Discrete/Zero-Order Hold', [sys '/zoh_meas'], ...
        'SampleTime', g(ct.Ts), 'Position', [100 235 150 265]);
    add_block('simulink/Discrete/Zero-Order Hold', [sys '/zoh_ff'], ...
        'SampleTime', g(ct.Ts), 'Position', [100 165 150 195]);
    add_block('simulink/Math Operations/Sum', [sys '/sum_e'], ...
        'Inputs', '+-', 'IconShape', 'rectangular', ...
        'Position', [200 60 230 100]);
    add_block('simulink/Math Operations/Gain', [sys '/Kp'], ...
        'Gain', g(ct.Kp), 'Position', [260 60 310 100]);
    add_block('simulink/Math Operations/Gain', [sys '/Kff'], ...
        'Gain', g(ct.Kff), 'Position', [200 165 250 195]);
    add_block('simulink/Math Operations/Sum', [sys '/sum_u'], ...
        'Inputs', '++', 'IconShape', 'rectangular', ...
        'Position', [340 80 370 120]);
    add_block('simulink/Discontinuities/Saturation', [sys '/sat_V'], ...
        'UpperLimit', g(ct.Vmax), 'LowerLimit', g(-ct.Vmax), ...
        'Position', [395 85 435 115]);

    % ---------------- Planta DC de primer orden (P10)
    add_block('simulink/Math Operations/Sum', [sys '/sum_bemf'], ...
        'Inputs', '+-', 'IconShape', 'rectangular', ...
        'Position', P([120 80 150 120]));
    add_block('simulink/Math Operations/Gain', [sys '/Ke'], ...
        'Gain', g(mo.Ke), 'Position', P([200 170 250 210]));
    add_block('simulink/Math Operations/Gain', [sys '/invR'], ...
        'Gain', g(1/mo.R), 'Position', P([300 80 350 120]));
    add_block('simulink/Math Operations/Gain', [sys '/Kt'], ...
        'Gain', g(mo.Kt), 'Position', P([390 80 440 120]));
    add_block('simulink/Math Operations/Sum', [sys '/sum_tau'], ...
        'Inputs', '+-', 'IconShape', 'rectangular', ...
        'Position', P([470 80 500 120]));
    add_block('simulink/Math Operations/Gain', [sys '/invJ'], ...
        'Gain', g(1/mo.J), 'Position', P([540 80 590 120]));
    add_block('simulink/Continuous/Integrator', [sys '/int_w'], ...
        'Position', P([620 80 650 120]));
    add_block('simulink/Continuous/Integrator', [sys '/int_th'], ...
        'Position', P([700 80 730 120]));

    % ---------------- Sensor de posicion (P10)
    add_block('simulink/Math Operations/Bias', [sys '/pos_bias'], ...
        'Bias', g(se.pos_offset), 'Position', P([760 210 800 240]));
    add_block('simulink/Discontinuities/Quantizer', [sys '/quant'], ...
        'QuantizationInterval', g(se.pos_res), ...
        'Position', P([820 210 860 240]));
    add_block('simulink/Sources/Random Number', [sys '/pos_noise'], ...
        'Mean', '0', 'Variance', g(se.pos_noise^2), ...
        'Seed', num2str(seed), 'SampleTime', g(se.Ts), ...
        'Position', P([820 280 860 310]));
    add_block('simulink/Math Operations/Sum', [sys '/sum_pos_noise'], ...
        'Inputs', '++', 'IconShape', 'rectangular', ...
        'Position', P([880 210 910 250]));
    add_block('simulink/Discontinuities/Saturation', [sys '/sat_pos'], ...
        'UpperLimit', g(se.pos_max), 'LowerLimit', g(se.pos_min), ...
        'Position', P([940 210 980 240]));

    % ---------------- Sensor de corriente (P10)
    add_block('simulink/Continuous/Transfer Fcn', [sys '/cur_filter'], ...
        'Numerator', '[1]', ...
        'Denominator', ['[' g(1/(2*pi*se.cur_bw)) ' 1]'], ...
        'Position', P([760 340 820 380]));
    add_block('simulink/Math Operations/Bias', [sys '/cur_bias'], ...
        'Bias', g(se.cur_offset), 'Position', P([850 340 890 380]));
    add_block('simulink/Sources/Random Number', [sys '/cur_noise'], ...
        'Mean', '0', 'Variance', g(se.cur_noise^2), ...
        'Seed', num2str(seed+1), 'SampleTime', g(se.Ts), ...
        'Position', P([850 410 890 440]));
    add_block('simulink/Math Operations/Sum', [sys '/sum_cur_noise'], ...
        'Inputs', '++', 'IconShape', 'rectangular', ...
        'Position', P([920 340 950 380]));
    add_block('simulink/Discontinuities/Saturation', [sys '/sat_cur'], ...
        'UpperLimit', g(se.cur_max), 'LowerLimit', g(se.cur_min), ...
        'Position', P([980 340 1020 380]));

    % ---------------- Velocidad: derivada filtrada de la posicion medida
    add_block('simulink/Continuous/Transfer Fcn', [sys '/vel_filter'], ...
        'Numerator', '[1 0]', ...
        'Denominator', ['[' g(1/(2*pi*se.vel_fc)) ' 1]'], ...
        'Position', P([900 500 960 540]));

    % ---------------- Conexiones: controlador
    add_line(sys, 'theta_ref/1', 'zoh_ref/1',  'autorouting', 'on');
    add_line(sys, 'zoh_ref/1',   'sum_e/1',    'autorouting', 'on');
    add_line(sys, 'sat_pos/1',   'zoh_meas/1', 'autorouting', 'on');
    add_line(sys, 'zoh_meas/1',  'sum_e/2',    'autorouting', 'on');
    add_line(sys, 'sum_e/1',     'Kp/1',       'autorouting', 'on');
    add_line(sys, 'Kp/1',        'sum_u/1',    'autorouting', 'on');
    add_line(sys, 'omega_ref/1', 'zoh_ff/1',   'autorouting', 'on');
    add_line(sys, 'zoh_ff/1',    'Kff/1',      'autorouting', 'on');
    add_line(sys, 'Kff/1',       'sum_u/2',    'autorouting', 'on');
    add_line(sys, 'sum_u/1',     'sat_V/1',    'autorouting', 'on');
    add_line(sys, 'sat_V/1',     'sum_bemf/1', 'autorouting', 'on');
    add_line(sys, 'sat_V/1',     'V/1',        'autorouting', 'on');

    % ---------------- Conexiones: planta
    add_line(sys, 'Ke/1',       'sum_bemf/2', 'autorouting', 'on');
    add_line(sys, 'sum_bemf/1', 'invR/1',     'autorouting', 'on');
    add_line(sys, 'invR/1',     'Kt/1',       'autorouting', 'on');
    add_line(sys, 'Kt/1',       'sum_tau/1',  'autorouting', 'on');
    add_line(sys, 'tau_load/1', 'sum_tau/2',  'autorouting', 'on');
    add_line(sys, 'sum_tau/1',  'invJ/1',     'autorouting', 'on');
    add_line(sys, 'invJ/1',     'int_w/1',    'autorouting', 'on');
    add_line(sys, 'int_w/1',    'int_th/1',   'autorouting', 'on');
    add_line(sys, 'int_w/1',    'Ke/1',       'autorouting', 'on');
    add_line(sys, 'int_th/1',   'theta_true/1', 'autorouting', 'on');
    add_line(sys, 'int_w/1',    'omega_true/1', 'autorouting', 'on');
    add_line(sys, 'invR/1',     'i_true/1',   'autorouting', 'on');

    % ---------------- Conexiones: sensores
    add_line(sys, 'int_th/1',        'pos_bias/1',      'autorouting', 'on');
    add_line(sys, 'pos_bias/1',      'quant/1',         'autorouting', 'on');
    add_line(sys, 'quant/1',         'sum_pos_noise/1', 'autorouting', 'on');
    add_line(sys, 'pos_noise/1',     'sum_pos_noise/2', 'autorouting', 'on');
    add_line(sys, 'sum_pos_noise/1', 'sat_pos/1',       'autorouting', 'on');
    add_line(sys, 'sat_pos/1',       'theta_meas/1',    'autorouting', 'on');

    add_line(sys, 'invR/1',          'cur_filter/1',    'autorouting', 'on');
    add_line(sys, 'cur_filter/1',    'cur_bias/1',      'autorouting', 'on');
    add_line(sys, 'cur_bias/1',      'sum_cur_noise/1', 'autorouting', 'on');
    add_line(sys, 'cur_noise/1',     'sum_cur_noise/2', 'autorouting', 'on');
    add_line(sys, 'sum_cur_noise/1', 'sat_cur/1',       'autorouting', 'on');
    add_line(sys, 'sat_cur/1',       'i_meas/1',        'autorouting', 'on');

    add_line(sys, 'sat_pos/1',       'vel_filter/1',    'autorouting', 'on');
    add_line(sys, 'vel_filter/1',    'omega_meas/1',    'autorouting', 'on');
end
