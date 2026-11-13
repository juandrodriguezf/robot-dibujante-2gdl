function build_motor_model()
%build_motor_model  Construye motor_2gdl.slx (Punto 10).
%
%   Modela cada articulacion como un motor DC de primer orden referido al eje
%   de salida del servo, con sensores de posicion, velocidad y corriente:
%
%       i       = (V - Ke*w) / R
%       J*dw/dt = Kt*i - tau_load
%       dtheta/dt = w
%
%   No se modelan la inductancia ni el amortiguamiento viscoso (L = B = 0),
%   de acuerdo con el modelo de primer orden aprobado. El servo real ya trae
%   su lazo interno; en la parte experimental se usa tal como se compro.
%
%   El modelo usa solo bloques base de Simulink (no requiere Simscape).
%
%   Entradas (From Workspace):
%       V1, V2       : tension de cada articulacion [V]
%       tau1, tau2   : par de carga de cada articulacion [N.m]
%
%   Salidas (To Workspace, formato timeseries):
%       y1, y2 = [theta_true omega_true i_true theta_meas omega_meas i_meas]
%
%   Uso:
%       build_motor_model;
%       % definir V1, V2, tau1, tau2 y simular el modelo.

    aqui = fileparts(mfilename('fullpath'));
    addpath(aqui);
    mo = param_motor();
    se = param_sensores();

    mdl  = 'motor_2gdl';
    ruta = fullfile(aqui, [mdl '.slx']);

    if bdIsLoaded(mdl)
        close_system(mdl, 0);
    end
    if exist(ruta, 'file')
        delete(ruta);
    end

    new_system(mdl);
    set_param(mdl, 'StopTime', '2', 'Solver', 'ode45', 'MaxStep', '1e-3');

    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Joint1'], ...
              'Position', [140 70 520 430]);
    construir_joint([mdl '/Joint1'], mo, se, 12345);

    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Joint2'], ...
              'Position', [140 490 520 850]);
    construir_joint([mdl '/Joint2'], mo, se, 67890);

    % Entradas de prueba
    add_block('simulink/Sources/From Workspace', [mdl '/V1'], ...
        'VariableName', 'V1', 'SampleTime', '0', 'Interpolate', 'on', ...
        'OutputAfterFinalValue', 'Holding final value', ...
        'Position', [30 100 100 130]);
    add_block('simulink/Sources/From Workspace', [mdl '/tau1'], ...
        'VariableName', 'tau1', 'SampleTime', '0', 'Interpolate', 'on', ...
        'OutputAfterFinalValue', 'Holding final value', ...
        'Position', [30 160 100 190]);
    add_block('simulink/Sources/From Workspace', [mdl '/V2'], ...
        'VariableName', 'V2', 'SampleTime', '0', 'Interpolate', 'on', ...
        'OutputAfterFinalValue', 'Holding final value', ...
        'Position', [30 520 100 550]);
    add_block('simulink/Sources/From Workspace', [mdl '/tau2'], ...
        'VariableName', 'tau2', 'SampleTime', '0', 'Interpolate', 'on', ...
        'OutputAfterFinalValue', 'Holding final value', ...
        'Position', [30 580 100 610]);

    add_line(mdl, 'V1/1',    'Joint1/1', 'autorouting', 'on');
    add_line(mdl, 'tau1/1',  'Joint1/2', 'autorouting', 'on');
    add_line(mdl, 'V2/1',    'Joint2/1', 'autorouting', 'on');
    add_line(mdl, 'tau2/1',  'Joint2/2', 'autorouting', 'on');

    % Registro de las seis salidas por articulacion
    add_block('simulink/Signal Routing/Mux', [mdl '/Mux1'], 'Inputs', '6', ...
        'Position', [570 100 575 400]);
    add_block('simulink/Signal Routing/Mux', [mdl '/Mux2'], 'Inputs', '6', ...
        'Position', [570 520 575 820]);
    add_block('simulink/Sinks/To Workspace', [mdl '/y1'], ...
        'VariableName', 'y1', 'SaveFormat', 'Timeseries', ...
        'Position', [620 220 680 260]);
    add_block('simulink/Sinks/To Workspace', [mdl '/y2'], ...
        'VariableName', 'y2', 'SaveFormat', 'Timeseries', ...
        'Position', [620 640 680 680]);

    for k = 1:6
        add_line(mdl, sprintf('Joint1/%d', k), sprintf('Mux1/%d', k), ...
                 'autorouting', 'on');
        add_line(mdl, sprintf('Joint2/%d', k), sprintf('Mux2/%d', k), ...
                 'autorouting', 'on');
    end
    add_line(mdl, 'Mux1/1', 'y1/1', 'autorouting', 'on');
    add_line(mdl, 'Mux2/1', 'y2/1', 'autorouting', 'on');

    save_system(mdl, ruta);
    fprintf('Modelo guardado: %s\n', ruta);

    % ------------------------------------------------ Verificacion
    tt = [0; 0.0999; 0.1; 0.2];
    V1   = timeseries([0; 0; 6; 6], tt);
    V2   = timeseries([0; 0; 0; 0], tt);
    tau1 = timeseries([0; 0; 0; 0], tt);
    tau2 = timeseries([0; 0; 0; 0], tt);

    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('V1', V1);
    in = in.setVariable('V2', V2);
    in = in.setVariable('tau1', tau1);
    in = in.setVariable('tau2', tau2);
    in = in.setModelParameter('StopTime', '0.2');
    out = sim(in);

    y1 = out.y1.Data;
    y2 = out.y2.Data;

    fprintf('\n--- Verificacion P10 (V1 = 6 V desde t = 0.1 s) ---\n');
    fprintf('omega_true final = %+.3f rad/s (esperado ~5.51)\n', y1(end, 2));
    fprintf('theta_true final = %+.3f rad\n', y1(end, 1));
    fprintf('i_true final     = %+.4f A\n', y1(end, 3));
    fprintf('theta_meas final = %+.3f rad\n', y1(end, 4));
    fprintf('omega_meas final = %+.3f rad/s\n', y1(end, 5));
    fprintf('i_meas final     = %+.4f A\n', y1(end, 6));
    fprintf('Salidas finitas  = %d\n', all(isfinite(y1(:))) && all(isfinite(y2(:))));

    close_system(mdl, 0);
end


% --------------------------------------------------------------- FUNCIONES
function construir_joint(sys, mo, se, seed)
%construir_joint  Arma la planta y los sensores de una articulacion.

    g = @(x) num2str(x, '%.12g');

    delete_block([sys '/In1']);
    delete_block([sys '/Out1']);

    % Entradas
    add_block('simulink/Sources/In1', [sys '/V'], ...
              'Position', [20 80 50 100]);
    add_block('simulink/Sources/In1', [sys '/tau_load'], ...
              'Position', [20 150 50 170]);
    set_param([sys '/V'], 'Port', '1');
    set_param([sys '/tau_load'], 'Port', '2');

    % Salidas: theta_true, omega_true, i_true, theta_meas, omega_meas, i_meas
    salidas = {'theta_true', 'omega_true', 'i_true', ...
               'theta_meas', 'omega_meas', 'i_meas'};
    ypos = [100 180 260 360 440 520];
    for k = 1:numel(salidas)
        add_block('simulink/Sinks/Out1', [sys '/' salidas{k}], ...
                  'Position', [1120 ypos(k) 1150 ypos(k)+20]);
        set_param([sys '/' salidas{k}], 'Port', num2str(k));
    end

    % Planta DC de primer orden
    add_block('simulink/Math Operations/Sum', [sys '/sum_bemf'], ...
        'Inputs', '+-', 'IconShape', 'rectangular', ...
        'Position', [120 80 150 120]);
    add_block('simulink/Math Operations/Gain', [sys '/Ke'], ...
        'Gain', g(mo.Ke), 'Position', [200 170 250 210]);
    add_block('simulink/Math Operations/Gain', [sys '/invR'], ...
        'Gain', g(1/mo.R), 'Position', [300 80 350 120]);
    add_block('simulink/Math Operations/Gain', [sys '/Kt'], ...
        'Gain', g(mo.Kt), 'Position', [390 80 440 120]);
    add_block('simulink/Math Operations/Sum', [sys '/sum_tau'], ...
        'Inputs', '+-', 'IconShape', 'rectangular', ...
        'Position', [470 80 500 120]);
    add_block('simulink/Math Operations/Gain', [sys '/invJ'], ...
        'Gain', g(1/mo.J), 'Position', [540 80 590 120]);
    add_block('simulink/Continuous/Integrator', [sys '/int_w'], ...
        'Position', [620 80 650 120]);
    add_block('simulink/Continuous/Integrator', [sys '/int_th'], ...
        'Position', [700 80 730 120]);

    % Sensor de posicion
    add_block('simulink/Math Operations/Bias', [sys '/pos_bias'], ...
        'Bias', g(se.pos_offset), 'Position', [760 210 800 240]);
    add_block('simulink/Discontinuities/Quantizer', [sys '/quant'], ...
        'QuantizationInterval', g(se.pos_res), ...
        'Position', [820 210 860 240]);
    add_block('simulink/Sources/Random Number', [sys '/pos_noise'], ...
        'Mean', '0', 'Variance', g(se.pos_noise^2), ...
        'Seed', num2str(seed), 'SampleTime', g(se.Ts), ...
        'Position', [820 280 860 310]);
    add_block('simulink/Math Operations/Sum', [sys '/sum_pos_noise'], ...
        'Inputs', '++', 'IconShape', 'rectangular', ...
        'Position', [880 210 910 250]);
    add_block('simulink/Discontinuities/Saturation', [sys '/sat_pos'], ...
        'UpperLimit', g(se.pos_max), 'LowerLimit', g(se.pos_min), ...
        'Position', [940 210 980 240]);

    % Sensor de corriente
    add_block('simulink/Continuous/Transfer Fcn', [sys '/cur_filter'], ...
        'Numerator', '[1]', ...
        'Denominator', ['[' g(1/(2*pi*se.cur_bw)) ' 1]'], ...
        'Position', [760 340 820 380]);
    add_block('simulink/Math Operations/Bias', [sys '/cur_bias'], ...
        'Bias', g(se.cur_offset), 'Position', [850 340 890 380]);
    add_block('simulink/Sources/Random Number', [sys '/cur_noise'], ...
        'Mean', '0', 'Variance', g(se.cur_noise^2), ...
        'Seed', num2str(seed+1), 'SampleTime', g(se.Ts), ...
        'Position', [850 410 890 440]);
    add_block('simulink/Math Operations/Sum', [sys '/sum_cur_noise'], ...
        'Inputs', '++', 'IconShape', 'rectangular', ...
        'Position', [920 340 950 380]);
    add_block('simulink/Discontinuities/Saturation', [sys '/sat_cur'], ...
        'UpperLimit', g(se.cur_max), 'LowerLimit', g(se.cur_min), ...
        'Position', [980 340 1020 380]);

    % Velocidad: derivada filtrada de la posicion medida
    add_block('simulink/Continuous/Transfer Fcn', [sys '/vel_filter'], ...
        'Numerator', '[1 0]', ...
        'Denominator', ['[' g(1/(2*pi*se.vel_fc)) ' 1]'], ...
        'Position', [900 500 960 540]);

    % Conexiones de la planta
    add_line(sys, 'V/1',        'sum_bemf/1', 'autorouting', 'on');
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

    % Sensor de posicion
    add_line(sys, 'int_th/1',      'pos_bias/1',      'autorouting', 'on');
    add_line(sys, 'pos_bias/1',    'quant/1',         'autorouting', 'on');
    add_line(sys, 'quant/1',       'sum_pos_noise/1', 'autorouting', 'on');
    add_line(sys, 'pos_noise/1',   'sum_pos_noise/2', 'autorouting', 'on');
    add_line(sys, 'sum_pos_noise/1', 'sat_pos/1',     'autorouting', 'on');
    add_line(sys, 'sat_pos/1',     'theta_meas/1',    'autorouting', 'on');

    % Sensor de corriente
    add_line(sys, 'invR/1',        'cur_filter/1',    'autorouting', 'on');
    add_line(sys, 'cur_filter/1',  'cur_bias/1',       'autorouting', 'on');
    add_line(sys, 'cur_bias/1',    'sum_cur_noise/1', 'autorouting', 'on');
    add_line(sys, 'cur_noise/1',   'sum_cur_noise/2', 'autorouting', 'on');
    add_line(sys, 'sum_cur_noise/1', 'sat_cur/1',      'autorouting', 'on');
    add_line(sys, 'sat_cur/1',     'i_meas/1',         'autorouting', 'on');

    % Velocidad derivada de la posicion medida
    add_line(sys, 'sat_pos/1',     'vel_filter/1',     'autorouting', 'on');
    add_line(sys, 'vel_filter/1',  'omega_meas/1',     'autorouting', 'on');
end
