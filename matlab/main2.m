% main2.m
% Ejecucion automatica de la Parte 2 (Puntos 8-11) del proyecto.
%
%   Ejecuta:
%       Punto 7  trayectoria_test        (insumo: q, qd, qdd en el workspace)
%       Punto 8  dinamica_inversa        (RNE y resumen de torques)
%       Punto 10 test_motor_model        (motor DC + sensores y figuras)
%       Punto 11 build_control_model + test_control_bajo_nivel
%
%   Nota: trayectoria_test regenera las figuras del Punto 7 en
%   docs/informe-1/figuras/; test_motor_model y test_control_bajo_nivel
%   regeneran las de P10/P11 en docs/informe-2/figuras/.
%
%   Uso:  >> main2
%
%   Requiere MATLAB + Robotics Toolbox for MATLAB (Peter Corke). La ruta del
%   toolbox se detecta automaticamente con add_rvc_path().

clear;
clc;
close all;

% Anadir al path la carpeta del proyecto y todas sus subcarpetas
aqui = fileparts(mfilename('fullpath'));
addpath(genpath(aqui));

fprintf('#########################################################\n');
fprintf('#  EJECUCION AUTOMATICA - PARTE 2 (P8, P10 y P11)       #\n');
fprintf('#########################################################\n\n');

fprintf('\n========== PUNTO 7: TRAYECTORIAS (insumo) =========\n');
trayectoria_test;

fprintf('\n========== PUNTO 8: DINAMICA INVERSA ==============\n');
dinamica_inversa;

fprintf('\n========== PUNTO 10: MODELO DE MOTOR Y SENSORES =========\n');
test_motor_model;

fprintf('\n========== PUNTO 11: CONTROL DE BAJO NIVEL =============\n');
build_control_model;
test_control_bajo_nivel;

fprintf('\n#########################################################\n');
fprintf('#  EJECUCION COMPLETADA\n');
fprintf('#########################################################\n');
