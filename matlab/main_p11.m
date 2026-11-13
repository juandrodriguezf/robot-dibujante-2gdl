% main_p11.m
% Ejecucion automatica de P11: control de bajo nivel.
%
%   Uso:  >> main_p11

aqui = fileparts(mfilename('fullpath'));
addpath(genpath(aqui));

build_control_model;
test_control_bajo_nivel;
