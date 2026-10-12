function tf = add_rvc_path()
%add_rvc_path  Agrega el Robotics Toolbox de Peter Corke al path de MATLAB.
%
%   Salida:
%       tf: true si el toolbox quedo disponible, false en caso contrario.
%
%   El Robotics Toolbox NO es necesario para la cinematica inversa: las
%   funciones Jacobian() e ikine() de este taller solo dependen de la
%   cinematica directa propia. El toolbox se usa unicamente para contrastar
%   los resultados contra las funciones de referencia SerialLink.fkine(),
%   SerialLink.jacob0() y SerialLink.ikine().
%
%   Busqueda, en orden:
%       1. Si SerialLink ya esta en el path, no se hace nada.
%       2. Variable de entorno RVC_ROOT.
%       3. Rutas relativas a esta carpeta (rvctools, toolbox, ../rvctools).
%       4. Carpetas rvctools* dentro de Descargas (con un nivel de anidamiento).
%
%   Puede sobreescribirse la ubicacion con la variable de entorno RVC_ROOT.

    tf = false;

    if exist('SerialLink', 'class') == 8
        tf = true;
        return
    end

    candidates = local_candidate_roots();

    for i = 1:numel(candidates)
        root = candidates{i};
        if ~exist(root, 'dir')
            continue
        end
        % La raiz valida es la que contiene robot\@SerialLink
        if ~exist(fullfile(root, 'robot', '@SerialLink'), 'dir')
            continue
        end

        addpath(fullfile(root, 'robot'));
        addpath(fullfile(root, 'common'));

        p = fullfile(root, 'spatial-math');
        if exist(p, 'dir')
            addpath(p);
        end
        p = fullfile(root, 'robot', 'models');
        if exist(p, 'dir')
            addpath(p);
        end
        p = fullfile(root, 'robot', 'data', 'meshes');
        if exist(p, 'dir')
            addpath(p);
        end

        if exist('SerialLink', 'class') == 8
            fprintf('Robotics Toolbox (release %s) en el path:\n  %s\n', ...
                local_release(fullfile(root, 'robot')), root);
            tf = true;
            return
        end
    end

    fprintf(['aviso: no se localizo el Robotics Toolbox de Peter Corke.\n' ...
             '       La cinematica inversa se ejecutara sin comparacion contra\n' ...
             '       las funciones de referencia. Configure la variable de\n' ...
             '       entorno RVC_ROOT con la ruta del toolbox para habilitarla.\n']);
end


function roots = local_candidate_roots()
    here = fileparts(mfilename('fullpath'));

    roots = {};

    env = getenv('RVC_ROOT');
    if ~isempty(env)
        roots{end+1} = env;
    end

    local_names = {'rvctools', 'toolbox', 'rvctools2024'};
    for i = 1:numel(local_names)
        roots{end+1} = fullfile(here, local_names{i});
    end
    roots{end+1} = fullfile(here, '..', 'rvctools');

    % Descargas: rvctools* y rvctools*/rvctools*
    dl = fullfile(getenv('USERPROFILE'), 'Downloads');
    if ~exist(dl, 'dir')
        dl = fullfile(getenv('USERPROFILE'), 'OneDrive', 'Downloads');
    end
    if exist(dl, 'dir')
        hits = dir(fullfile(dl, 'rvctools*'));
        for i = 1:numel(hits)
            if ~hits(i).isdir
                continue
            end
            top = fullfile(hits(i).folder, hits(i).name);
            roots{end+1} = top;

            sub = dir(fullfile(top, 'rvctools*'));
            for j = 1:numel(sub)
                if sub(j).isdir
                    roots{end+1} = fullfile(sub(j).folder, sub(j).name);
                end
            end
        end
    end
end

function rel = local_release(tbpath)
    rel = 'desconocida';
    try
        fid = fopen(fullfile(tbpath, 'RELEASE'), 'r');
        if fid > 0
            rel = strtrim(fgetl(fid));
            fclose(fid);
        end
    catch
    end
end