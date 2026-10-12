% Esta función permite obtener las matrices de rotación y el vector de
% posición a partir de las matrices homogéneas que definen al robot 

function [r, p] = homogenous_matrix(DH,q,base)

n = size(DH, 1);
q = q(:)';

if length(q) ~= n
    error('q debe contener un valor por articulación.');
end

if nargin < 3 || isempty(base)
    base = eye(4);
end

r = zeros(3,3,n);
p = zeros(3,n);
T = base;

for i = 1:n
    theta_offset = DH(i,1);
    d_offset = DH(i,2);
    alpha = DH(i,4);
    a = DH(i,3);
    sigma = DH(i,5);

    if sigma == 0
        theta = q(i) + theta_offset;
        d = d_offset;
    else
        theta = theta_offset;
        d = q(i) + d_offset;
    end

    A = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];

    T = T * A;
    r(:,:,i) = T(1:3,1:3);
    p(:,i) = T(1:3,4);
end