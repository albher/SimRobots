function [H, pose]= Prod(H1, H2, H3)
% [H,pose]= Prod(H1, H2, H3)
% Multiplica en forma de matriz homogenea
% el objeto/vector/matriz H1 * H2
% Hi puede ser: 
%   Matriz homogenea
%   Objeto Pieza H.Htcp
%   Objeto Cin H.Pose en forma H
%   Objeto se3 de Robotic Toolbox
%   Vector Pose [[x,y,z]m,[Rz,Ry,Rx]rad] transformado en H
%   Vector pos [x,y,z]m
% Si el número de argumentos es 3 devuelve
%    el eje generado por los 3 puntos.
% Devuelve: H matriz homogenea
%           pose [[x,y,z]m,[Rz,Ry,Rx]rad]
    if nargin==0
        H1= []; H2=[];
    elseif nargin==1
        H2=[];
    end
    if nargin==3
       H= T3pts([H1;H2;H3]);
    else
       H= Htrans(H1)*Htrans(H2);
    end
    if nargout==2
        pose= H2pose(H);
    end
end

function H= Htrans(H)
   if isempty(H)
        H= eye(4);
    elseif isa(H,'Pieza')
        H= H.Htcp;
    elseif isa(H,'Cin')
        H= pose2H(H.Pose);
    elseif isa(H,'se3')
        H= tform(H);
    elseif length(H)==6
        H= pose2H(H(:)');
    elseif length(H)==3
       H1= eye(4);
       H1(1:3,4)= H(:);
       H= H1;
       %H= pose2H([H(:)',0,0,0])
    elseif length(H(:))~= 16
        error('H no válida')
    end
end

function pose= H2pose(H)
   pose= H(1:3,4)';
   pose(4:6)= tform2eul(H);
end

function H= pose2H(pose)
   H= eul2tform(pose(4:6));
   H(1:3,4)= pose(1:3)';
end

function T= T3pts(pts)
    % Genera un eje a partir de tres puntos
    %    * Dos puntos en el eje x
    %    * Un punto en el eje y
                
    if nargin==0
       error('T= T3pts(pts)');
    end
                
    [n,m]= size(pts);
    if n~=3 || m~= 3
        error('pts 3x3');
    end
                
    % Distancia de la recta del eje x con el punto del eje y
    dist= @(nu) sum(((1-nu)*pts(1,:)+ nu*pts(2,:)- pts(3,:)).^2);
                
    % Parámetro que minimiza esa distancia
    nu0= fminsearch(dist,rand);
                
    % Punto origen del eje
    p0= (1-nu0)*pts(1,:)+ nu0*pts(2,:);
    % Hallar los ejes normal (x) orientación (y) aproximación (z)
    n= (pts(2,:)- p0)'; n= n/norm(n);
    o= (pts(3,:)- p0)'; o= o/norm(o);
    a= cross(n,o);
    T= eye(4);
    T(1:3,1:3)= [n,o,a];
    T(1:3,4)= p0';
end
