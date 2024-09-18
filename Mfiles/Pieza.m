classdef Pieza < handle
    % Autor: Alberto Herreros albher@uva.es
    % Versión: 15-2-2024
    % Define los ficheros gráficos y dinámica
    % de las cargas y objetos de trabajo
    % PegarEn pega un objeto en otro sin modificar
    % la posición absoluta del mismo.
    
    properties (SetAccess = private)
      nombre % Nombre de la variable en strig
      base % Base de la figura tRzyx
      tcp % Salida de la figura en tRzyx
      masa % Masa
      cdg % Centro de gravedad
      inercia % Momentos de inecia
      color % color y opacidad de la figura.
     % aux % para usar el Hmat
    end % properties
    
    
    methods
        function this= Pieza(nombre)
        %  Renera una pieza de nombre 'vNombre' a la que asocia un
        %  posible fichero Fstl del directorio Biblioteca
        %  Si la pieza es estática (no va a ser modificado el fichero stl)
        %  los ficheros stl están definidos en el icono.
        
            if nargin==0
               nombre= 'Null';
            end

            if ~ischar(nombre)
               error('Los argumentos deben ser string')
            end
            
            this.nombre= nombre;
            this.Inicio;
            
        end % nPieza
        
        function Inicio(this)
            this.base= zeros(1,6);
            this.tcp= zeros(1,6);
            this.masa= 0;
            this.cdg= [0,0,0];
            this.inercia= [1,1,1]*1e-3;
            this.color= [rand(1,3), 0.7];
           % this.aux= Hmat;
           % this.aux.Rad;
        end % Inicio
        

        function Base(this, punto, wobj)
        % Método para introducir la base del objeto
           if nargin==1
               punto= [];
               wobj= [];
           elseif nargin==2
               wobj= [];
           end
           punto= Prod(punto);
           wobj= Prod(wobj);
           [~, pose]= Prod(wobj*punto);

           %this.base= MovTo(pose);
           this.base= pose;
                      
        end %Mover
        
        function Tcp(this, punto)
        % Método para introducir el  TCP del objeto
            if nargin==1
                punto= [];
            end
            [~,this.tcp]= Prod(punto);
        end % Tcp
        
        function H= Htcp(this)
        % Método para obtener el tcp del objeto en matriz homogénea
            H= Prod(this.base, this.tcp);
           % H= this.aux.tRzyx(this.base)*this.aux.tRzyx(this.tcp);
        end % Htcp
        
        function H= Hbase(this)
        % Método para obtener la base del objeto en matriz homogénea
            H= Prod(this.base);
           %H= this.aux.tRzyx(this.base);
        end % Hbase
        
        function Color(this, color)
        % Método para introducir el color y opacidad del objeto.
            if nargin==1
               color= [rand(1,3), 0.7]; 
            end
            this.color=color;
        end % color
                
        function PegarEn(this1, this2, wobj1, wobj2) 
        % Pega todas la propiedades del objeto this2 en this1
        % wobj1 es la referencia [x,y,z,Rz,Ry,Rx] del objeto this1
        % wobj2 es la referencia [x,y,z,Rz,Ry,Rx] del objeto this2
        % La base del objeto this1 se modifica de forma que en valor
        % absoluto el objeto no se mueva. 

            if nargin<2
               error('PegarEn(this1, this2, [wobj1, wobj2])');
            elseif nargin<3
                wobj1= []; wobj2=[];
            elseif nargin<4
                wobj2=[];
            end

            % Filtros de objetos de entrada a Homogénea
            wobj1= Prod(wobj1);
            wobj2= Prod(wobj2);

            [~, this1.base]= Prod(wobj2,this2.base);
            %[~, this1.base]= Prod(inv(wobj1),this1.base);
            this1.tcp= this2.tcp;
            this1.masa= this2.masa;
            this1.cdg= this2.cdg;
            this1.inercia= this2.inercia;
            this1.color= this2.color;

            this1.nombre= this2.nombre;
            this2.nombre= 'Null';

            this2.Inicio;
            % Pone el objeto en la misma posición
            % absoluta que tenía
            [~,pose]= Prod(Prod(inv(wobj1), this1.base));
            this1.Base(pose);
         
        %     if 0
        %      [~,this1.base]= Prod(Prod(inv(wobj1), this1.base));
        %     else
        %      % poseR= [0,0,0,pose(4:6)];
        %      % [~,poseR]= Prod(inv(Prod(poseR)));
        %      % poseR(1:3)= pose(1:3);
        %      % poseR(4:6)= -poseR(4:6);
        %      % this1.base= poseR;
        %      end

        end % PegarEn 
        
        function Din(this, masa, cdg, inercia) 
        % Método para introducir los elementos dinámicos del sistema
            if nargin <2
                masa= []; cdg= []; inercia=[];
            elseif nargin<3
                cdg= []; inercia=[];
            elseif nargin<4
                inercia=[];
            end
            if isempty(masa)
                masa= 1;
            end
            if isempty(cdg)
               cdg= [0,0,0];
            end
            if isempty(inercia)
                inercia= [1,1,1]*1e-3;
            end
            this.masa= masa;
            this.cdg= cdg;
            this.inercia= inercia;
        end % Dinamica
        
        
    end % methods
    
    
end % classdef