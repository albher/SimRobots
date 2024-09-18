classdef Cin < handle
    % Autor: Alberto Herreros albher@uva.es
    % Versión: 17-7-2024
    % Nueva versión del Kin
    % Usa solo funciones de Robotic Toolbox, no de la librería de Corke
    % Se ha incorporado la IK, GIK y AIK de Robotic Toolbox, y JacIK
    % Puede soportar simulaciones de varios robots uno trás otro y
    % simultáneos a partir de los puntos de cada uno
    
    
    properties
       robot % Objeto Rigid Body asociado
       q % Posición q actual del robot
       tool % Herramienta (eje) sobre el que se trabaja en movimiento
       ejes % Elementos asociados a la simulacion (matlab RigidBody)
       psim % Elementos asociados a la simulación (simulink MultiBody)
       save % Elementos asociados a salvar las posiciones

       % Propiedades de los algoritmos IK, GIK y AIK
       w % Pesos de la cinemática inversa IK
       aik % nombre robot donde se tiene la función AIK
       jacik % Variable lógica para usar IK con Jacobiano
       gikSolver % Solver definido para GIK
       gikObj % Objetivos definidos para GIK
       env % Para el manipulador rrt 
    end %properties
    
    methods
        function this= Cin(robot, mdl, tool, q, in)
        % this= Cin(robot, mdl, tool, q)
        %   robot: objeto Rigid Body asociado al robot y herramienta
        %   mdl: fichero simulink de la estación robótica
        %   tool: objeto Rigid Body asociado a la herramienta
        %   q: posición inicial de los ejes del robot
        %   in: Primera posición del la q dentro de la q total de la
        %   estación. Puede haber varios objetos Cin y que cada uno apunte
        %   a un robot.
        % Variables internas de paso al fichero simulink
        %   q0_: Posición inicial de los ejes de los robots
        %   pose_: Posición inicial del TCP para el rastreador
        %   sol_: Devuelve la solución de la simulación

            if nargin<1
                error('this= Cin(robot, mdl, tool, )');
            elseif nargin==1
                mdl= []; tool=[]; q=[]; in=[];
            elseif nargin==2
                tool= []; q= []; in=[];
            elseif nargin==3
                q= []; in=[];
            elseif nargin==4
                in=[];
            end
            if isempty(tool)
                this.tool= robot.BodyNames{end};
            else
                this.tool= tool;
            end
            if isempty(q)
                this.q= [0,0,0,0,0,0];
            else
                this.q= q;
            end     
            if isempty(in)
               this.psim.in= 1;
            else
               this.psim.in= in;
            end
            robot.DataFormat= 'row';
            this.robot= robot;
            % Parámetros por defecto
            fprintf('Herramienta: %s\n', this.tool);
            if isempty(mdl)
                % Simulación de robot de palo
                this.psim.mdl= [];
                this.ejes.axis= [-700,700,-700,700,-100,1000]*1e-3;
                this.ejes.view= [120,30];
                this.ejes.size= 70*1e-3;
            else
               % Simulación con Multibody
               this.psim.mdl= mdl; 
               this.psim.Ts= 0.1; 
               this.psim.nPts= 10;
               fprintf('Simulación. Ts= %.3f m/s\n', this.psim.Ts);
            %  Es útil si los datos han sido obtenidos de *.mat
               if isempty(in)
                  % Solo hay un robot en la estación
                  assignin('base', 'q0_', this.q);
               end
               assignin('base', 'pose_', rand(3,3)*1e-3);
            end
            % Guarda posiciones del robot
            this.save.q= []; % Salva joint
            this.save.t= []; % Salva tiempo
            this.save.pose= []; % Salva pose
            this.save.value= 0; % 0 no graba, 1 graba
            % Algoritmos IK (propiedades)
            this.w= ones(1,6); % W IK (pesos)
            this.aik= ''; % AIK (analítico)
            this.gikSolver= []; % GIK (generalizado)
            this.gikObj= [];
            this.jacik= -1; % JacIK (incremental)
            this.env= {}; % manipulador rrp

            this.Show();
        end

        function disp(this)
           if ~isempty(this.psim.mdl)
              fprintf('Herramienta: %s\n', this.tool);
              fprintf('Modelo: %s\n', this.psim.mdl);
              fprintf('Ts= %.3f m/s\n', this.psim.Ts);
              fprintf('Entradas= %d-%d\n', this.psim.in, this.psim.in+length(this.q)-1)
              fprintf('IK alg= ');
              if this.jacik >=0
                 fprintf('JacIK (incremental)\n');
              elseif ~isempty(this.aik)
                 fprintf('AIK (analítico)\n');
              elseif ~isempty(this.gikSolver)
                 fprintf('GIK (generalizado)\n');
              else
                 fprintf('W IK (pesos)\n');
              end
           end
        end
                        
        function robot= Robot(this)
        % robot= Robot(this)
        % Devuelve el objeto RigidBody asociado al objeto Kin
        % Muestra la estructura del RigidBody asociado
            robot= this.robot;
            robot.showdetails;
        end

        function robot= Hbody(this, Obj, Nbody)
        % Modifica la matriz H (Joint) del body Nbody
        % Por defecto la base
        % Se debe modificar a la vez la variable de Simulink
        % para que MultiBody sea igual a RigidBody
        % Nbody puede ser el índice del Body o su nombre

           if nargin==1
               Obj= []; Nbody=1;
           elseif nargin==2
               Nbody=1;
           end

           if ischar(Nbody)
              for i= 1:this.robot.NumBodies
                 if strcmp(this.robot.Bodies{i}.Name,Nbody)
                    Nbody= i;
                    break;
                 end
              end
           end

           if Nbody<= this.robot.NumBodies
              setFixedTransform(this.robot.Bodies{Nbody}.Joint, Prod(Obj)); 
           else
              error('Arg Obj. Pieza y Nbody< maxNbody');
           end
           this.Show;
           robot= this.robot;
        end

        function DelBody(this, nameBody, q0)
        % Elimina un body del objeto RigidBody
           if nargin==2
              q0= this.q;
           end
           removeBody(this.robot, nameBody)
           this.robot.showdetails;
           % Depende si el eje es fijo (misma q) o articulado
           this.q= q0;
        end

        function AddRob(this, nameBody, Rob, q0)
        % Añade un RigidBody a partir del body NameBody
           if nargin<4
              error('AddRob(this, nameBody, Rob, q0)')
           end
           addSubtree(this.robot, nameBody, Rob.robot);
           this.robot.showdetails;
           % Nueva q0 incrementada en ejes de Rob
           this.q= q0;
        end

        function Q= JacIK(this, pose) 
        % Algoritmo incremental usando el jacobiano
        % Para medir incrementos usar la formula Euler-Rodrigues
        % Se evitan los saltos entre [-180,180]
        % Se define una norma para limitar el incremento en w
            if nargin==1
               this.jacik= 0.3;
               fprintf('JacIK activado');
               return
            elseif length(pose)==1
               this.jacik= pose;
               if pose<0
                   fprintf('JacIK desactivado');
               else
                   fprintf('JacIK activado');
               end
               return
            end
            %disp('JacIK')
            % Ángulos entre [-pi, pi]
            nPts= size(pose,1);
            Q= zeros(nPts,length(this.q));
            Q(1,:)= this.q;
            pose(:,4:6)= wrapToPi(pose(:,4:6));
            poseR= pose(1,:);          

            for i= 2:nPts
                % Hallar el incremento entre poseR y pose(i,:)
                % Usar la formula de Euler-Rodrigues
                % Evita los saltos entre [-180, 180]
                dpose= this.tf2dpose(Prod(poseR), Prod(pose(i,:)));

                if norm(dpose(4:6))> this.jacik
                % Si el incremento w es muy grande se elimina
                % Si this.jacik es 0 solo se modifica la posición
                   fprintf('max iter %d  ', i);
                   dpose(4:6)= 0*dpose(4:6);
                end

                % Hallar el incremento de dq a partir del dpose con ganancia
                dq= this.Jac(Q(i-1,:))\dpose';
                % Incrementar q, problemas con los ángulos [-pi,pi]
                Q(i,:)= Q(i-1,:)+ dq';
                % Conseguir que los ángulos estén entre [-pi,pi]
                Q(i,:)= wrapToPi(Q(i,:));
                % Hallar el nuevo punto a partir de la posición articular
                poseR= this.Pose(Q(i,:));
            end
        end

        function q= IK(this, pose)
            if nargin==1
               error('IK: Pose entrada')
            end
           ik = robotics.InverseKinematics('RigidBodyTree', this.robot);
           npose= size(pose, 1);
           q= zeros(npose, length(this.q));
           q0= this.q;
           for i= 1:npose
              H= Prod(pose(i,:));
              q0= ik(this.tool, H, this.w, q0);
              q(i,:)= q0;
           end
        end

        function GIKGen(this, varargin)
        % Por defecto hay un objetivo 1 pose
        % Se pueden introducir otros objetivos
        % Ejemplo:
        %    cartesian = constraintCartesianBounds(r1.TcpCarga);
        %    cartesian.Bounds = [-inf, inf ; ...
        %               -inf, inf; ...
        %               0*1e-3, inf];
        %    GIKGen('cartesian', cartesian)
            obj{1}= 'pose';
            this.gikObj= [];
            this.gikObj{1}= constraintPoseTarget(this.tool);
            %if nargin==2
            j= 1;
            for i= 1:2:nargin-1
                j= j+1;
                obj{j}= varargin{i};
                this.gikObj{j}= varargin{i+1};
            end
            this.gikSolver = generalizedInverseKinematics('RigidBodyTree', ...
                             this.robot, 'ConstraintInputs', obj);
        end

        function Q= GIK(this, vPose)
        % Obtiene la cinemática inversa del problema generalizado
        % El primer objetivo es el pose al que ir
        % El resto son los que el usuario quiera.
           fun= '[qend, infor]= this.gikSolver(this.q';
           for i=1:length(this.gikObj)
              fun= [fun,', ', sprintf('this.gikObj{%d}',i)];
           end
           fun= [fun,');'];

           n= size(vPose,1);
           dim= length(this.q);
           Q= zeros(n, dim);
           this.gikObj{1}.EndEffector= this.tool;
           for i= 1:n
              this.gikObj{1}.TargetTransform= Prod(vPose(i,:));
              eval(fun);
              Q(i,:)= qend;
           end
        end

        function AIKGen(this, nombre, tool)
        % Genera ficheros AIK_nombre_tool 
        % por cada componente de tool (celda)
        % nombre es cadena con el nombre robot 'irb120'
        % tool es una celda con los bodies terminales 
        % {'Body09', 'Body10'}
           if nargin== 1
               this.aik='';
           elseif nargin==2
               this.aik= nombre;
           else
               AIK = analyticalInverseKinematics(this.robot);
               if AIK.IsValidGroupForIK
                   for i= 1:length(tool)
                      AIK.KinematicGroup.EndEffectorBodyName= tool{i};
                      generateIKFunction(AIK, sprintf('AIK_%s_%s', nombre, tool{i}));
                   end
               else
                   disp(['No AIK válido. Robot= ', AIK.KinematicGroupType])
               end
           end
        end

        function [Q, qOpt]= AIK(this, vPose)
        % Cinemática inversa obtenida del algoritmo analítico
        % Se debe haber compilado la función que lo define
        % vPose es una matriz de pose, Q la matriz correspondiente IK
        % Si tiene varias soluciones se toma la primera
            if nargin==1
               error('AIK: Pose entrada')
            end
            [n,~]= size(vPose);
            Q= zeros(n, length(this.q));
            for i= 1:n
               qOpt= feval(sprintf('AIK_%s_%s', this.aik, this.tool), Prod(vPose(i,:)));
               [nq,~]= size(qOpt);
               if nq==0
                  disp('AIK (Error): Sin solución inversa')
                  Q(i:end,:)= [];
                  break
               else
                  Q(i,:)= qOpt(1,:);
               end
            end    
        end

        function tool= Tool(this, name)
        % Tool(this, name)
            if nargin==2
                this.tool= name;
            end
            tool= this.tool;
        end

        function w= W(this, w)
        % w es el peso dado a cada elemento pose para su obtención por
        % optimización. El peso es [x,y,z,Rz,Ry,Rx]. 
        % Se cambia el valor porque Robot Toolbox usa [Rz,Ry,Rx,x,y,z]  
        % Ejemplo [1,1,1,0,0,0] solo optimiza la posición
           if nargin==1
              w= ones(1,6);
           end
           this.w= w([4,5,6,1,2,3]);
           % Desactivamos los demás algoritmos
           this.aik= ''; % AIK (analítico)
           this.gikSolver= []; % GIK (generalizado)
           this.gikObj= [];
           this.jacik= -1; % JacIK (incremental)
           this.env= {}; % manipulador rrp
        end

        function q= Q(this, q)
           % Introduce el valor q en la propiedad 
           if nargin==1
              q= this.q;
           end
           this.q= q(:)';
        end
                
        function pose= Pose(this, q, Hwobj)
        % pose= Pose(this, q)
        % Cinemática directa
        % Devuelve la posición pose= [[x,y,z]mm,[Rz,Ry,Rx]rad] 
        % Si se añade el argumento Hwobj se convierte los pose a ese eje
        % correspondiente a q o por defecto la posición del robot actual
            if nargin==1
               q= []; Hwobj=[]; 
            elseif nargin<3
                Hwobj=[];
            end
            if isempty(q)
                q= this.q;
            end
            nq= size(q, 1);
            pose= zeros(nq,6);
            for i= 1:nq
               try
                  H= getTransform(this.robot, q(i,:), this.tool);
               catch
                  error('Error Cin Inv: Mal Dim. q0, Mal nombre TCP body, ...')
               end
               [~, pose(i,:)]= Prod(H);
            end
            % Posición respecto de un eje dado
            if ~isempty(Hwobj)
               Hwobj= inv(Prod(Hwobj));
               pose= ProdV(Hwobj, pose);
               % for i= 1:nq
               %    [~, pose(i,:)]= Prod(Hwobj, pose(i,:));
               % end
            end
        end
        
        function q= Joint(this, pose)
        % q= Joint(this, pose)
        % Cinemática inversa
        % Devuelve la posición q correspondiente a pose= [[x,y,z]mm,[Rz,Ry,Rx]rad]
        % w es el peso dado a cada elemento pose para su obtención por
        % optimización. El peso es [Rz,Ry,Rx,x,y,z] (de 0-1). 
        % Ejemplo [0,0,0,1,1,1] solo optimiza la posición
           if nargin==1
              q= this.q;
              return
           end
           % Uso uno de los algoritmos propuestos
           if this.jacik >=0
               q= this.JacIK(pose);
           elseif ~isempty(this.aik)
              q= this.AIK(pose);
           elseif ~isempty(this.gikSolver)
              q= this.GIK(pose);
           else
              q= this.IK(pose);
           end
        end
        
        function [q, poseN]= MoveAbsJ(this, qN, nPts)
        % q= MoveAbsJ(this, qN, nPts)
        % Posiciones q para mover la tool eleguida hasta la posición 
        % qN en nPts pasos
        % Si no hay argumentos de salida, se simula el movimiento
            if nargin<2
                error('q= MoveAbsJ(this, q1, [nPts])')
            elseif nargin==2
                nPts= this.psim.nPts;
            end
            [fil, col]= size(qN);
            if length(this.q) ~= col
               error('Dimensión q incorrecta')
            end

            if fil==1
                % Interpolación de q
                q0= this.q;
                % Interpolación rrt
                q= this.interpQ(q0, qN, nPts);
            else
                q= [this.q; qN];
            end
             poseN= this.Pose(q);
%            this.q= q(end,:);
            if nargout==0
               this.Show(q, poseN); 
            end
        end
        
        function [q, poseN]= MoveJ(this, poseN, Hwobj, nPts)
        % [q, poseN]= MoveJ(this, poseN, [Hwobj], [nPts], [w])
        % Posiciones en q y pose= [[x,y,z]mm,[Rz,Ry,Rx]rad]
        % desde la posición actual a poseN. 
        % Se mueven todos los ejes de forma simultánea
        % Se toma por referencia Hwobj (Matriz homogénea) defecto base
        % Si no hay argumentos de salida, se simula el movimiento
            if nargin<2
                error('q= MoveJ(this, poseN, [Hwobj], [nPts])')
            elseif nargin==2
                Hwobj=[]; nPts= [];
            elseif nargin==3
                nPts= [];
            end

            if length(poseN(:))==16
            % Convierte H en Pose
               [~, poseN]= Prod(poseN);
            end
            if isempty(nPts)
               nPts= this.psim.nPts;
            end

            % Se desea no cambiar el giro actual
            [fil, col]= size(poseN);
            if col==3
               pose0= this.Pose(this.q, Hwobj);
               poseN(:, 4:6)= repmat(pose0(1, 4:6), [fil,1]);
            end
            
            % El objeto puede se Piezas, Cin, Pose o H                       
            if ~isempty(Hwobj)
               for i= 1:fil
                  [~, poseN(i,:)]= Prod(Hwobj, poseN(i,:)); 
               end
            end
               
            if fil==1 % poseN solo es un punto y se desea interpolar 
                % Va de usando joint
                % q0= this.q;
                % qN= this.Joint(poseN(1,:));
                % q= this.interpL(q0, qN, nPts);
                % poseN= this.Pose(q);
                % Va por interpolación de H
                pose0= this.Pose(this.q);
                poseN= this.interpH(pose0, poseN, nPts);
                q= this.Joint(poseN);
            else % poseN son varios puntos y se desea ir por ellos
                q= [this.q; this.Joint(poseN)];
            end
%            this.q= q(end,:);
            if nargout==0
               this.Show(q, poseN);
            end
        end
        
        function [q, poseN]= MoveL(this, pose1, Hwobj, nPts)
        % [q, poseN]= MoveL(this, pose1, Hwobj, nPts)
        % Posiciones en q y pose= [[x,y,z]mm,[Rz,Ry,Rx]rad]
        % del movimiento lineal entre la posición actual del tool
        % y la posición final pose1
        % Se toma por referencia Hwobj (Matriz homogénea) defecto base
        % Si no hay argumentos de salida, se simula el movimiento
            if nargin<2
                error('q= MoveL(this, pose1, [Hwobj], [nPts])');
            elseif nargin==2
                Hwobj= []; nPts= [];
            elseif nargin==3
                nPts= [];
            end

            if length(pose1(:))==16
            % Convierte H en Pose
               [~, pose1]= Prod(pose1);
            end
            
            if isempty(nPts)
               nPts= this.psim.nPts;
            end

            pose0= this.Pose(this.q);
            if length(pose1)==3
                pose1(4:6)= pose0(4:6);
            end
            % El objeto puede se Piezas, Cin, pose o H
            if ~isempty(Hwobj)
               [~, pose1]= Prod(Hwobj, pose1); 
            end
                        
            % Interpolación en todos los elementos
            poseN= [this.interpL(pose0(1:3), pose1(1:3), nPts), ...
                    this.interpR(pose0(4:6), pose1(4:6), nPts)];
            q= this.Joint(poseN);
%            this.q= q(end, :);
            if nargout==0
               this.Show(q, poseN);
            end
        end
        
        function [q,poseN]= MoveC(this, y1, x1, Hwobj, nPts)
        % [q,poseN]= MoveC(this, y1, x1, Hwobj, nPts)
        % Solo admite puntos y1 x1, con coordenadas [x,y,x], sin ángulos  
        % El ángulo es el de la posición que se parte
        % Posiciones q y pose [[x,y,z]mm,[Rz,Ry,Rx]rad]
        % del punto actual a x1 pasando por y1 en arco de circunferencia
        % Si no hay argumentos de salida, se simula el movimiento
            if nargin<3 
                error('q= MoveC(this, y1, x1, [Hwobj], [nPts])');
            elseif nargin==3
                Hwobj= []; nPts= 20;
            elseif nargin==4
                nPts= this.psim.nPts;
            end

            % El objeto puede se Piezas, Cin, Pose o H
            if ~isempty(Hwobj)                
                Hwobj= Prod(Hwobj);
                [~, x1]= Prod(Hwobj, x1);
                x1= x1(1:3);
                [~, y1]= Prod(Hwobj, y1);
                y1= y1(1:3);
            end
            
            pose1= this.Pose;
            x2= pose1(1:3);
            % Tres puntos
            pts= [x1; x2; y1];
            H1= Prod(x1,x2,y1);
            % No se modifican los ángulos de los puntos
            pts= [pts, repmat(pose1(:,4:6), [3,1])];
            % Mover los puntos al nuevo eje 
            % x1, x2 solo tienen componente x
            % y1 solo tiene componente y
            p3pts= ProdV(inv(H1),pts);
            % Se mueve este eje de forma que x1 y x2 sean simétricos y la distancia de
            % los tres puntos al origen sea la misma
            x1= p3pts(1,1); x2= p3pts(2,1); y1= p3pts(3,2);
            xc= -(x1+x2)/2;
            yc= (x1^2- y1^2+ 2*x1*xc)/2/y1;
            H2= Prod([-xc, -yc, 0]);
            % Puntos iniciales en el nuevo eje x1 y x2 simétricos y todos a la misma
            % distancia del origen
            Tpts= ProdV(inv(H1*H2),pts);
            % Radio del arco es la distancia a los puntos 
            % la misma distancia a todos los tres puntos
            radio= norm(Tpts(1,1:3));
            % Ángulo para trazar el arco desde el x1 al x2 pasando por y1
            theta0= asin(yc/radio);
            % Puntos del arco en el nuevo eje y en el eje origen
            pose= [zeros(nPts,3), repmat(Tpts(1,4:6),[nPts,1])];
            theta= linspace(theta0, pi-theta0, nPts);
            for i= 1:nPts
               pose(i,1)= radio*cos(theta(i));
               pose(i,2)= radio*sin(theta(i));
            end
            % Mover por Joint de los puntos generados
            if nargout==0
                this.MoveJ(pose(:,1:6), H1*H2);
            else
               [q,poseN]= this.MoveJ(pose(:,1:6), H1*H2);
            end
        end
        
        function Show(this, q, pose, t)
        % Show(this, q, pose, t)
        % Simula los cambios q en el sistema RigidBody (matlab) 
        % o MultiBody (Simulink)
        % Si q=[] solo rastrea las posiciones pose en Simulink 
            if nargin==1
                q= []; pose= [];
            elseif nargin==2
                pose=[];
            end
            if isempty(q)
                q= this.q;
            end
            if isempty(this.psim.mdl) 
               % Gráfica matlab
               for i= 1:size(q,1)
                  hold off
                  show(this.robot, q(i,:));
                  axis(this.ejes.axis);
                  if ~isempty(pose)
                     hold on
                     for j= 1:size(pose,1)
                        H= se3(pose(j, 4:6), 'eul', pose(j, 1:3));
                        plotTransforms(H, 'FrameSize', this.ejes.size);
                     end
                  end
                  view(this.ejes.view(1),this.ejes.view(2));
                  drawnow
               end
               if this.save.value==1
                  this.save.q= [this.save.q; q];
                  this.save.pose= [this.save.pose; pose(:,1:3)];
               end
            else 
               % Simulación Simulink multi-body
               if nargin<4 % No hay tiempo de entrada
                  t= (0:this.psim.Ts:this.psim.Ts*(size(q,1)-1))';
               end
               % Hay que trabajar en el Workspace porque todas las
               % variables están en ese entorno. Cuidado con no usar las
               % variables pose_ y q0_
               try 
                   q0_= evalin('base','q0_');
               catch
                   error('Indefinida variable q0_')
               end
               
               [nq, mq]= size(q);
               in= this.psim.in;
               q0_(in:in+mq-1)= q(1,:);
               qf= repmat(q0_, [nq,1]);
               qf(:, in:in+mq-1)= q;
               this.Sim(qf, pose, t);
           end
           % modifica el q del objeto a la nueva posición  
           this.q= q(end,:);
        end
      
           
        function Sim(this, q, pose, t)
        % Simula el sistema con unas entradas dadas
           if isempty(this.psim.mdl)
               error('Valido para simscape');
           end
           if nargin<2
               error('Sim(this, q, [pose], [t])')
           elseif nargin<3
               t= [];
               pose= [];
           elseif nargin==3
               t= [];
           end
           if isempty(t)
               t= (0:this.psim.Ts:this.psim.Ts*(size(q,1)-1))';
           end
           if size(pose,1)<3 % Para dibujar trayectoria en pantalla
               pose= this.Pose();
               pose= pose(1:3);
               poseT= repmat(pose, [3,1])+ randn(3,3)*1e-4;
           else
               % El icono splice (simulink) interpola y no admite 
               % dos puntos iguales
               np= size(pose,1); 
               pose= pose(:, 1:3)+ randn(np, 3)*1e-5; 
               poseT= [this.save.pose; pose];
           end
           try
               assignin('base', 'q0_', q(1,:));
               assignin('base', 'pose_', poseT);
               sol_= sim(this.psim.mdl, t, [], [t,q]);
               assignin('base', 'q0_', q(end,:));
               assignin('base', 'sol_', sol_);
           catch
               fprintf('Error en simulación: \n        No definidas q0_, pose_, Falta *.stl, Mal dimensión q0 \n');
               fprintf('Tam q robot= %d\n', length(this.q));
               fprintf('Tam q0_ global= %d\n', size(q,2));
               fprintf('mdl= %s\n', this.psim.mdl);
               eval(['open_system(', this.psim.mdl, ');']);
           end

           % Guarda valores para reproducción final
           if this.save.value==1
              this.save.q= [this.save.q; q];
              if isempty(this.save.t)
                 this.save.t= [this.save.t; t];
              else
                 this.save.t= [this.save.t; t+ this.save.t(end)+ this.psim.Ts];
              end
              this.save.pose= [this.save.pose; pose];
           end          
        end

        function Ts(this, T)
        % Ts(this, T)
        % Tiempo de simulación s entre puntos
           if nargin==1
               T= 0.1;
           end
           this.psim.Ts= T;
        end

        function Npts(this, nPts) 
           if nargin==1
              this.psim.nPts= 10;
           else
              this.psim.nPts= nPts;
           end
        end

        function Rec(this, value) 
        % Control de grabación de simulación
            if nargin==1 % Resetea e inicia la grabación
                this.save.q= [];
                this.save.t= [];
                this.save.pose= [];
                this.save.value= 1;                
            elseif value==0 % Para la grabación
                this.save.value= 0;
            elseif value==1 % Inicia la grabación sin resetear
                this.save.value= 1;
            else
                error('Valor incorrecto');
            end
        end
        
        function sol= Rep(this)
        % Simula las posiciones grabadas
           
           if size(this.save.q,1)>3
               opt= [];
               assignin('base', 'q0_', this.save.q(1,:));
               assignin('base', 'pose_', this.save.pose);
               sol= sim(this.psim.mdl, this.save.t, opt, [this.save.t, this.save.q]);
               assignin('base', 'q0_', this.save.q(end,:));
               assignin('base', 'sol_', sol);
           else
               disp('No hay record')
           end
        end

        function jac= Jac(this, q)
           % Devuelve el jacobiano de un robot en un joint y con la
           % herramienta que tenga
           % 6xN matriz. Cad
           % [wx,wy,wz,vx,vy,vz]'= Jac*[dq1,dq2, ...,dqN]'
           if nargin== 1
              q= this.Joint;
           end
           jac= geometricJacobian(this.robot, q(:)', this.tool);
           % El jacobiano [dRx,dRy,dRz, dx, dy, dz]'= Jac*q
           % Jac (6xN) siendo N el número de articulaciones
           % lo paso a la forma
           % [dx, dy, dz, dRz, dRy, dRx]'= Jac*q
           jac= jac([4,5,6,3,2,1],:);
        end

        function Torque= IDin(this, q, dq)
           % Calcula el torque de cada articulación a partir de su posición
           % Puede ser interesante para robot colabotativos
           % Relacionado: 
           %    externalForce, definir fuerzas externas
           %    forwardDynamics: Calcula la aceleración a partir de la
           %    velocidad y el par.
           if nargin==1
               q= this.q;
           elseif nargin==2
               dq= [];
           end
           [nf, nc]= size(q);
           if isempty(dq)
              dq= zeros(nf,nc);
           end
           Torque= zeros(nf,nc);
           for i= 1:nf
              Torque(i,:)= inverseDynamics(this.robot, q(i,:), dq(i,:))';
           end
        end

        function Env(this, env)
            % Se definen los restricciones del manipulador
            % Ejemplos
            % env = {collisionBox(0.5, 0.5, 0.05) collisionSphere(0.1)};
            % Pose es una matriz homogenea
            % env{1}.Pose(3, end) = -0.05;
            % env{2}.Pose(1:3, end) = [3 0.2 0.4];
            if nargin==1
                this.env={};
            else
                this.env= env;
            end
        end

        function Q= interpQ(this, q1, q2, nPts)
           rrt = manipulatorRRT(this.robot, this.env);
           rrt.SkippedSelfCollisions = "parent";
           %path = plan(rrt, q1, q2);
           path= [q1; q2];
           Q = interpolate(rrt, path, nPts); 
        end

    end % methods

    methods (Static)
        function Q= interpL(q1,q2,nPts)

           dim= length(q1);
           q1= wrapToPi(q1);
           q2= wrapToPi(q2);
           Q= zeros(nPts,dim);
           for i= 1:dim
              Q(:,i)= linspace(q1(i),q2(i),nPts);
           end
        end


        function Reul= interpR(rot1, rot2, nPts)
           Reul= zeros(nPts, 3);
           % Objetos de Robotic Toolbox
           Rot= interp(so3(rot1,"eul"), so3(rot2,"eul"), nPts);
           for i= 1:nPts
              Reul(i,:)= eul(Rot(i));
           end
        end

        function Pose= interpH(pose1, pose2, nPts)
           Pose= zeros(nPts, 6);
           % Objetos de Robotic Toolbox
           Ht= interp(se3(Prod(pose1)), se3(Prod(pose2)), nPts);
           for i= 1:nPts
              [~, Pose(i,:)]= Prod(tform(Ht(i)));
           end
        end

        function d= tf2dpose(t1, t2)
        % Obtiene la diferencia entre t2-t1
        % t1 y t2 están en formato tf
        % en dpose [dx, dy, dz, dRz, dRy, dRx]
        % Basado en la formula de Rodrigues

              if nargin==1
                 d= [t1(1:3,4);
                     0.5*[t(3,2)- t1(2,3); ...
                     t1(1,3)- t(3,1); ...
                     t1(2,1)- t1(1,2)]];
              else
                  d= [t2(1:3,4)- t1(1:3,4);
                      0.5*(cross(t1(1:3,1), t2(1:3,1))+ ...
                           cross(t1(1:3,2), t2(1:3,2))+ ...
                           cross(t1(1:3,3), t2(1:3,3)))];
              end
              d= d([1,2,3,6,5,4])';
        end
    end % methods static
    
end % class


