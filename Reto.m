%% Mini-Reto: Modulo 3 Control Difuso
% Algoritmo Bug 2 con control difuso
% Equipo 3: 
% Noemi Carolina Guerra Montiel
% Mizael Beltran Romero
% Izac Salazar
% Maria Fernanda Hernandez
% Viernes 13 de mayo del 2022

close all;
clear;

%% Inicializacion de variables

% Cargar matriz de obstaculos
load Obstaculo1.mat
entorno=Obstaculo1;

% Definir mapa y figura 1
refMap = binaryOccupancyMap(entorno,1);
refFigure = figure('Name','Sim robot con obstaculos');
show(refMap);
% Definir mapa y figura 2
[mapdimx,mapdimy] = size(entorno);
map = binaryOccupancyMap(mapdimy,mapdimx,10);
mapFigure = figure('Name','Contornos');
show(map);

% Definir cinematica y controlador PurePursuit
diffDrive = differentialDriveKinematics("VehicleInputs","VehicleSpeedHeadingRate");
controller = controllerPurePursuit('DesiredLinearVelocity',2,'MaxAngularVelocity',3);

% Cargar documento de control difuso
control=readfis('controlReto.fis');

% Lidar
sensor = rangeSensor;
sensor.Range = [0,10];
sensor.HorizontalAngleResolution;

%% Coordenadas 

% Coordenadas de incio y meta
path = [4 6;  22 22];
% Definir x, y de P1 y P2
x1 = path(1,1);
x2 = path(end,1);
y1 = path(1,2);
y2 = path(end,2);
% Calculo de pendiente
m = (y2-y1)/(x2-x1);
% Constante para ecuacion de y
b = y1 - m*x1;
% Vector de puntos en x,y de la linea
xVector = linspace(path(1,1), path(2,1), 100);
yVector = linspace(path(1,2), path(2,2), 100);
pts = [xVector(:), yVector(:)];
x = pts(1);
% Calculo de angulo del segmento de linea
aRectaDeg = atand(m); % Angulo en grados
aRecta = deg2rad(aRectaDeg); % Angulo en radianes

% Llamar a la figura de referencia
figure(refFigure);
hold on
% Plot del segmento de linea del inicio a la meta
plot(path(:,1),path(:,2), 'o-');
hold off

% Definir características del control NO difuso
controller.Waypoints = path;
LookaheadDistance=2;
controller.LookaheadDistance=LookaheadDistance;

% Pose inicial con el robot viendo hacia denlante
initPose = [path(1,1) path(1,2), aRecta];
% Pose de la meta
goal = [path(end,1) path(end,2)]';
% Coordenada x inicial y final
poses(:,1) = initPose';

% Iniciacion del indice y posicion
currX3 = 0;
currY3 = 0;
ind=0;

%% Llamado de funciones
% Mientras el robot no llegue a la meta
while ((x2 - currX3) > 0.3) && ((y2 - currY3) > 0.3)
% Si no es el inicio, inicia la posicion en donde se quedo
if ind > 0
   initPose = [currX3 currY3, aRecta];
end    
% Llama la funcion principal con la posicion inicial 
% Le pasa el control, x, m, b y el ind como parametros adicionales
[currX2, currY2, ang2] = exampleHelperDiffDriveCtrl(diffDrive,controller,initPose,goal,refMap,map,refFigure,mapFigure,sensor, control, x, m, b, ind, aRecta);
% Posicion actualizada y angulo de la recta
initPose2 = [currX2  currY2, aRecta];
ind=ind+1;
% Ya que circumnavego un obstaculo y encontro la linea, vuelve a llamar a
% la funcion con la posicion y rotacion actualizada.
[currX3, currY3, ang3] = exampleHelperDiffDriveCtrl(diffDrive,controller,initPose2,goal,refMap,map,refFigure,mapFigure,sensor, control, x, m, b, ind, aRecta);
end

%% Funcion para la navegacion del robot
function [currX, currY, ang] = exampleHelperDiffDriveCtrl(diffDrive,ppControl,initPose,goal,map1,map2,fig1,fig2,lidar, control, x ,m, b, ind, aRecta)

% Tiempo de simulacion
sampleTime = 0.05;             % Sample time [s]
t = 0:sampleTime:100;         % Time array
% Array para guardar valores de posicion y rotacion
poses = zeros(3,numel(t));    % Pose matrix
poses(:,1) = initPose';

% Tasa de iteracion de estado
r = rateControl(1/sampleTime);

% Obtener los ejes de las figuras
ax1 = fig1.CurrentAxes;
ax2 = fig2.CurrentAxes;
% Array para guardar los angulos 
qA = [];
    % Ciclo de inicio hasta que termina el tiempo de simulacion
    for idx = 1:numel(t)
        % Obtener posicion actual
        position = poses(:,idx)';
        currPose = position(1:2);
        
        % Distancia entre la meta y la posicion actual 
        dist = norm(goal'-currPose);   

        % Terminar si el robot llega a la meta con tolerancia de 0.3
        if (dist < .3)
            disp("Llegue a la meta!!")
            break;
        end
        
        % Calcular los valores de y de la recta dependiendo del valor
        % actual de x
        y = m*currPose(1) + b;        
        
        % Actualizar el mapa con las medidas del sensor
        figure(2)
        [ranges, angles] = lidar(position, map1);
        scan = lidarScan(ranges,angles);
        validScan = removeInvalidData(scan,'RangeLimits',[0,lidar.Range(2)]);
        insertRay(map2,position,validScan,lidar.Range(2));
        show(map2);

        % Condicion que avisa si el robot encontro la linea
        if ( (find(y-currPose(2)) < 0.1 | find((y-currPose(2)) > -0.1)) & (find((x-currPose(1)) < 0.3) | find((x-currPose(1)) > 0.5)) & (currPose(2) > poses(2,1)+0.1)) 
        disp("Llegue a pendiente")
        break;
        end

        % Ejecute el controlador Pure Pursuit y convierta la salida a velocidades de rueda
        [vRef,wRef] = ppControl(poses(:,idx));
        

        vs=double(validScan.Ranges);
        an=validScan.Angles;

        vc=10;
        vl=10;
        vr=10;

        % Distancia del robot a los obstaculos
        safeDistance=3;

        for i=1:length(vs)
            if an(i)>-0.5236 && an(i)<0.5236
            if(~isnan(vs(i)))
            if(vs(i)<safeDistance&&vs(i)<vc)
                vc=vs(i);
            end
            end
            end
            if an(i)>-1.5708 && an(i)<-0.5236 
            if(~isnan(vs(i)))
            if(vs(i)<safeDistance&&vs(i)<vl)
                vl=vs(i);
            end
            end
            end
            if an(i)>0.5236 && an(i)<1.5708
            if(~isnan(vs(i)))
            if(vs(i)<safeDistance&&vs(i)<vr)
                vr=vs(i);
            end
            end
            end
        end

        if((vc<safeDistance)&&(vc<dist)||...
                (vl<safeDistance)&&(vl<dist)|| ...
                (vr<safeDistance)&&(vr<dist))
            dist;
        if vc>4
        vc=4;
        end
        if vl>4
        vl=4;
        end
        if vr>4
        vr=4;
        end
        v=[vc; vl; vr];
        % Evaluacion del control difuso
        wRef=evalfis(control,v);
        %pause;
        end

                
        % Realizar paso de integración discreta hacia adelante
        vel = derivative(diffDrive, poses(:,idx), [vRef wRef]);
        poses(:,idx+1) = poses(:,idx) + vel*sampleTime; 
       
    
        % Actualizar visualizacion
        plotTrvec = [poses(1:2, idx+1); 0];
        plotRot = axang2quat([0 0 1 poses(3, idx+1)]);
        % Guardar angulos en qA array
        qA(end+1) = poses(3, idx+1);
        

        % Eliminar la imagen del último robot para evitar mostrar varios robots
        if idx > 1 || ind > 0
           items = get(ax1, 'Children');
           delete(items(1)); 
        end
    
        % Plot robot al mapa conocido
        plotTransforms(plotTrvec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'View', '2D', 'FrameSize', 1, 'Parent', ax1);
        % Plot robot al mapa nuevo
        plotTransforms(plotTrvec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'View', '2D', 'FrameSize', 1, 'Parent', ax2);

        % Esperar para iterar a la velocidad adecuada
        waitfor(r);
        
    end
    % Salidas de la funcion (posicion y rotacion final)
    currX = currPose(1);
    currY = currPose(2);
    %ang = qA(1, end)
    ang = aRecta;
end



