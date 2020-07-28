clear all
close all
clc

% definimos el robot

kinematicModel = differentialDriveKinematics;
kinematicModel.WheelRadius = (65.65/2)*10^-3;% Diametro de 66.5mm
kinematicModel.TrackWidth = 19.80*10^-3;%Ancho de la rueda de 19.80mm
kinematicModel.WheelSpeedRange = [-10  10]*2*pi;
initialState = [2  2  0*pi/180];   % pose => position in [m], and orientation [deg]
%Posicion inicial en (2,2)
% mapa
image = imread('../Images/mapa1.png');

grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;

% convMap = binaryOccupancyMap(source,resolution);
convMap = binaryOccupancyMap(bwimage,40);
refFigure = figure('Name','SimpleMap');
show(convMap);

% Get the axes from the figure
ax1 = refFigure.CurrentAxes;

% definicion de los sensores
sensor = rangeSensor;
sensor.Range = [0.1,4.5];% Rango del Sensor (2cm a 450cm)
sensor.HorizontalAngle = [-7.5  7.5]*pi/180; %Angulo del sensor

% ubicacion de los sensores
sensorx_R = 0.001*[  0   -33.5   33.5    -41     41]'; %Coordenadas en X en mm
sensory_R = 0.001*[ 178  128,5   128,5   20.5    20.5]';%Coordenadas en Y en mm
sensorAngle_R = [   0    -45     45     -90      90]';

% %tspan = 0:0.05:1;
% %inputs = [10 30]; % (1) wL y (2) wR
% %[t,y_car] = ode45(@(t,y)derivative(kinematicModel,y,inputs),tspan,initialState);
% 
% path = [4 6; 6.5 12.5; 4 22; 12 14; 22 22; 16 12; 20 10; 14 6; 22 3];
% % Plot the path on the reference map figure.
% 
% figure(refFigure);
% hold on
% plot(path(:,1),path(:,2), 'o-');
% hold off
% % Set the path as the waypoints of the pure pursuit controller.
% 
% controller.Waypoints = path;
% initPose = [path(1,1) path(1,2), pi/2];
% goal = [path(end,1) path(end,2)]';
% poses(:,1) = initPose';

% exampleHelperDiffDriveCtrl(diffDrive,controller,initPose,goal,refMap,map,refFigure,mapFigure,sensor)
% 
% function exampleHelperDiffDriveCtrl(diffDrive,ppControl,initialState,goal,map1,map2,fig1,fig2,lidar)
sampleTime = 0.05;             % Sample time [s]
dt = sampleTime;
t = 0:sampleTime:10;         % Time array
poses = zeros(3,numel(t));    % Pose matrix
poses(:,1) = initialState';
% poses1(:,1) = initialState';
% poses2(:,1) = initialState';

%set rate to iterate at
r = rateControl(1/sampleTime);      % rateControl ejecuta el loop a una frecuencia fija
xg=[];
yg=[];
for idx = 1:numel(t)
    position = poses(:,idx)';
    currPose = position(1:2);
    
    theta = poses(3,idx);
    theta = wrapToPi(theta);
    x = currPose(1);
    y = currPose(2);
    yg=[yg y];
    xg=[xg x];
    % Take sensor measurements
    for k = 1:length(sensorx_R)
        sensor_G(:,k) = [cosd(theta*180/pi)  -sind(theta*180/pi);
                         sind(theta*180/pi)   cosd(theta*180/pi)]*[sensorx_R(k);  sensory_R(k)] + [x; y];
        
        d_R(:,k) = [cosd(sensorAngle_R(k))  -sind(sensorAngle_R(k));
                    sind(sensorAngle_R(k))   cosd(sensorAngle_R(k))]*[0.1;  0] + [sensorx_R(k);  sensory_R(k)];
        d_G(:,k) = [cosd(theta*180/pi)  -sind(theta*180/pi);
                    sind(theta*180/pi)   cosd(theta*180/pi)]*d_R(:,k) + [x; y];
        
        sensorAngle_G(k) = 180/pi*atan2(d_G(2,k)-sensor_G(2,k),d_G(1,k)-sensor_G(1,k));
        
        %figure(1), plot([x  d_G(1,k)], [y  d_G(2,k)],'b'), hold on
        %figure(1), plot([x  sensor_G(1,k)], [y  sensor_G(2,k)],'r')
        truePose(k,:) = [sensor_G(:,k)'  pi/180*sensorAngle_G(k)'];
        [ranges(:,k), angles(:,k)] = sensor(truePose(k,:), convMap);
    end
    %ranges
    if isnan(ranges(1,:)) == [1  1  1  1  1]
        wP(idx) = 0;
    else
        theta_EO(idx) = evitarObstaculos(ranges,sensorx_R,sensory_R,sensorAngle_R,x,y,theta);
        e_theta(idx) = wrapToPi(theta_EO(idx) - theta);
        wP(idx) = 1*e_theta(idx);
    end
    
    vP(idx) = 0.4;
    
    d_x(idx) = vP(idx)*cosd(theta*180/pi);
    d_y(idx) = vP(idx)*sind(theta*180/pi);
    d_theta(idx) = wP(idx);
    
    x = x + dt*d_x(idx);
    y = y + dt*d_y(idx);
    theta = theta + dt*d_theta(idx);
    
    poses(:,idx+1) = [x; y; theta];
    
%     % Perform forward discrete integration step
%     vel = derivative(kinematicModel, poses1(:,idx), [vP(idx) wP(idx)]);
%     poses1(:,idx+1) = poses(:,idx) + vel*sampleTime
%     
%     tspan = 0:dt/2:dt;
%     inputs = [33.333  33.333]; % (1) wL y (2) wR
%     [t,y_car] = ode45(@(t,y_car)derivative(kinematicModel,y_car,inputs),tspan,poses2(:,idx));
% 
%     poses2(:,idx+1) = [y_car(:,end)']
%     
    % Update visualization
    plotTrvec = [poses(1:2, idx+1); 0];
    plotRot = axang2quat([0 0 1 poses(3, idx+1)]);
    
    % Delete image of the last robot to prevent displaying multiple robots
    if idx > 1
       items = get(ax1, 'Children');
       delete(items(1)); 
    end

    % Plot robot onto known map
    plotTransforms(plotTrvec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'View', '2D', 'FrameSize', 1, 'Parent', ax1);

    % Wait to iterate at the proper rate
    waitfor(r);
end
figure(refFigure);
hold on
plot(ax1,xg,yg,'Linewidth',3);
%end