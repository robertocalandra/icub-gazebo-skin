% author: Moritz Nakatenus, TU Darmstadt
% last update: 24. März 2015
function sensors_calculate(body_part, bending, bend_a_height, bend_a_vert, bend_width, bend_height)
 
% This script gets the orientation and positions from triangles
% from a xml files and creates thereby the several sensor positions
% of the artificial skin of the iCub with trigonometric
% calculations
%
% First there are created little triangles of 3 sensors.
% the little triangles are then composed to a big triangle
% the big triangle is then rotated and translated to its position
% this happens with each triangle which is read from the xls file

triangles=xlsread(strcat(body_part,'.xls'));

% Create isosceles triangle from position point
alpha=pi/3; % 60 degree
geg=8.7; % opposite leg
ank=geg/tan(alpha/2);

all_sensors=[];

% going through all triangles
for i=1:length(triangles)
    
    P=triangles(i,:);

    P1=[P(1)-ank; P(2)-geg];
    P2=[P(1)+ank; P(2)-geg];

    P3y=P1(2)+ank*tan(alpha);
    P3=[P(1);P3y];

    triangle=[P1,P2,P3]';

    % create little triangles to composite
    [points,sensors]=generatePatch(triangle');
    
    % now rotate everthing
    [points,sensors]=transformPoints(points',sensors',P(3),[0;0],2);

    % sensors
    drawTriangleSensors(points',sensors');
    
    all_sensors=[all_sensors,sensors];

end

x_coordinates = zeros(1,length(all_sensors(1,:)));

if(bending==1)
    % now generate bending
    % data for cosine functions
    w=bend_width; % half wave-length
    a_w=bend_a_vert; % amplitude
    h=bend_height;
    a_h=bend_a_height;
    
    % depth = x-coordinte
    % Width bending
    y_coordinates = all_sensors(1,:); % The x coord. in 2D-View
    x_coordinates = a_w*cos(pi/w*y_coordinates); % calculate bending (in x-y layer)

    % Height bending
    z_coordinates = all_sensors(2,:); % The y coord. in 2D-View
    x_coordinates = x_coordinates + a_h*cos(pi/h*z_coordinates); % calculate bending (in x-z layer)
end

% now lets add a third 'coordinate' for bending to the sensors
all_sensors=[all_sensors;x_coordinates];

% translate sensors to origin
% matrix for scaling
scale=eye(3)/1400;

all_sensors=scale*all_sensors; % scale sensors to Gazebo related positions
    
% now convert sensors into mesh format for gazebo (y-z layer)
% translate points up
trans=eye(4);
trans(:,4)=[0.022;0.23;0;1];
all_sensors=[all_sensors;ones(1,length(all_sensors(1,:)))]; % homogenize
all_sensors=trans*all_sensors;
all_sensors(4,:)=[]; % spatial coordinates

convert_gazebo=[ones(length(all_sensors),1)*0.05+all_sensors(3,:)',all_sensors(2,:)',all_sensors(1,:)',zeros(length(all_sensors),3)];
save(strcat(body_part,'.txt'), 'convert_gazebo', '-ASCII');

end

% bigTriangle=[p1,p2,p3] -> 2x3
function [TPs,sensors]=createLittleTriangle(bigTriangle)

    % first compute adjacent leg of little triangle)
    ank=(bigTriangle(1,2)-bigTriangle(1,1))/4;
    alpha=pi/3;
    
    % now compute points for triangle
    P1=bigTriangle(:,1);
    P2=[P1(1)+2*ank;P1(2)];
    p3y=P1(2)+ank*tan(alpha);
    P3=[P1(1)+ank;p3y];
    
    % compute sensors for triangle
    % generate layers (y-positions)
    little_height=P3(2)-P1(2);
    sy1=P1(2)+little_height/4;
    sy2=P1(2)+little_height/2;
    geg=sy2-sy1; % opposite leg
    ank=geg/tan(alpha);
    % generate x positions
    sx1=P3(1)-ank;
    sx2=P3(1)+ank;
    sx3=P3(1);
    sP1=[sx1;sy1];
    sP2=[sx2;sy1];
    sP3=[sx3;sy2];
    sensors=[sP1,sP2,sP3];
    
    triangle=[P1,P2,P3]';
    
    TPs=triangle;
end

% creates little triangles which fill up the patch
function [points,sensors]=generatePatch(bigTriangle)

    [bl_triangle,sensors]=createLittleTriangle(bigTriangle);
    
    % create below right triangle
    br_triangle=bl_triangle';
    
    ank_big=(bigTriangle(1,2)-bigTriangle(1,1))/2;
    geg_big=(bigTriangle(2,3)-bigTriangle(2,1))/2;
    
    % for drawing triangle
    for i=1:3
       
        br_triangle(1,i)=br_triangle(1,i)+ank_big;
    end
    
    sensors2=sensors;
    sensors3=sensors;
    
    % getting second part of triangle
    sensors2=transformPoints(sensors2,0,0,[ank_big;0],2);
    
    % getting third part of triangle
    sensors3=transformPoints(sensors3,0,0,[ank_big/2;geg_big],2);
    
    % getting fourth part of triangle
%     sensors4=transformPoints(sensors4,0,180,[ank_big/2;geg_big/3],2);
    ys=(bigTriangle(2,3)-bigTriangle(2,1))/3; % midpoint y
    
    sensors4=[bigTriangle(1,3);bigTriangle(2,1)+ys];
    
    sensors=[sensors';sensors2';sensors3';sensors4'];
   
    
    points=bigTriangle';
end

function drawTriangleSensors(triangle,sensors)

    hold on;
    d=delaunayTriangulation(triangle);
    triplot(d);
    
    drawSensors(sensors);

end

function drawSensors(sensors)
        
    figure(1);
    hold on;
    
    for i=1:length(sensors)
        
        plot(sensors(i,1),sensors(i,2),'.');
        
    end
end

function [points,point_sensors]=transformPoints(points,point_sensors,rad,trans,kind_angle)

    P1=[points(1,1);points(2,1);1];
    P2=[points(1,2);points(2,2);1];
    P3=[points(1,3);points(2,3);1];
    
    % first compute center rotation-point
    xs=P1(1)+(P2(1)-P1(1))/2;
    ys=P1(2)+(P3(2)-P1(2))/2;
    
    xs=1/3*(P1(1)+P2(1)+P3(1));
    ys=1/3*(P1(2)+P2(2)+P3(2));
    
    % now creating transformation matrices (homogen coordinates)
    % translation to spatial origin
    tS_origin=eye(3);
    tS_origin(:,3)=[-xs;-ys;1];
    % rotation matrix
    tR=eye(3);
    if(kind_angle==1)
        tR(1:2,1:2)=[cos(rad),-sin(rad);sin(rad),cos(rad)];
    else
        tR(1:2,1:2)=[cosd(rad),-sind(rad);sind(rad),cosd(rad)];
    end
    
    % matrix for back translation
    tS_back=eye(3);
    tS_back(:,3)=[xs;ys;1]+[trans;1];
    
    points=[points;ones(1,3)]; % make points homogen coordinates
        
    points=tS_back*tR*tS_origin*points;
    
    % if full triangle with sensors rotate triangle+sensors
    if(point_sensors~=0)
            
        point_sensors=[point_sensors;ones(1,length(point_sensors(1,:)))]; % make points homogen coordinates
        point_sensors=tS_back*tR*tS_origin*point_sensors;
        point_sensors(3,:)=[];
    end
    
    points(3,:)=[]; % transform to spatial coordinates
    
end
