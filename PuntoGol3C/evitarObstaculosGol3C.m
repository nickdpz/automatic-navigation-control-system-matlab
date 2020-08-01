% Sin ponderacion Valor maximo

function [angR,vD]=evitarObstaculosGol3C(ranges,sensorAngle_R,x,y,theta,xd,yd)
sensory_R = 0.001*[  0   -33.5   33.5    -41     41]';
sensorx_R = 0.001*[ 178  128.5   128.5   20.5    20.5]';
angGTG=wrapToPi(atan2(yd-y,xd-x));
go=2;
O_R=[0;0];
for i=1:length(sensorx_R)
    ang=sensorAngle_R(i);
    xs=sensorx_R(i);
    ys=sensory_R(i);
    d=ranges(i);
    if isnan(d)==1
       d=4.5;
    end    
    Rang=[cosd(ang) -sind(ang); sind(ang) cosd(ang)];
    Ss=[xs;ys];
    xso=[d;0];
    xo=(Rang*xso)+Ss;
    O_R=O_R+xo;
    
    if d<1
        go=1;
    end
    if d>1
        go=3;
    end
    if d<0.5
        go=4;
    end
end
if go==1

   Rthetav=[cos(theta) -sin(theta);
            sin(theta) cos(theta)];
   Pv=[x;y];
   O_G=(Rthetav*O_R)+Pv;
   dir_o=O_G-Pv;
   dir_eo=atan2(dir_o(2),dir_o(1));
   angAO=dir_eo
   alfa=0.5;
   angR=alfa*angAO+(1-alfa)*angGTG;
   vD=0.6;
   go=2;
   evitar=1;
end
if go==3
   angR=angGTG;
   vD=1;
   go=2;
   evitar=2;
end
if go==4
    Rthetav=[cos(theta) -sin(theta);
            sin(theta) cos(theta)];
   Pv=[x;y];
   O_G=(Rthetav*O_R)+Pv;

   dir_o=O_G-Pv;

   dir_eo=atan2(dir_o(2),dir_o(1));
   angAO=theta-dir_eo;
   angR=angAO

   vD=0.5;
   go=2;
   evitar=3;
end
     evitar=4;
end