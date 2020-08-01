% Sin ponderacion Valor maximo

function [angG,vD]=evitarObstaculosGol3C(ranges,sensorAngle_R,x,y,theta,xd,yd)
sensory_R = 0.001*[  0   -33.5   33.5    -41     41]'; %Coordenadas en X en mm
sensorx_R = 0.001*[ 178  128.5   128.5   20.5    20.5]';
rangesF=ranges;
aux=find(isnan(rangesF));
rangesF(aux)=4.5;
%aux2=find(rangesF<0.45);
O_R=[0;0];
for i=1:(length(sensorx_R))
    d=rangesF(i);
    Rang=[cosd(sensorAngle_R(i)) -sind(sensorAngle_R(i)); sind(sensorAngle_R(i)) cosd(sensorAngle_R(i))];
    Ss=[sensorx_R(i);sensory_R(i)];
    xso=[d;0];
    xo=(Rang*xso)+Ss;
    O_R=O_R+xo;  
end    
Rthetav=[cos(theta) -sin(theta);
        sin(theta) cos(theta)];
dir_g=(Rthetav*O_R);
%ang=abs(atan2(dir(2),dir(1)));
Pv=[x;y];
dir_o=dir_g+Pv;
ugtg=[yd-y;xd-x];
uao=[dir_o(2);dir_o(1)];
ugtgn=ugtg/(norm(ugtg));
uaon=uao/(norm(uao));
alfa=1;
uaogtg=alfa*uaon+(1-alfa)*ugtgn;
angR=-atan2(uaogtg(2),uaogtg(1))+pi;
angG=angR;
angG*180/pi
vD=0.1;
end