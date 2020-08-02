function [angR,vD]=evitarObstaculosGolP(ranges,sensorAngle_R,x,y,theta,xd,yd)
    sensory_R = 0.001*[  0   -33.5   33.5    -41     41]'; %Coordenadas en X en mm
    sensorx_R = 0.001*[ 178  128.5   128.5   20.5    20.5]';
    rangesF=ranges(1,:);
    aux=find(isnan(rangesF));
    rangesF(aux)=4.5;
    ks=[0.6, 1.1, 1.1, 1.4, 1.4];
    rangesA=rangesF.*ks;
    aux2=find(rangesA<0.4);
    if(~isempty(aux2))
        O_R=[0;0];
        for i=1:length(aux2)
            d=rangesF(1,aux2(i));
            Rang=[cosd(sensorAngle_R(aux2(i))) -sind(sensorAngle_R(aux2(i))); sind(sensorAngle_R(aux2(i))) cosd(sensorAngle_R(aux2(i)))];
            Ss=[sensorx_R(aux2(i));sensory_R(aux2(i))];
            xso=[d;0];
            xo=(Rang*xso/ks(aux2(i)))+Ss;
            O_R=O_R+xo; 
        end    
       Rthetav=[cos(theta) -sin(theta);
                sin(theta) cos(theta)];
       %Pv=[x;y];
       dir_g=(Rthetav*O_R);
       angG=atan2(dir_g(2),dir_g(1));
       %dir_o=dir_g+Pv;
       %angO=atan2(dir_o(2),dir_o(1))
       if(angG>pi/2)
            angR=wrapToPi(angG+pi);
       else
            angR=wrapTo2Pi(angG+pi);
       end
       vD=0.1;
    else
       angR=atan2(yd-y,xd-x);
       vD=1;
    end 
end