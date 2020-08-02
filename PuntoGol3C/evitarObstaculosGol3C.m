function [angR,vD]=evitarObstaculosGol3C(ranges,sensorAngle_R,x,y,theta,xd,yd,sensorx_R,sensory_R)
angGTG=wrapToPi(atan2(yd-y,xd-x));
rangesF=ranges(1,:);
aux=find(isnan(rangesF));
rangesF(aux)=4.5;
%ks=[0.6, 1.1, 1.1, 1.4, 1.4];
%rangesA=rangesF.*ks;
aux2=find(rangesF<0.5);
if(~isempty(aux2))
    O_R=[0;0];
    aux3=find(rangesF<0.3);
    if(~isempty(aux3))
        for i=1:length(aux3)
            d=rangesF(1,aux3(i));
            Rang=[cosd(sensorAngle_R(aux3(i))) -sind(sensorAngle_R(aux3(i))); sind(sensorAngle_R(aux3(i))) cosd(sensorAngle_R(aux3(i)))];
            Ss=[sensorx_R(aux3(i));sensory_R(aux3(i))];
            xso=[d;0];
            %xo=(Rang*xso/ks(aux3(i)))+Ss;
            xo=(Rang*xso)+Ss;
            O_R=O_R+xo; 
        end    
       Rthetav=[cos(theta) -sin(theta);
                sin(theta) cos(theta)];
       dir_g=(Rthetav*O_R);
       angA=atan2(dir_g(2),dir_g(1));
    else
        for i=1:length(aux2)
            d=rangesF(1,aux2(i));
            Rang=[cosd(sensorAngle_R(aux2(i))) -sind(sensorAngle_R(aux2(i))); sind(sensorAngle_R(aux2(i))) cosd(sensorAngle_R(aux2(i)))];
            Ss=[sensorx_R(aux2(i));sensory_R(aux2(i))];
            xso=[d;0];
            xo=(Rang*xso)+Ss;
            %xo=(Rang*xso/ks(aux2(i)))+Ss;
            O_R=O_R+xo; 
        end    
        Rthetav=[cos(theta) -sin(theta);
                sin(theta) cos(theta)];
        dir_g=(Rthetav*O_R);
        angG=atan2(dir_g(2),dir_g(1));
        alfa=0.6;
        angA=alfa*angG+(1-alfa)*angGTG;
    end
    if(angA>pi/2)
            angR=wrapToPi(angA+pi);
       else
            angR=wrapTo2Pi(angA+pi);
       end
   vD=0.1;
else
   angR=angGTG;
   vD=1;
end 
end


