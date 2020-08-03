function [angR,vD]=evitarObstaculosGol3C(ranges,sensorAngle_R,x,y,theta,xd,yd,sensorx_R,sensory_R)
angGTG=atan2(yd-y,xd-x);
rangesF=ranges(1,:);
aux=find(isnan(rangesF));
rangesF(aux)=4.5;
ks=[0.6, 1.1, 1.1, 1.4, 1.4];
rangesA=rangesF.*ks;
aux2=find(rangesA<0.45);
if(~isempty(aux2))
    O_R=[0;0];
    aux3=find(rangesA<0.3);
    if(~isempty(aux3))
        for i=1:length(aux3)
            d=rangesA(1,aux3(i));
            Rang=[cosd(sensorAngle_R(aux3(i))) -sind(sensorAngle_R(aux3(i))); sind(sensorAngle_R(aux3(i))) cosd(sensorAngle_R(aux3(i)))];
            Ss=[sensorx_R(aux3(i));sensory_R(aux3(i))];
            xso=[d;0];
            xo=(Rang*xso/ks(aux3(i)))+Ss;
            %xo=(Rang*xso)+Ss;
            O_R=O_R+xo; 
        end    
       Rthetav=[cos(theta) -sin(theta);
                sin(theta) cos(theta)];
       dir_g=(Rthetav*O_R);
       angA=atan2(dir_g(2),dir_g(1));
    else
        for i=1:length(aux2)
            d=rangesA(1,aux2(i));
            Rang=[cosd(sensorAngle_R(aux2(i))) -sind(sensorAngle_R(aux2(i))); sind(sensorAngle_R(aux2(i))) cosd(sensorAngle_R(aux2(i)))];
            Ss=[sensorx_R(aux2(i));sensory_R(aux2(i))];
            xso=[d;0];
            %xo=(Rang*xso)+Ss;
            xo=(Rang*xso/ks(aux2(i)))+Ss;
            O_R=O_R+xo; 
        end    
        Rthetav=[cos(theta) -sin(theta);
                sin(theta) cos(theta)];
        dir_g=(Rthetav*O_R);
        dir_g=dir_g/norm(dir_g);
        alfa=0.3;
        %dd=abs(sqrt((xd-x)^2+(yd-y)^2));
        %[utgn_x,utgn_y]=pol2cart(angGTG,dd);
        utgn=[xd-x,xd-x];
        utgn=utgn/norm(utgn);
        vector=alfa*dir_g+(1-alfa)*utgn';
        angA=atan2(vector(2),vector(1));
    end
    if(angA>pi/2&&angA<3*pi/5)
       angR=pi;
    elseif(angA>pi/3&&angA<2*pi/3)
        angR=wrapToPi(angA+pi); 
    else
        angR=wrapTo2Pi(angA+pi);
    end
   vD=0.1;
else
   angR=wrapToPi(angGTG);
   vD=1;
end 
end


