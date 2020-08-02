% Sin ponderacion Valor maximo

function [angR,vD]=evitarObstaculosGolP(ranges,sensorx_R,sensory_R,sensorAngle_R,x,y,theta,xd,yd)
rangesF=ranges(1,:);
aux=find(isnan(rangesF));
rangesF(aux)=4.5;
ks=[1, 2, 2, 4, 4];
rangesF=rangesF.*ks;
aux2=find(rangesF<0.6);
if(~isempty(aux2))
    ks=[1, 0.25, 0.25, 0.125, 0.125];
    aux=find((rangesF>1.8));
    rangesF(aux)=0;
    O_R=[0;0];
    
    for i=1:length(sensorx_R)
        d=rangesF(1,i);
        Rang=[cosd(sensorAngle_R(i)) -sind(sensorAngle_R(i)); sind(sensorAngle_R(i)) cosd(sensorAngle_R(i))];
        Ss=[sensorx_R(i);sensory_R(i)];
        xso=[d;0];
        xo=(ks(i)*Rang*xso)+Ss;
        O_R=O_R+xo; 
    end    
    Rthetav=[cos(theta) -sin(theta);
            sin(theta) cos(theta)];
   Pv=[x;y];
   dir=(Rthetav*O_R);
   dir_o=dir+Pv;
   dir_eo=atan2(dir_o(2),dir_o(1));
   ang=abs(atan2(dir(2),dir(1)));
   if(((theta>19*pi/40))&&((ang>9*pi/20)||(33*pi/20>ang)))
       angR=wrapToPi(theta+pi-dir_eo);
   else
       angR=0;
   end
   vD=0.1;
else
   angR=atan2(yd-y,xd-x);
   vD=2;
end
        
end

% Con Ponderacion, Valor maximo

% function [angR]=evitarObstaculos(ranges,sensorx_R,sensory_R,sensorAngle_R,x,y,theta)
% go=2;
% O_R=[0;0];
% ks=[1, 0.5, 0.5, 0.25, 0.25];
% for i=1:length(sensorx_R)
%     ang=sensorAngle_R(i);
%     xs=sensorx_R(i);
%     ys=sensory_R(i);
%     d=ranges(i);
%     if isnan(d)==1
%        d=4.5
%     end    
%     Rang=[cosd(ang) -sind(ang); sind(ang) cosd(ang)];
%     Ss=[xs;ys];
%     xso=[d;0];
%     xo=(ks(i)*Rang*xso)+Ss;
%     
%     O_R=O_R+xo;
%     
%     if d<0.6
%         go=1;
%     end
% end
%     if go==1
%        
%        Rthetav=[cosd(theta) -sind(theta);
%                 sind(theta) cosd(theta)];
%        Pv=[x;y];
%        O_G=(Rthetav*O_R)+Pv;
% 
%        dir_o=O_G-Pv;
% 
%        dir_eo=atan2(dir_o(2),dir_o(1))-180;
%        angR=theta-dir_eo
%        
%        go=2;
%     else
%        angR=theta;
%     end
%         
% end

% Sin ponderacion Valor minimo

% function [angR]=evitarObstaculos(ranges,sensorx_R,sensory_R,sensorAngle_R,x,y,theta)
% go=2;
% O_R=[0;0];
% for i=1:length(sensorx_R)
%     ang=sensorAngle_R(i);
%     xs=sensorx_R(i);
%     ys=sensory_R(i);
%     d=ranges(i);
%     if isnan(d)==1
%        d=0
%        xo=0;
%     else
%         Rang=[cosd(ang) -sind(ang); sind(ang) cosd(ang)];
%         Ss=[xs;ys];
%         xso=[d;0];
%         xo=(Rang*xso)+Ss;
%     end
%     O_R=O_R+xo;
%     
%     if d<0.6 & d>0
%         go=1;
%     end
% end
%     if go==1
%        
%        Rthetav=[cosd(theta) -sind(theta);
%                 sind(theta) cosd(theta)];
%        Pv=[x;y];
%        O_G=(Rthetav*O_R)+Pv;
% 
%        dir_o=O_G-Pv;
% 
%        dir_eo=atan2(dir_o(2),dir_o(1))-180;
%        angR=theta-dir_eo
%        
%        go=2;
%     else
%        angR=theta;
%     end
%         
% end

% Con Ponderacion, Valor minimo
% 
% function [angR]=evitarObstaculos(ranges,sensorx_R,sensory_R,sensorAngle_R,x,y,theta)
% go=2;
% O_R=[0;0];
% ks=[1, 0.5, 0.5, 0.25, 0.25];
% for i=1:length(sensorx_R)
%     ang=sensorAngle_R(i);
%     xs=sensorx_R(i);
%     ys=sensory_R(i);
%     d=ranges(i);
%     if isnan(d)==1
%        d=0
%        xo=0;
%     else
%         Rang=[cosd(ang) -sind(ang); sind(ang) cosd(ang)];
%         Ss=[xs;ys];
%         xso=[d;0];
%         xo=(ks(i)*Rang*xso)+Ss;
%     end
%     
%     O_R=O_R+xo;
%     
%     if d<0.6 & d>0
%         go=1;
%     end
% end
%     if go==1
%        
%        Rthetav=[cosd(theta) -sind(theta);
%                 sind(theta) cosd(theta)];
%        Pv=[x;y];
%        O_G=(Rthetav*O_R)+Pv;
% 
%        dir_o=O_G-Pv;
% 
%        dir_eo=atan2(dir_o(2),dir_o(1))-180;
%        angR=theta-dir_eo
%        
%        go=2;
%     else
%        angR=theta;
%     end
%         
% end