% % Seguimiento de pared
function [angT]=seguirPared(ranges,sensorx_R,sensory_R,sensorAngle_R,x,y,theta,seguimiento)
   d=0.2;
    ranges
    if seguimiento==1
       ang4=sensorAngle_R(4);
       d4=ranges(5);
       if isnan(d4)==1
          d4=4.5; 
       end
       xs4=sensorx_R(5);
       ys4=sensory_R(5);
       Rang4=[cosd(ang4) -sind(ang4); sind(ang4) cosd(ang4)];
       Ss4=[xs4;ys4];
       xo4=[d4;0];
       P1=(Rang4*xo4)+Ss4;
       
       ang2=sensorAngle_R(3);
       d2=ranges(3);
       if isnan(d2)==1
          d2=4.5; 
       end
       xs2=sensorx_R(3);
       ys2=sensory_R(3);
       Rang2=[cosd(ang2) -sind(ang2); sind(ang2) cosd(ang2)];
       Ss2=[xs2;ys2];
       xo2=[d2;0];
       P2=(Rang2*xo2)+Ss2;
       pared=P2-P1;
       norm(pared)
       paredG=([cos(theta) -sin(theta); sin(theta) cos(theta)]*pared)+[x;y];
       anguloP=atan2(paredG(2),paredG(1))*180/pi
       paredn=pared/norm(pared);
       pared_p=P1-((transpose(P1))*paredn)*paredn;
       pared_pn=pared_p/(norm(pared_p));
       seguir_pared_ad=d*paredn+(pared_p-d*pared_pn);
       
       Rang=[cos(theta) -sin(theta); sin(theta) cos(theta)];
       Ss=[x;y];
       Usp=(Rang*seguir_pared_ad)+Ss;
       angR=atan2(Usp(2),Usp(1));
       angT=angR
%        if ranges(1)<0.1 || ranges(3)<0.1 || ranges(5)<0.1
%            angT=evitarObstaculos(ranges,sensorAngle_R,x,y,theta);
%        else
%            angT=angR;
%        end
   else
       angT=0;
    end
    pause(1);
end