% % Seguimiento de pared
function [angT,vD]=seguirPared(ranges,sensorx_R,sensory_R,sensorAngle_R,x,y,theta,seguimiento)
   d=0.5;
   ranges;
    if seguimiento==1
        d=1;
    ang4=sensorAngle_R(5);
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
       paredn=pared/norm(pared);
       %pared_p=P1-((transpose(P1))*paredn)*paredn;
       pared_p = - (P1-((transpose(P1))*paredn)*paredn);
       pared_pn=pared_p/(norm(pared_p));
       seguir_pared_ad=d*paredn+(pared_p-d*pared_pn);
       angT = (atan2(seguir_pared_ad(2),seguir_pared_ad(1))-theta);
%        Rang=[cos(theta) -sin(theta); sin(theta) cos(theta)];
%        Ss=[x;y];
%        Usp1=(Rang*seguir_pared_ad);
%        angR1=atan2(Usp1(2),Usp1(1));
%        Usp2=(Rang*seguir_pared_ad)+Ss;
%        angR2=atan2(Usp2(2),Usp2(1));
       d1=ranges(1);
%        angT=theta+angR2;
       vD=0.4;
       if isnan(d1)==1
           d1=4.5;
       end
       if d1<0.5 || d2<0.2 || d4<0.2
          % angT=evitarObstaculos(ranges,sensorAngle_R,x,y,theta);
           angT=-(90*pi/180)+theta;
           vD=0.2;
       end
       
%        if d2>1.5 && d2<4.5
%           angT=theta-pi;
%           vD=0.2;
%        end
%        prom=(d2+d4)/2;
%        
%        if prom>1 && d2<4.5 && d4<4.5
%           angT=(90*pi/180)+theta;
%           vD=0.1;
%        end
        if d2>1 && d2<4.5 
          angT=(50*pi/180)+theta;
          vD=0.2;
        end
%        if d4>1 && d4<4.5 
%           angT=(90*pi/180)+theta;
%           vD=0.1;
%        end

    else
       % Seguimiento por derecha
      ang4=sensorAngle_R(4);
       d4=ranges(4);
       if isnan(d4)==1
          d4=4.5; 
       end
       xs4=sensorx_R(4);
       ys4=sensory_R(4);
       Rang4=[cosd(ang4) -sind(ang4); sind(ang4) cosd(ang4)];
       Ss4=[xs4;ys4];
       xo4=[d4;0];
       P1=(Rang4*xo4)+Ss4;
       
       ang2=sensorAngle_R(2);
       d2=ranges(2);
       if isnan(d2)==1
          d2=4.5; 
       end
       xs2=sensorx_R(2);
       ys2=sensory_R(2);
       Rang2=[cosd(ang2) -sind(ang2); sind(ang2) cosd(ang2)];
       Ss2=[xs2;ys2];
       xo2=[d2;0];
       P2=(Rang2*xo2)+Ss2;
       pared=P2-P1;
       paredn=pared/norm(pared);
       pared_p=P1-((transpose(P1))*paredn)*paredn;
       %pared_p = - (P1-((transpose(P1))*paredn)*paredn);
       pared_pn=pared_p/(norm(pared_p));
       seguir_pared_ad=d*paredn+(pared_p-d*pared_pn);
       angT = (atan2(seguir_pared_ad(2),seguir_pared_ad(1))+theta);
%        Rang=[cos(theta) -sin(theta); sin(theta) cos(theta)];
%        Ss=[x;y];
%        Usp1=(Rang*seguir_pared_ad);
%        angR1=atan2(Usp1(2),Usp1(1));
%        Usp2=(Rang*seguir_pared_ad)+Ss;
%        angR2=atan2(Usp2(2),Usp2(1));
       d1=ranges(1);
       vD=0.5;
%        angT=theta+angR2;
       if isnan(d1)==1
           d1=4.5;
       end
       if d1<0.5 %|| d2<0.2 || d4<0.2
           %angT=evitarObstaculos(ranges,sensorAngle_R,x,y,theta);
           angT=(90*pi/180)+theta;
           vD=0.1;
       end
       
       if d2>1 && d2<4.5
          angT=theta-(50*pi/180);
          vD=0.4;
       end
%        prom=(d2+d4)/2;
% %        
%        if prom>1 && d2<4.5 && d4<4.5
%           angT=-(90*pi/180)+theta;
%           vD=0.1;
%        end
%        if d4>1 && d4<4.5
%           angT=-(90*pi/180)+theta;
%           vD=0.1;
%        end
%        if d1<d || d2<d || d4<d
%            angT=angR2;
%        else
%            angT=angR1;
%        end
       
    end
    angT*180/pi;
%     pause(1);
end