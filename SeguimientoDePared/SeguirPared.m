% % Seguimiento de pared
function [angR]=SeguirPared(ranges,sensorx_R,sensory_R,sensorAngle_R,x,y,theta,seguimiento)
    d=0.5;%distancia
    %ranges
    theta1=theta*180/pi;
    if seguimiento==1
    % Componentes del sensor 4, izquierdo atras
    ang4=sensorAngle_R(5);
    d4=ranges(5);
    if isnan(d4)==1
        d4=4.5;
    end
    xs4=sensorx_R(5);
    ys4=sensory_R(5);
    Rang4=[cosd(ang4) -sind(ang4); sind(ang4) cosd(ang4)];
    Ss4=[xs4;ys4];
    xso4=[d4;0];
    P1=(Rang4*xso4)+Ss4;
    % Componentes del sensor 2, izquierdo adelante
    ang2=sensorAngle_R(3);
    d2=ranges(3);
    if isnan(d2)==1
        d2=4.5;
    end
    xs2=sensorx_R(3);
    ys2=sensory_R(3);
    Rang2=[cosd(ang2) -sind(ang2); sind(ang2) cosd(ang2)];
    Ss2=[xs2;ys2];
    xso2=[d2;0];
    P2=(Rang2*xso2)+Ss2;
    % Hallamos el vector pared
    pared=P2-P1;
    % Normalizamos el vector pared
    paredn=(transpose(pared))/(norm(pared));
    paredn=transpose(paredn);
    % Calculamos el vector perpendicular a la pared
    pared_p=P1-((transpose(P1))*paredn)*paredn;
    % Calculamos la direccion del vector pared_p
    pared_pn=(transpose(pared_p))/(norm(pared_p));
    pared_pn=transpose(pared_pn);
     % Calculamos seguir pared
    seguir_pared_ad=d*paredn+(pared_p-d*pared_pn);
    % Calculamos el marco de referencia inercial
    Rang=[cosd(theta) -sind(theta); sind(theta) cosd(theta)];
    Ss=[x;y];
    Usp=(Rang*seguir_pared_ad)+Ss;
    % Calculo del angulo
    thetag=atan2(Usp(1),Usp(2))-180;
    if ranges(1)<0.5 & isnan(d4)==0
        angR=evitarObstaculos(ranges,sensorx_R,sensory_R,sensorAngle_R,x,y,theta);
        evitar=1;
    else
        angR=thetag;
        evitar=2;
    end
    if d2<0.5
        angR=evitarObstaculos(ranges,sensorx_R,sensory_R,sensorAngle_R,x,y,theta);
        evitar=1
    end
    if d4<0.5
        angR=evitarObstaculos(ranges,sensorx_R,sensory_R,sensorAngle_R,x,y,theta);
        evitar=1
    end
    else
% Seguimiento de pared

% function [angR]=SeguirPared(ranges,sensorx_R,sensory_R,sensorAngle_R,x,y,theta)
    d=1;%distancia
    ranges
    theta1=theta*180/pi;
    % Componentes del sensor 4, izquierdo atras
    ang4=sensorAngle_R(4);
    d4=ranges(4);
    if isnan(d4)==1
        d4=4.5;
    end
    xs4=sensorx_R(4);
    ys4=sensory_R(4);
    Rang4=[cosd(ang4) -sind(ang4); sind(ang4) cosd(ang4)];
    Ss4=[xs4;ys4];
    xso4=[d4;0];
    P1=(Rang4*xso4)+Ss4;
    % Componentes del sensor 2, izquierdo adelante
    ang2=sensorAngle_R(2);
    d2=ranges(2);
    if isnan(d2)==1
        d2=4.5;
    end
    xs2=sensorx_R(2);
    ys2=sensory_R(2);
    Rang2=[cosd(ang2) -sind(ang2); sind(ang2) cosd(ang2)];
    Ss2=[xs2;ys2];
    xso2=[d2;0];
    P2=(Rang2*xso2)+Ss2;
    % Hallamos el vector pared
    pared=P2-P1;
    % Normalizamos el vector pared
    paredn=(transpose(pared))/(norm(pared));
    paredn=transpose(paredn);
    % Calculamos el vector perpendicular a la pared
    pared_p=P1-((transpose(P1))*paredn)*paredn;
    % Calculamos la direccion del vector pared_p
    pared_pn=(transpose(pared_p))/(norm(pared_p));
    pared_pn=transpose(pared_pn);
     % Calculamos seguir pared
    seguir_pared_ad=d*paredn+(pared_p-d*pared_pn);
    % Calculamos el marco de referencia inercial
    Rang=[cosd(theta) -sind(theta); sind(theta) cosd(theta)];
    Ss=[x;y];
    Usp=(Rang*seguir_pared_ad)+Ss;
    % Calculo del angulo
    thetag=-atan2(Usp(1),Usp(2))+180;
    if ranges(1)<0.5 & isnan(ranges(1))==0
        angR=evitarObstaculosI(ranges,sensorx_R,sensory_R,sensorAngle_R,x,y,theta);
        evitar=1
    else
        angR=-theta+thetag;
        evitar=2
    end
    if d2<0.8
        angR=evitarObstaculosI(ranges,sensorx_R,sensory_R,sensorAngle_R,x,y,theta);
        evitar=1
    end
    if d4<0.8
        angR=evitarObstaculosI(ranges,sensorx_R,sensory_R,sensorAngle_R,x,y,theta);
        evitar=1
    end
end