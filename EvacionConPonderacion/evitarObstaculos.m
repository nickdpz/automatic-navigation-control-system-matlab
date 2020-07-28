function [angR]=evitarObstaculos(ranges,sensorx_R,sensory_R,sensorAngle_R,x,y,theta)
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
       angR=wrapToPi(theta+pi-dir_eo);
    else
       angR=theta;
    end
        
end