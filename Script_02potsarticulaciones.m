%% Limpia la memora de variables
clear all
close all
clc

%% Cierra y elimina cualquier objeto de tipo serial 
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end
%% Creación de un objeto tipo serial
arduino = serial('COM3','BaudRate',9600,'Terminator','CR/LF');
fopen(arduino);
if arduino.status == 'open'
    disp('Arduino conectado correctamente \n');
else
    disp('No se ha conectado el arduino \n');
    return
end

%%


%%
d='introduzca l1:';%distancia 1
l1=input(d);

e='introduzca l2:';%distancia 2
l2=input(d);

%%
p1= [0;0;0]'; %punto inicial (el apostrofe hace vector tipo columna)

%%
while (true)
    
 clf
 printAxis();%imprime eje x,y
 
 
    a = fscanf(arduino,'%d,%d')'
    v1 = a(1);
    v2 = a(2);
    
 
 theta1_deg = ((v1(1))-512)*130/512;
 theta1_rad = deg2rad(theta1_deg);
    
    
% valor_con_offset = fscanf(arduino,'%d');
% theta1_deg = ((valor_con_offset(1))-512)*130/512;
% theta1_rad = deg2rad(theta1_deg);
%  
    
 TRz1=[cos(theta1_rad ) -sin(theta1_rad ) 0 0;sin(theta1_rad ) cos(theta1_rad ) 0 0;0 0 1 0;0 0 0 1]; % primera rotacion
 TTx1=[1 0 0 l1;0 1 0 0;0 0 1 0;0 0 0 1];% primera traslacion
 T1=TRz1*TTx1;
 p2=T1(1:3,4);
 eje_x1=T1(1:3,1);
 eje_y1=T1(1:3,2);
 line([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'color',[0.5 0 0.8],'linewidth',4);
 line([p1(1) eje_x1(1)],[p1(2) eje_x1(2)],[p1(3) eje_x1(3)],'color',[0.5 0 0],'linewidth',5);
 line([p1(1) eje_y1(1)],[p1(2) eje_y1(2)],[p1(3) eje_y1(3)],'color',[0 0.5 0],'linewidth',5);

 %segundo eslabon
  
 theta2_deg = ((v2(1))-512)*130/512;
 theta2_rad2 = deg2rad(theta2_deg);
 
printAxis();
TRy2=[cos(theta2_rad2) -sin(theta2_rad2) 0 0;sin(theta2_rad2) cos(theta2_rad2) 0 0;0 0 1 0;0 0 0 1];
TTx2=[1 0 0 l2;0 1 0 0;0 0 1 0;0 0 0 1];
T2=TRy2*TTx2;
tf=T1*T2
p3=tf(1:3,4)
 eje_x2=p2+tf(1:3,1);
 eje_y2=p2+tf(1:3,2);
 
 eje_x3=p3+tf(1:3,1);
 eje_y3=p3+tf(1:3,2);
 line([p2(1) p3(1)],[p2(2) p3(2)],[p2(3) p3(3)],'color',[0 0.5 0.8],'linewidth',4);
 line([p2(1) eje_x2(1)],[p2(2) eje_x2(2)],[p2(3) eje_x2(3)],'color',[0.5 0 0],'linewidth',4); %se manda a imprimir la segunda articulacion
 line([p2(1) eje_y2(1)],[p2(2) eje_y2(2)],[p2(3) eje_y2(3)],'color',[0 0.5 0],'linewidth',4); %se manda a imprimir la segunda articulacion
 
 line([p3(1) eje_x3(1)],[p3(2) eje_x3(2)],[p3(3) eje_x3(3)],'color',[0.5 0 0],'linewidth',4); %se manda a imprimir la segunda articulacion
 line([p3(1) eje_y3(1)],[p3(2) eje_y3(2)],[p3(3) eje_y3(3)],'color',[0 0.5 0],'linewidth',4); %se manda a imprimir la segunda articulacion
 
pause(0.01)



end



%% Limpiar la escena del crimen
fclose(arduino);
delete(arduino);
clear arduino;





