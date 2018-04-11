% Cinematica inversa de brazo robotico con 2 grados de libertad
% Ingenieria mecatronica, Laboratorio de robotica.
clc;
close all;
disp(' Programa de cinematica inversa para un brazo de ');
disp(' 2 grados de libertad controlado con Arduino UNO - Laboratorio de Robotica ')
disp(' ');
state=1;

%while(state==1){
l1 = 11; 
l2 = 9; 
theta1 = 0:0.3:pi; 
theta2 = 0:0.3:2*pi; 

    a = arduino('COM3');
a.servoAttach(6); 
a.servoAttach(7); 
a.servoWrite(6,0);
a.servoWrite(7,0); 
    
[THETA1, THETA2] = meshgrid(theta1, theta2); 
X = l1 * cos(THETA1) + l2 * cos(THETA1 + THETA2); % 
Y = l1 * sin(THETA1) + l2 * sin(THETA1 + THETA2); % 

data1 = [X(:) Y(:) THETA1(:)]; 
data2 = [X(:) Y(:) THETA2(:)]; %

plot(X(:), Y(:), 'r.');
  axis equal;
  xlabel('X','fontsize',10)
  ylabel('Y','fontsize',10)
  title('X-Y posibles para el brazo','fontsize',10)
  grid;
  
disp('Abriendo librerias fuzzy ......')
%disp(' ');
anfis1 = anfis(data1, 7, 150, [0,0,0,0]); % 
disp('Generando las matrices fuzzy ......')
disp(' ');
anfis2 = anfis(data2, 6, 150, [0,0,0,0]); % 
while true

disp('Elija la posicion deseada en la grafica siguiente')
disp(' ');


[x y]=ginput(1);

fprintf('Para X elegido %5.3f\n',x);
fprintf('Para Y elegido %5.3f\n',y);

[X, Y] = meshgrid(x,y);

c2 = (X.^2 + Y.^2 - l1^2 - l2^2)/(2*l1*l2);
s2 = sqrt(1 - c2.^2);
THETA2D = atan2(s2, c2); % theta2 

k1 = l1 + l2.*c2;
k2 = l2*s2;
THETA1D = atan2(Y, X) - atan2(k2, k1); % theta1 
% 
% disp(' ');
% disp('Usando calculo numerico:');
%disp(' ');

th1 = (THETA1D*180)/pi;
% fprintf('El valor de Theta1 es %5.3f\n',th1');
th2 = (THETA2D*180)/pi;
% fprintf('El valor de Theta2 es %5.3f\n',th2');

XY = [X(:) Y(:)];
THETA1P = evalfis(XY, anfis1); % theta1
THETA2P = evalfis(XY, anfis2); % theta2 
% disp(' ');
% disp('Usando calculo matricial:');

th1a = (THETA1P*180)/pi;
% fprintf('El valor de Theta1* es %5.3f\n',th1a');
th2a = (THETA2P*180)/pi;
% fprintf('El valor de Theta2* es %5.3f\n',th2a');

theta1diff = THETA1D(:) - THETA1P;
theta2diff = THETA2D(:) - THETA2P;
% disp(' ');
% disp('Promedio de ambos metodos:');
%disp(' ');

th1_prom = (th1+th1a)/2;
fprintf('Theta1 es %5.0f\n',th1_prom');
th2_prom = (th2+th2a)/2;
fprintf('Theta2 es %5.0f\n',th2_prom');

th1_entero = ceil(th1_prom);
th2_entero = ceil(th2_prom);

a.servoWrite(7,th1_entero);
pause(2);
a.servoWrite(6,th2_entero);
pause(10);
a.servoWrite(6,0);
a.servoWrite(7,0);
end
% Grados en el motor DC -- 2º = 1%  se tiene que dividir entre 2
% Grados en el motor PAP -- 3º = 1%  se tiene que dividir entre 3

% th1_porcentaje= ceil(th1_entero/1.6);
% th2_porcentaje= ceil(th2_entero/2.5);
% 
% if th1_porcentaje >9
%     th1_final=9;
% else
%     th1_final=th1_porcentaje;
% end
% 
% if th2_porcentaje>35
%     th2_final=33;
% else
%     th2_final=th2_porcentaje;
% end


fprintf('Porcentaje en el eje DC es %5.0f\n',th1_final');
fprintf('Porcentaje en el eje PAP es %5.0f\n',th2_final');

% %---------------------------------Serial -----------------
% %-----------------------------------------------------------------
% SerPIC = serial('COM6');
% set(SerPIC,'BaudRate',9600);
% set(SerPIC,'DataBits',8);
% set(SerPIC,'Parity','none');
% set(SerPIC,'StopBits',1);
% set(SerPIC,'FlowControl','none');
% 
% disp(' ');
% disp('Abriendo puerto serial')
%     fopen(SerPIC);
%     fwrite(SerPIC, th1_final, 'uint8' );
%     fwrite(SerPIC, th2_final, 'uint8' );
%     fclose(SerPIC);
% disp('Cerrando puerto serial')

 
    
    



