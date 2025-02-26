clear
%% 1.- Parámetros, ecuaciones, condiciones iniciales & rango temporal
m = 1;                             
g = 9.8;                            
theta0 = 0.174;                    
omega0 = 0;                        
tf = 120;                          
FR=50;
p1=1.5*1280;
p2=1.45*720;
e1=0;
e2=0;
lvec = [];  
cvec = []; 
Evec = [];
%Programamos un bucle para obtener los valores de los 8 péndulos
%diferentes y los almacenamos en los vectores vacíos definidos previamente
%para almacenar los datos de forma compacta.
for n=0:7
T=60/(20+n);
lvec=[lvec,g/(4*pi^2)*T^2];
cvec = [cvec,g/lvec(end)];                     
Evec = [Evec,m*lvec(end)^2*omega0^2/2 + m*g*lvec(end)*(1-cos(theta0))];                              
end

%% 2.- Características del plot (ventana, colores, tamaño de las fuentes y los puntos, grosor de la líneas).
figh=figure('WindowState','fullscreen');
axis equal
axis manual
AR = 16/9;                  %'Screen aspect ratio'
%Se reserva un espacio a la derecha de la ventana con el objetivo de
%insertar texto más adelante.
%El objetivo de la siguiente línea es encajar las componentes que
%aparecerán en el vídeo de una forma organizada y estética.
axis(0.7*lvec(1)*[-1 1.5*AR -1.6*(0.7*lvec(1))^(-1)-0.7 1.5*(0.7*lvec(1))^(-1) -0.9 0.9])
ax = gca
ax.GridColor = [0.25 0.25 0.25];
ax.GridAlpha = 0.5;
box on
bgColor = [0.4 0.4 0.4]; %Color del backgroud (en el modelo de colores RGB).
fgColor = [0.1 0.1 0.1]; %Color del Foreground.
bggColor = [0.5 0.5 0.5]; %Color de fondo del Figure.
mColor = [0.635 0.0780 0.184];        %Color de la masa (granate).
sColor = [0.15 0.15 0.15];        %Color del soporte.
pColor = [1 1 0.75];        %Color del péndulo.
ma= 17;                     %Tamaño de la bola de soporte.
ms = 11;                    %Tamaño de las masas.
fs = 15;                    %Tamaño de la fuente.
lw = 2;                     %Grosor de las líneas.
set(gca,'Color',bgColor)
set(gcf,'color',bggColor);
set(gca,'xticklabel',[])
set(gca,'yticklabel',[])
set(gca,'zticklabel',[],'tickDir','none')
set(figh, 'ToolBar', 'none');
hold on
grid on
%% 3.- Características del vídeo
nameFile = 'SimplePendulum';                 
vwo = VideoWriter(nameFile,'MPEG-4');         
FR = 50;                                               
prescribedTimes = 0:1/FR:tf;  
vwo.Quality = 95;             
vwo.FrameRate = FR;         )
open(vwo);
%% 4.- Computación numérica del movimiento del péndulo con el comando ode45
%El siguiente bucle permite definir la ecuación diferencial que seguirá
%cada uno de entre los ocho péndulos que hemos definido. Los vectores
%obtenidos tras la resolución de las respectivas edos se almacenan en una
%celda para llamarlos más adelante.
for i=1:8
F =  @(t,X) [X(2);-cvec(i)*sin(X(1))];
cell{i}=F;
opciones = odeset('AbsTol',1e-8,'RelTol',1e-8);      
[t,X] = ode45(cell{i},prescribedTimes,[theta0;omega0],opciones);
cellfun{i} = X;
if i==1
E = (m*lvec(1)^2*X(:,2).^2)/2 + m*g*lvec(1)*(1-cos(X(:,1))); %Energía del péndulo de mayor longitud.
end
end
%% 5.- Frame inicial
%Creamos un bucle para dibujar cada péndulo en el instante t=0 y adaptamos
%las coordenadas añadiendo la componente 3D.
for i=1:8
xMass = lvec(i)*sin(theta0);
yMass = -lvec(i)*cos(theta0);
hCord(i) = plot3([0,xMass],[-i*0.1,-i*0.1],[0.45*lvec(1),0.45*lvec(1)+yMass],'-','Color',pColor,'LineWidth',lw);
hSupport(i) = plot3(0,-0.1*i,0.45*lvec(1),'o','MarkerSize',ma,'MarkerFaceColor',sColor,'MarkerEdgeColor',sColor);
hMass(i) = plot3(xMass,-0.1*i,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',mColor,'MarkerEdgeColor',mColor);
view([50 35]) %Establecemos la perspectiva inicial del plot.
end
%A continuación añadimos el texto que aparecerá en el vídeo. Los parámetros
%que varían con el tiempo habrá que volver a definirlos más adelante para
%el resto de frames.
%IMPORTANTE: Usamos el interpretador de texto LaTeX para los símbolos matemáticos.
h1=text(lvec(1),0,0.4*lvec(1),'\bf Physical parameters:','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
h2=text(lvec(1),0,0.3*lvec(1),['$m = ',num2str(m,3),' kg$'],'interpreter','LaTeX','FontSize',fs,'Color',fgColor);
h3=text(lvec(1),0,0.2*lvec(1),['$g = ',num2str(g,3),' m/s^2$'],'interpreter','LaTeX','FontSize',fs,'Color',fgColor);
h4=text(lvec(1),0,0.1*lvec(1),['$l_0 = ',num2str(lvec(1),3),' m$'],'interpreter','LaTeX','FontSize',fs,'Color',fgColor);
h5=text(lvec(1),0,0.0*lvec(1),'\bf Initial condition:','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
h6=text(lvec(1),0,-0.1*lvec(1),['$\theta_0 = ',num2str(theta0,6),' rad.$'],'interpreter','LaTeX','FontSize',fs,'Color',fgColor);
h7=text(lvec(1),0,-0.2*lvec(1),['$\omega_0 = ',num2str(omega0,6),' rad/s$'],'interpreter','LaTeX','FontSize',fs,'Color',fgColor);
hEnergy = text(lvec(1),0,-0.3*lvec(1),['\bf Energy: $',num2str(Evec(1),10),'$ \rm J'],'interpreter','LaTeX','FontSize',fs,'Color',fgColor);
hTime = text(lvec(1),0,-0.4*lvec(1),'\bf Time: $0$ s','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
frame = getframe(figh,[e1 e2 p1 p2]);
writeVideo(vwo,frame);
%% 6.- Loop para el resto de frames del vídeo
%Definimos todo lo que el bucle necesitará borrar.
resonancia14=text(0,0,0.6*lvec(1),'','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
resonancia13=text(0,0,0.6*lvec(1),'','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
resonancia12=text(0,0,0.6*lvec(1),'','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
resonancia23=text(0,0,0.6*lvec(1),'','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
resonancia34=text(0,0,0.6*lvec(1),'','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
hMass2 = plot3(xMass,0,0.45*lvec(1)+yMass,'','MarkerSize',ms,'MarkerFaceColor',mColor,'MarkerEdgeColor',mColor);

v = [0.75 0.75 0]; 
b = [1 1 1]; 
mColor = [0.635 0.0780 0.184]; 
a = [0.667 0.667 1]; 

nTimes = length(t);
ti=200;
%Realizamos un bucle anterior al movimiento para los primeros 4 segundos en
%los que se podrá observar una vista panorámica del sistema.
for o=1:ti
    view([50-50/200*o 35-35/200*o]) %Hacemos que la perspectiva rote.
    frame = getframe(figh,[e1 e2 p1 p2]);
   writeVideo(vwo,frame);
end
for i=1:nTimes
  if i<50
ax.GridAlpha = 0.5-1/200*(i) %Hacemos que el grid reduzca su opacidad al entrar en el modo 2D.
  end
   if i>nTimes/2-50 && i<nTimes/2 %Hacemos que el grid aumente su opacidad al entrar en el modo 3D.
ax.GridAlpha = 0.25+1/200*(i-nTimes/2+50)
    end
  delete([hCord,hMass,hMass2,hSupport,hEnergy,hTime,resonancia14,resonancia13,resonancia12,resonancia23,resonancia34]);
  view([0 0])
   for j=1:8
   xMass = lvec(j)*sin(cellfun{j}(i,1));
   yMass = -lvec(j)*cos(cellfun{j}(i,1));
   hCord(j) = plot3([0,xMass],[-j*0.1,-j*0.1],[0.45*lvec(1),0.45*lvec(1)+yMass],'-','Color',pColor,'LineWidth',lw);
   hSupport(j) = plot3(0,-0.1*j,0.45*lvec(1),'o','MarkerSize',ma,'MarkerFaceColor',sColor,'MarkerEdgeColor',sColor);
   hMass(j) = plot3(xMass,-0.1*j,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',mColor,'MarkerEdgeColor',mColor);
   if i>700 && i<802 | i>3700 && i<3802 %Añadimos a los if´s los nuevos tiempos de resonancia, pues ahora el conjunto realiza dos veces el período.
       if i>700 && i<802
        resonancia14(j) = text(-0.25,0,0.55*lvec(1),'Resonancia 1/4','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
       end
           if j==1|| j==5
           hMass2(j) = plot3(xMass,-0.1*j,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',b,'MarkerEdgeColor',b);
           elseif j==2|| j==6
           hMass2(j) = plot3(xMass,-0.1*j,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',v,'MarkerEdgeColor',v);
           elseif j==3|| j==7
           hMass2(j) = plot3(xMass,-0.1*j,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',a,'MarkerEdgeColor',a);
           else
           hMass2(j) = plot3(xMass,-0.1*j,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',mColor,'MarkerEdgeColor',mColor);
           end
      elseif i>950 && i<1052| i>3950 && i<4052
         if i>950 && i<1052
        resonancia13(j) = text(-0.25,0,0.55*lvec(1),'Resonancia 1/3','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
         end
           if j==1|| j==4 || j==7
           hMass2(j) = plot3(xMass,-0.1*j,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',mColor,'MarkerEdgeColor',mColor);
           elseif j==2|| j==5 || j==8
           hMass2(j) = plot3(xMass,-0.1*j,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',b,'MarkerEdgeColor',b);
           else
           hMass2(j) = plot3(xMass,-0.1*j,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',a,'MarkerEdgeColor',a);
           end
      elseif i>1450 && i<1552| i>4450 && i<4552
       if i>1450 && i<1552
        resonancia12(j) = text(-0.25,0,0.55*lvec(1),'Resonancia 1/2','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
       end
           if j==1|| j==3 || j==5 || j==7
           hMass2(j) = plot3(xMass,-0.1*j,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',mColor,'MarkerEdgeColor',mColor);
           else
           hMass2(j) = plot3(xMass,-0.1*j,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',b,'MarkerEdgeColor',b);
           end
      elseif i>1950 && i<2052| i>4950 && i<5052
        if i>1950 && i<2052
        resonancia23(j) = text(-0.25,0,0.55*lvec(1),'Resonancia 2/3','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
        end
      if j==1|| j==4 || j==7
           hMass2(j) = plot3(xMass,-0.1*j,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',mColor,'MarkerEdgeColor',mColor);
           elseif j==2|| j==5 || j==8
           hMass2(j) = plot3(xMass,-0.1*j,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',b,'MarkerEdgeColor',b);
           else
           hMass2(j) = plot3(xMass,-0.1*j,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',a,'MarkerEdgeColor',a);
           end
      elseif i>2200 && i<2302| i>5200 && i<5302
        if i>2200 && i<2302
        resonancia34(j) = text(-0.25,0,0.55*lvec(1),'Resonancia 3/4','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
        end
        if j==1|| j==5
           hMass2(j) = plot3(xMass,-0.1*j,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',b,'MarkerEdgeColor',b);
           elseif j==2|| j==6
           hMass2(j) = plot3(xMass,-0.1*j,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',v,'MarkerEdgeColor',v);
           elseif j==3|| j==7
           hMass2(j) = plot3(xMass,-0.1*j,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',a,'MarkerEdgeColor',a);
           else
           hMass2(j) = plot3(xMass,-0.1*j,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',mColor,'MarkerEdgeColor',mColor);
           end
   else
      end
   end
   
   hEnergy = text(lvec(1),0,-0.3*lvec(1),['\bf Energy: \rm',num2str(E(i),10),' J'],'interpreter','LaTeX','FontSize',fs,'Color',fgColor);   
   hTime = text(lvec(1),0,-0.4*lvec(1),['\bf Time: \rm', num2str(t(i),3),' s'],'interpreter','LaTeX','FontSize',fs,'Color',fgColor);
    if i>nTimes/2 && i<nTimes/2+151 %Una vez transcurrido un periodo completo, volvemos a variar la perspectiva y, junto a ella, los ejes para conseguir una visión más compacta del sistema.
view([6/50*(i-nTimes/2) 15/150*(i-nTimes/2)])
axis(0.7*lvec(1)*[-1 1.5*AR-(90^(-1))*(i-nTimes/2) -1.6*(0.7*lvec(1))^(-1)-0.7+(900^(-1))*(i-nTimes/2) 1.5*(0.7*lvec(1))^(-1) -0.9 0.9])
delete([hEnergy,hTime,h1,h2,h3,h4,h5,h6,h7]);
    end
if i==nTimes/2+150
    axis(0.7*lvec(1)*[-1 1.5*AR-(90^(-1))*(150) -1.6*(0.7*lvec(1))^(-1)-0.7+(900^(-1))*(150) 1.5*(0.7*lvec(1))^(-1) -0.9 0.9])
    delete([hEnergy,hTime,h1,h2,h3,h4,h5,h6,h7,resonancia14,resonancia13,resonancia12,resonancia23,resonancia34]);
end
if i>nTimes/2+150 %Ahora dejamos de levantar el punto de vista pero seguimos rotando el sistema para que realice una vuelta en 60 segundos. 
 view([6/50*(i-nTimes/2) 15])
 delete([hEnergy,hTime,h1,h2,h3,h4,h5,h6,h7,resonancia14,resonancia13,resonancia12,resonancia23,resonancia34]);
end
if i>nTimes-150 %Regresamos a la visión 2D inicial.
 view([6/50*(i-nTimes/2) 15-15/150*(i-nTimes+150)])
 delete([hEnergy,hTime,h1,h2,h3,h4,h5,h6,h7,resonancia14,resonancia13,resonancia12,resonancia23,resonancia34]);
end
   frame = getframe(figh,[e1 e2 p1 p2]);
   writeVideo(vwo,frame);
end

% %7.- Loop para obtener un segundo de un 'frame' estático con la figura ya dibujada
% nFrames = FR*1;  %Número de 'frames' en 5 segundos
% for i=1:nFrames
%    frame = getframe;
%    writeVideo(vwo,frame);
% end
close(vwo);
