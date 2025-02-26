clear


%% 1.- Parámetros, ecuaciones, condiciones iniciales & rango temporal.
m = 1;                              %Masa [en kg].
g = 9.8;                            %Aceleración de la gravedad [en m/s^2] .
theta0 = 0.174;                     %Ángulo inicial [en rad].
omega0 = 0;                         %Velocidad angular inicial [en rad/s].
tf = 60;                            %El rango temporal es [0,tf].
FR=50;                              %Frames por segundo.

lvec = [];  %Definimos tres vectores vacíos que albergarán
cvec = [];  %los valores relevantes del problema.
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
%NOTA: La obtención de las relaciones matemáticas utilizadas en el cálculo de las 
%respectivas longitudes de cada péndulo y sus energías se especifica en el 
%trabajo escrito.


%% 2.- Características del plot (ventana, colores, tamaño de las fuentes y los puntos, grosor de la líneas).
figure('WindowState','Maximized')
axis equal
axis manual
AR = 16/9;                  %'Screen aspect ratio'
%Se reserva un espacio a la derecha de la ventana con el objetivo de
%insertar texto más adelante.
%El objetivo de la siguiente línea es encajar las componentes que
%aparecerán en el vídeo de una forma organizada y estética.
axis(0.7*lvec(1)*[-1 1.5*AR -0.9 0.9])
grid off
box off
bgColor = [0 0 0]; %Color del backgroud (en el modelo de colores RGB).
fgColor = [1 1 1];          %Color del Foreground (blanco en RGB).
mColor = [0.635 0.0780 0.184];        %Color de la masa (granate).
sColor = [0.75 1 1];        %Color del soporte.
pColor = [1 1 0.75];        %Color del péndulo
ma= 17;                     %tamaño de la bola de soporte.
ms = 11;                    %Tamaño de los puntos.
fs = 15;                    %Tamaño de la fuente.
lw = 2;                     %Grosor de las líneas.
set(gca,'Color',bgColor)
set(gca,'xtick',[])
set(gca,'ytick',[])
set(gca,'xticklabel',[])
set(gca,'yticklabel',[])
hold on


%% 3.- Características del vídeo
nameFile = 'SimplePendulum';                  %Nombre del fichero de vídeo.
vwo = VideoWriter(nameFile,'MPEG-4');          %Usamos el formato MPEG-4.
FR = 50;                                               %Frames por segundo.
prescribedTimes = 0:1/FR:tf;  %La solución solo se computará en estos instantes de tiempo.
vwo.Quality = 95;             %Calidad del vídeo (Valor predeterminado: 75).
vwo.FrameRate = FR;          %Frames por segundo (Valor predeterminado: 30).
open(vwo);


%% 4.- Computación numérica del movimiento del péndulo con el comando ode45

%NOTA: Solo se computa la posición del péndulo en los instantes de tiempo
%ya definidos en la variable prescribedTimes.

%El siguiente bucle permite definir las ecuaciones diferenciales que
%seguirán cada uno de entre los ocho péndulos que hemos definido. Las funciones y los vectores
%obtenidos tras la resolución de las respectivas EDOs se almacenan en una
%celda para llamarlos más adelante.
for i=1:8
F =  @(t,X) [X(2);-cvec(i)*sin(X(1))];
cell{i}=F;
opciones = odeset('AbsTol',1e-8,'RelTol',1e-8);       %Control del error y 
%resolución de las edos.
[t,X] = ode45(cell{i},prescribedTimes,[theta0;omega0],opciones);
cellfun{i} = X;
E = m*lvec(1)^2*X(:,2).^2/2 + m*g*lvec(1)*(1-cos(X(:,1))); 
%Energía mecánica del péndulo de mayor longitud.
end

%% 5.- Frame inicial.

%Creamos un bucle para dibujar cada péndulo en el instante t=0 y adaptamos
%las coordenadas.
for i=1:8
xMass = lvec(i)*sin(theta0); %Posición x de la masa.
yMass = -lvec(i)*cos(theta0); %Posición y de la masa.
hCord(i) = plot([0,xMass],[0.45*lvec(1),0.45*lvec(1)+yMass],'-','Color',pColor,'LineWidth',lw); %Cuerda que ¨sostiene¨ la masa.
hSupport(i) = plot(0,0.45*lvec(1),'o','MarkerSize',ma,'MarkerFaceColor',sColor,'MarkerEdgeColor',sColor); %Punto de soporte.
hMass(i) = plot(xMass,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',mColor,'MarkerEdgeColor',mColor); %Masa.
end

%A continuación añadimos el texto que aparecerá en el vídeo. Los parámetros
%que varían con el tiempo tendrán que ser actualizados para
%el resto de frames.
%IMPORTANTE: Usamos el interpretador de texto LaTeX para los símbolos matemáticos. 
text(lvec(1),0.4*lvec(1),'Physical parameters:','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
text(lvec(1),0.3*lvec(1),['$m = ',num2str(m,3),'$'],'interpreter','LaTeX','FontSize',fs,'Color',fgColor);
text(lvec(1),0.2*lvec(1),['$g = ',num2str(g,3),'$'],'interpreter','LaTeX','FontSize',fs,'Color',fgColor);
text(lvec(1),0.1*lvec(1),['$l_0 = ',num2str(lvec(1),3),'$'],'interpreter','LaTeX','FontSize',fs,'Color',fgColor);
text(lvec(1),0.0*lvec(1),'Initial condition:','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
text(lvec(1),-0.1*lvec(1),['$\theta_0 = ',num2str(theta0,6),'$'],'interpreter','LaTeX','FontSize',fs,'Color',fgColor);
text(lvec(1),-0.2*lvec(1),['$\omega_0 = ',num2str(omega0,6),'$'],'interpreter','LaTeX','FontSize',fs,'Color',fgColor);
hEnergy = text(lvec(1),-0.3*lvec(1),['Energy: $',num2str(Evec(1),10),'$'],'interpreter','LaTeX','FontSize',fs,'Color',fgColor);
hTime = text(lvec(1),-0.4*lvec(1),'Time: $0$','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
frame = getframe;
writeVideo(vwo,frame);


%% 6.- Loop para el resto de frames del vídeo

%Definimos todo lo que el bucle necesitará borrar.
resonancia14=text(0,0.6*lvec(1),'','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
resonancia13=text(0,0.6*lvec(1),'','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
resonancia12=text(0,0.6*lvec(1),'','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
resonancia23=text(0,0.6*lvec(1),'','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
resonancia34=text(0,0.6*lvec(1),'','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
hMass2 = plot(xMass,0.45*lvec(1)+yMass,'','MarkerSize',ms,'MarkerFaceColor',mColor,'MarkerEdgeColor',mColor);

%Colores usados para diferenciar los estados de resonancia del sistema
v = [0.75 0.75 0]; %Verde.
b = [1 1 1]; %Blanco.
mColor = [0.635 0.0780 0.184]; %Granate.
a = [0.667 0.667 1]; %Azul cielo.

%La siguiente cadena de bucles anidados es en esencia la parte del código
%donde se le pide a Matlab que emplee los resultados de las edos ya 
%resueltas de cada péndulo y que, frame a frame, construya el vídeo. 
%Los dos bucles 'for' son los encargados de hacer el 'plot' de cada péndulo
%j-ésimo en función de los valores almacenados en la celda que contiene los 
%datos de sus respectivas edos, mientras que el índice 'i' recorre un
%vector del tamaño del vector 't'. Es decir hace que matlab use todos los
%valores almacenados en el vector de posiciones 'X'. Al combinar ambos
%bucles 'for' se consigue que no solo se dibuje un péndulo, en cuyo caso solo
%haría falta resolver una edo, sino que se dibujen los ocho a la vez, cada
%uno siguiendo su respectiva ecuación diferencial.
%NOTA: en este caso 'length(t)=3001', lo que tiene sentido, pues se trata
%de un vídeo de 60 segundos de duración a 50 'frames' por segundo.
%En cuanto a la cadena de bucles 'if' anidados, esta tiene propósitos 
%puramente estéticos. Para diferentes valores de 'i', es decir para ciertos
%instantes de tiempo, se le pide al ordenador que imprima un texto 
%anunciando los respectivos estados de resonancia del sistema, con tal
%de que quien lo visualiza no pase por alto dicho fenómeno. De añadido,
%se imponen ciertas condiciones más para cada estado de resonancia con tal
%de que el color de las masas de los péndulos j-ésimos pertinentes cambie, 
%ayudando así a visualizar los estados de resonancia.  
nTimes = length(t);
for i=1:nTimes
   delete([hCord,hMass,hMass2,hSupport,hEnergy,hTime,resonancia14,resonancia13,resonancia12,resonancia23,resonancia34]);
    for j=1:8
    xMass = lvec(j)*sin(cellfun{j}(i,1));
    yMass = -lvec(j)*cos(cellfun{j}(i,1));
    hCord(j) = plot([0,xMass],[0.45*lvec(1),0.45*lvec(1)+yMass],'-','Color',pColor,'LineWidth',lw);
    hSupport(j) = plot(0,0.45*lvec(1),'o','MarkerSize',ma,'MarkerFaceColor',sColor,'MarkerEdgeColor',sColor);
    hMass(j) = plot(xMass,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',mColor,'MarkerEdgeColor',mColor);
       
    if i>725 && i<777
       resonancia14(j) = text(-0.35,0.55*lvec(1),'Resonancia 1/4','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
            if j==1|| j==5
            hMass2(j) = plot(xMass,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',b,'MarkerEdgeColor',b);
            elseif j==2|| j==6
            hMass2(j) = plot(xMass,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',v,'MarkerEdgeColor',v);
            elseif j==3|| j==7
            hMass2(j) = plot(xMass,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',a,'MarkerEdgeColor',a);
            else 
            hMass2(j) = plot(xMass,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',mColor,'MarkerEdgeColor',mColor);
            end

       elseif i>975 && i<1027
       resonancia13(j)=text(-0.35,0.55*lvec(1),'Resonancia 1/3','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
            if j==1|| j==4 || j==7
            hMass2(j) = plot(xMass,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',mColor,'MarkerEdgeColor',mColor);
            elseif j==2|| j==5 || j==8
            hMass2(j) = plot(xMass,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',b,'MarkerEdgeColor',b);
            else 
            hMass2(j) = plot(xMass,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',a,'MarkerEdgeColor',a);
            end

       elseif i>1475 && i<1527
       resonancia12(j)=text(-0.35,0.55*lvec(1),'Resonancia 1/2','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
            if j==1|| j==3 || j==5 || j==7
            hMass2(j) = plot(xMass,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',mColor,'MarkerEdgeColor',mColor);
            else 
            hMass2(j) = plot(xMass,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',b,'MarkerEdgeColor',b);
            end

       elseif i>1975 && i<2027
       resonancia23(j)=text(-0.35,0.55*lvec(1),'Resonancia 2/3','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
       if j==1|| j==4 || j==7
            hMass2(j) = plot(xMass,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',mColor,'MarkerEdgeColor',mColor);
            elseif j==2|| j==5 || j==8
            hMass2(j) = plot(xMass,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',b,'MarkerEdgeColor',b);
            else 
            hMass2(j) = plot(xMass,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',a,'MarkerEdgeColor',a);
            end

       elseif i>2225 && i<2277
       resonancia34(j)=text(-0.35,0.55*lvec(1),'Resonancia 3/4','interpreter','LaTeX','FontSize',fs,'Color',fgColor);
         if j==1|| j==5
            hMass2(j) = plot(xMass,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',b,'MarkerEdgeColor',b);
            elseif j==2|| j==6
            hMass2(j) = plot(xMass,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',v,'MarkerEdgeColor',v);
            elseif j==3|| j==7
            hMass2(j) = plot(xMass,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',a,'MarkerEdgeColor',a);
            else 
            hMass2(j) = plot(xMass,0.45*lvec(1)+yMass,'o','MarkerSize',ms,'MarkerFaceColor',mColor,'MarkerEdgeColor',mColor);
            end

    else 

       end
    end
    hEnergy = text(lvec(1),-0.3*lvec(1),['Energy: ',num2str(E(1),10)],'interpreter','LaTeX','FontSize',fs,'Color',fgColor);    
    hTime = text(lvec(1),-0.4*lvec(1),['Time: ',num2str(t(i),3)],'interpreter','LaTeX','FontSize',fs,'Color',fgColor); 


    frame = getframe;
    writeVideo(vwo,frame);
    
end

%7.- Loop para obtener un segundo de un 'frame' estático con la figura ya dibujada 
% nFrames = FR*1;  %Número de 'frames' en 5 segundos
% for i=1:nFrames
%     frame = getframe;
%     writeVideo(vwo,frame);
% end
% 
close(vwo);