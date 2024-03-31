%delete(instrfind({'port'},{'COM3'}));
%dataPort = serialport("COM3", 9600);
% Inicialización de variables
nS =192;               %Numero de muestras
level = unnamed1;
%levelnoFilter = unnamed2;
t = 1:nS; 
%Ajustar los limite de la figura
figure
xMin = 0;
xMax = nS;
yMin = 0;
yMax = 101;

% Definir los puntos para la línea
x1 = 32; 
y1 = 12;
x2 = 93;
y2 = 93;

for i=1:nS
    try
        %info = readline(dataPort);
        %level(i) = str2double(info);
        % Actualizar gráfico
        plot(t, level,'g','LineWidth',1.5);
        xlim([xMin xMax]);
        ylim([yMin yMax]);
        title('Level Control System', 'FontName', 'Arial', 'FontSize', 12);
        xlabel('Sample', 'FontName', 'Arial', 'FontSize', 12);
        ylabel('Value', 'FontName', 'Arial', 'FontSize', 12); 
        grid on;
        % Dibuja la línea horizontal en y=93
        hold on; % Para superponer la línea en el mismo gráfico
        line([xMin, xMax], [93, 93], 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
        % Dibuja la tangente
        line([x1, x2], [y1, y2], 'Color', 'b', 'LineStyle', '--','LineWidth', 1.5);
        line([93, 93], [93, 12], 'Color', 'y', 'LineWidth', 1.5);
        hold off;
        
        drawnow; % Actualiza la ventana de la figura
        pause(0.01);
    catch
        disp(['Error reading data: ' exception.message]);
    end
end
