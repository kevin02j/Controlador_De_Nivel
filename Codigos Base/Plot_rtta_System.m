% Configuración de la comunicación serial con Arduino
s = serialport("COM6", 9600);
configureTerminator(s, "LF");

% Inicialización de variables
nS =5000;           %Numero de muestras
vSetG = zeros(1, nS);
vT = zeros(1, nS);
t = 1:nS; 

%Ajustar los limite de la figura
figure
xMin = 0;
xMax = nS;
yMin = 100;
yMax = 0;

title('Level Control System', 'FontName', 'Arial', 'FontSize', 12);
xlabel('Sample', 'FontName', 'Arial', 'FontSize', 12);
ylabel('Value', 'FontName', 'Arial', 'FontSize', 12); 
grid on;

% Bucle principal para capturar datos y actualizar el gráfico
for i = 1:nS
    try
        datos = readline(s);             %Catura los datos del puerto
        newStr = split(datos, ';');
        SetG = str2double(newStr(1));
        tValue = str2double(newStr(2));

        % Almacenar datos para graficar
        vSetG(i) = SetG;
        vT(i) = tValue;

        % Actualizar gráfico
        plot(t, vSetG, 'b', t, vT, 'r', 'LineWidth', 1.5);
        legend('Distance', 'distanceFilter', 'Location', 'NorthEast');
        xlim([xMin xMax]);
        ylim([yMin yMax]);
        drawnow; % Actualiza la ventana de la figura
    catch exception
        disp(['Error reading data: ' exception.message]);
    end
end

disp('Ends process...');