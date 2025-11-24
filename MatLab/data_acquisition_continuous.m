% @author  Dominik Luczak
% @date    2021-11-24, 2022-01-07

clear all;

time_window_vibrations = 1040; %ms
time_window_currents = 520; %ms
data_windows_to_save = 24; % 1 data window = 80 ms

read_JSON = 0;

%% Serial port setup
seriallist
port_name = seriallist;
port_name = port_name{end};
port_name = 'COM13';
hSerial=[];%close
hSerial = serialport(port_name, 2250000, 'Timeout', 5, 'Parity', 'none');
configureTerminator(hSerial,'CR/LF');
pause(0.5);

%% Write serial settings
pwm = 40;
write(hSerial, sprintf('pwm=%03d;',pwm), 'char' );
pause(1.5);

%% Enable/disable save to file
save_enable = 0;
is_file_open = 0;
allow_save_start = 1;
class_type = 'fault2';


%% Prepare plots for constant update

vibrations_freq = 1; %kHz
currents_freq = 20; %kHz

vibrations_axis_samples = 80;
currents_axis_samples = 1600;

vibrations_time_vector = 0:1/vibrations_freq:time_window_vibrations-1/vibrations_freq;
currents_time_vector = 0:1/currents_freq:time_window_currents-1/currents_freq;


accelerometer_x_vector = zeros(1,length(vibrations_time_vector));
accelerometer_y_vector = zeros(1,length(vibrations_time_vector));
accelerometer_z_vector = zeros(1,length(vibrations_time_vector));    
gyroscope_x_vector = zeros(1,length(vibrations_time_vector));
gyroscope_y_vector = zeros(1,length(vibrations_time_vector));
gyroscope_z_vector = zeros(1,length(vibrations_time_vector));  
current_u_vector = zeros(1,length(currents_time_vector));
current_v_vector = zeros(1,length(currents_time_vector));
current_w_vector = zeros(1,length(currents_time_vector));


figure(1);
subplot(3,1,1);
hACCx = plot(vibrations_time_vector,accelerometer_x_vector, 'r.-');
hold on;
hACCy = plot(vibrations_time_vector,accelerometer_y_vector, 'g.-');
hACCz = plot(vibrations_time_vector,accelerometer_z_vector, 'b.-');
hold off;
legend('x','y','z');
title("Data logger, STM32 (LSM6DSOX/MPU6050,UART,JSON). (author: D. Łuczak)")
xlabel('Time (ms)');
ylabel('accelerometer value (-)');
set(gcf,'color','w'); 
%ylim([-5000, 20000]);
%ylim([-5 5])

subplot(3,1,2);
hGYx = plot(vibrations_time_vector,gyroscope_x_vector, 'r.-');
hold on;
hGYy = plot(vibrations_time_vector,gyroscope_y_vector, 'g.-');
hGYz = plot(vibrations_time_vector,gyroscope_z_vector, 'b.-');
hold off;
legend('x','y','z');
title("Data logger, STM32 (LSM6DSOX/MPU6050,UART,JSON). (author: D. Łuczak)")
xlabel('Time (ms)');
ylabel('gyroscope value (-)');
set(gcf,'color','w'); 
%ylim([-5000, 20000]);
%ylim([-5 10])

subplot(3,1,3);
hCUu = plot(currents_time_vector,current_u_vector, 'r.-');
hold on;
hCUv = plot(currents_time_vector,current_v_vector, 'g.-');
hCUw = plot(currents_time_vector,current_w_vector, 'b.-');
%ylim([1500 2000]);
hold off;
legend('u','v','w');
title("Data logger, STM32 (LSM6DSOX/MPU6050,UART,JSON). (author: D. Łuczak)")
xlabel('Time (ms)');
ylabel('current value (-)');
set(gcf,'color','w'); 

hACCx.YDataSource = 'accelerometer_x_vector';
hACCy.YDataSource = 'accelerometer_y_vector';
hACCz.YDataSource = 'accelerometer_z_vector';
hGYx.YDataSource = 'gyroscope_x_vector';
hGYy.YDataSource = 'gyroscope_y_vector';
hGYz.YDataSource = 'gyroscope_z_vector';
hCUu.YDataSource = 'current_u_vector';
hCUv.YDataSource = 'current_v_vector';
hCUw.YDataSource = 'current_w_vector';

%% setup variable
set_only_once_sdft=1;

n_accelerometer_x_vector = zeros(1,vibrations_axis_samples);
n_accelerometer_y_vector = zeros(1,vibrations_axis_samples);
n_accelerometer_z_vector = zeros(1,vibrations_axis_samples);
n_gyroscope_x_vector = zeros(1,vibrations_axis_samples);
n_gyroscope_y_vector = zeros(1,vibrations_axis_samples);
n_gyroscope_z_vector = zeros(1,vibrations_axis_samples);
n_current_u_vector = zeros(1,currents_axis_samples);
n_current_v_vector = zeros(1,currents_axis_samples);
n_current_w_vector = zeros(1,currents_axis_samples);

loop_index=1;
correct_byte_num = 0;
dataIncoming = 0;
%% Main infinity loop
while 1
    received_bytes = [];
    loop_index=loop_index+1;
    %disp(loop_index);

    if (read_JSON == 1)
        dataIncoming = -1;
        JSON_text = readline(hSerial);
        try
            JSON_structure = jsondecode(JSON_text);
            disp("JSON decoded")
            saved_JSON_text = JSON_text;
        catch
            %disp("could not decode JSON")
        end
        continue
    end

    if (dataIncoming == 0)
        test_byte = read(hSerial, 1, "uint8");
        
        if (correct_byte_num == 0 & test_byte == 1)
            correct_byte_num = 1;
        elseif (correct_byte_num == 1 & test_byte == 100)
            correct_byte_num = 2;
        elseif (correct_byte_num == 2 & test_byte == 200)
            correct_byte_num = 3;
        elseif (correct_byte_num == 3 & test_byte == 1)
            dataIncoming = 1;
            correct_byte_num = 0;
        else
            dataIncoming = 0;
            correct_byte_num = 0;
        end
    end
    
    if (dataIncoming == 1)
        text = read(hSerial, 5282,"int16");
        dataIncoming = 0;

        %try

            end_val1 = text(5281); %7880
            end_val2 = text(5282); %-14321
      
            if ~(end_val1 == 7880 & end_val2 == -14321)
                disp("incorrect uart data")
            end

    


            % for i=1:vibrations_axis_samples
            % 
            %     % n_accelerometer_x_vector(i) = (text(6*i - 5))/16384.0;
            %     % n_accelerometer_y_vector(i) = (text(6*i - 4))/16384.0;
            %     % n_accelerometer_z_vector(i) = (text(6*i -3))/14418.0;
            %     % n_gyroscope_x_vector(i) = (text(6*i - 2))/131.0;
            %     % n_gyroscope_y_vector(i) = (text(6*i - 1))/131.0;
            %     % n_gyroscope_z_vector(i) = (text(6*i))/131.0;
            % 
            %     n_accelerometer_x_vector(i) = (text(6*i - 5));
            %     n_accelerometer_y_vector(i) = (text(6*i - 4));
            %     n_accelerometer_z_vector(i) = (text(6*i -3));
            %     n_gyroscope_x_vector(i) = (text(6*i - 2));
            %     n_gyroscope_y_vector(i) = (text(6*i - 1));
            %     n_gyroscope_z_vector(i) = (text(6*i));
            % 
            % end

            n_accelerometer_x_vector = text(1:80);
            n_accelerometer_y_vector = text(81:160);
            n_accelerometer_z_vector = text(161:240);
            n_gyroscope_x_vector = text(241:320);
            n_gyroscope_y_vector = text(321:400);
            n_gyroscope_z_vector = text(401:480);
            
            % for i=1:currents_axis_samples
            %     n_current_u_vector(i) = text(480+3*i-2);
            %     n_current_v_vector(i) = text(480+3*i-1);
            %     n_current_w_vector(i) = text(480+3*i);
            % end

            n_current_u_vector = text(481:2080);
            n_current_v_vector = text(2081:3680);
            n_current_w_vector = text(3681:5280);
    
            accelerometer_x_vector = circshift(accelerometer_x_vector, -vibrations_axis_samples);
            accelerometer_y_vector = circshift(accelerometer_y_vector, -vibrations_axis_samples);
            accelerometer_z_vector = circshift(accelerometer_z_vector, -vibrations_axis_samples);
            gyroscope_x_vector = circshift(gyroscope_x_vector, -vibrations_axis_samples);
            gyroscope_y_vector = circshift(gyroscope_y_vector, -vibrations_axis_samples);
            gyroscope_z_vector = circshift(gyroscope_z_vector, -vibrations_axis_samples);
            current_u_vector = circshift(current_u_vector, -currents_axis_samples);
            current_v_vector = circshift(current_v_vector, -currents_axis_samples);
            current_w_vector = circshift(current_w_vector, -currents_axis_samples);
    
            accelerometer_x_vector(end-vibrations_axis_samples+1:end) = n_accelerometer_x_vector;
            accelerometer_y_vector(end-vibrations_axis_samples+1:end) = n_accelerometer_y_vector;
            accelerometer_z_vector(end-vibrations_axis_samples+1:end) = n_accelerometer_z_vector;
            gyroscope_x_vector(end-vibrations_axis_samples+1:end) = n_gyroscope_x_vector;
            gyroscope_y_vector(end-vibrations_axis_samples+1:end) = n_gyroscope_y_vector;
            gyroscope_z_vector(end-vibrations_axis_samples+1:end) = n_gyroscope_z_vector;
            current_u_vector(end-currents_axis_samples+1:end) = n_current_u_vector;
            current_v_vector(end-currents_axis_samples+1:end) = n_current_v_vector;
            current_w_vector(end-currents_axis_samples+1:end) = n_current_w_vector;
    
        

        if save_enable == 0
            save_file_created = 0;
            if is_file_open == 1
                fclose(hFile);
                is_file_open = 0;
                disp("saving stopped")
            end
        end

        if save_enable == 1
            
            allow_save_start = 0;

            if is_file_open == 0
                hFile = fopen(sprintf('saved_data_%s.txt', datestr(now,'yyyy_mm_dd-HH_MM_SS_FFF')), 'a');
                is_file_open = 1;
                fprintf(hFile,'curr_u,curr_v,curr_w,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z\r\n');
                disp("saving started")
                window_num = 0;
            end

           
            for k=1:length(n_current_u_vector)
                if mod(k-1, 20) == 0
                    fprintf(hFile, '%d,%d,%d,',n_current_u_vector(k),n_current_v_vector(k),n_current_w_vector(k));
                    fprintf(hFile,'%d,%d,%d,',n_accelerometer_x_vector(floor(0.95+k/20)),n_accelerometer_y_vector(floor(0.95+k/20)),n_accelerometer_z_vector(floor(0.95+k/20)));
                    fprintf(hFile,'%d,%d,%d\r\n',n_gyroscope_x_vector(floor(0.95+k/20)),n_gyroscope_y_vector(floor(0.95+k/20)),n_gyroscope_z_vector(floor(0.95+k/20)));
                else
                    fprintf(hFile, '%d,%d,%d\r\n',n_current_u_vector(k),n_current_v_vector(k),n_current_w_vector(k));
                end
            end

            window_num = window_num + 1;
            if (window_num >= data_windows_to_save)
                save_enable = 0;
                
            end

        end
        % Plot results
        refreshdata(hACCx);
        refreshdata(hACCy); 
        refreshdata(hACCz);
        refreshdata(hGYx);
        refreshdata(hGYy); 
        refreshdata(hGYz);    
        refreshdata(hCUu);
        refreshdata(hCUv);
        refreshdata(hCUw);
    end
    
        if allow_save_start == 1 && strcmp(get(gcf,'currentcharacter'),'s')
            save_enable = 1;
        end
        if strcmp(get(gcf,'currentcharacter'),'d')
            if allow_save_start == 0
                disp("start allowed")
            end
            save_enable = 0;
            allow_save_start = 1;
            
        end

    % Quit loop
    if strcmp(get(gcf,'currentcharacter'),'q')
      close(gcf);
      break
    end    
end

hSerial=[];%close
if save_enable == 1
    fclose(hFile);
end

