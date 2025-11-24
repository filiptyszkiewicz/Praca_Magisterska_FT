%  'z' -> 180° (MAX)
%  's' -> 180°do 30° 
%  'b' -> 0°
%  'd' -> stop zapisu
%  'q' -> wyjście.

clear; clc;

%% ========= settings =========
OUTPUT_FOLDER = 'C:\dane';
if ~exist(OUTPUT_FOLDER,'dir'), mkdir(OUTPUT_FOLDER); end

time_window_vibrations = 1040; 
time_window_currents   = 520;  

read_JSON = 0; 

ESP_ENABLED      = true;
ESP_BASE         = 'http://192.168.4.1';
SERVO_MIN_DEG    = 0;    % min
SERVO_MAX_DEG    = 180;  % max
SERVO_END_DEG    = 30;   % end deg
TOTAL_T_S        = 60;   % time
HTTP_MIN_DT      = 0.10;
lastHttpTic      = tic;

servo_deg_current = 0;
profile_active    = false;
profile_t0        = 0;
profile_last_deg  = NaN;

save_enable      = 0;
is_file_open     = 0;
allow_save_start = 1;
hFile            = [];
log_t0           = 0;

titleBox = annotation('textbox',[0.63 0.90 0.36 0.09], ...
    'String','', 'EdgeColor',[0.85 0.85 0.85], 'BackgroundColor',[1 1 1], ...
    'HorizontalAlignment','left', 'FontSize',10);
updateTitle = @() set(titleBox,'String', sprintf( ...
    'Servo: %3d° | Profil: %s | Log: %s', ...
    round(servo_deg_current), ...
    ternary(profile_active,'ON','OFF'), ...
    ternary(save_enable,'ON','OFF')));

seriallist;
port_name = 'COM13';
hSerial=[];
hSerial = serialport(port_name, 2250000, 'Timeout', 5, 'Parity', 'none');
configureTerminator(hSerial,'CR/LF');
pause(0.5);

pwm = 40;
write(hSerial, sprintf('pwm=%03d;',pwm), 'char' );
pause(1.5);

if ESP_ENABLED
    try
        esp_servo('set', SERVO_MIN_DEG, ESP_BASE);
        servo_deg_current = SERVO_MIN_DEG;
        lastHttpTic = tic;
        fprintf("SERVO start -> %d°\n", SERVO_MIN_DEG);
    catch
        warning('ESP: start error 0 deg.');
    end
end
updateTitle();

vibrations_freq = 1;   % kHz
currents_freq   = 20;  % kHz
vibrations_axis_samples = 80;
currents_axis_samples   = 1600;

vibrations_time_vector = 0:1/vibrations_freq:time_window_vibrations-1/vibrations_freq;
currents_time_vector   = 0:1/currents_freq:time_window_currents-1/currents_freq;

accelerometer_x_vector = zeros(1,length(vibrations_time_vector));
accelerometer_y_vector = zeros(1,length(vibrations_time_vector));
accelerometer_z_vector = zeros(1,length(vibrations_time_vector));
gyroscope_x_vector     = zeros(1,length(vibrations_time_vector));
gyroscope_y_vector     = zeros(1,length(vibrations_time_vector));
gyroscope_z_vector     = zeros(1,length(vibrations_time_vector));
current_u_vector       = zeros(1,length(currents_time_vector));
current_v_vector       = zeros(1,length(currents_time_vector));
current_w_vector       = zeros(1,length(currents_time_vector));

figure(1);
subplot(3,1,1);
hACCx = plot(vibrations_time_vector,accelerometer_x_vector, 'r.-'); hold on;
hACCy = plot(vibrations_time_vector,accelerometer_y_vector, 'g.-');
hACCz = plot(vibrations_time_vector,accelerometer_z_vector, 'b.-'); hold off;
legend('x','y','z'); title("Data logger (author: D. Łuczak)")
xlabel('Time (ms)'); ylabel('accelerometer (-)'); set(gcf,'color','w');

subplot(3,1,2);
hGYx = plot(vibrations_time_vector,gyroscope_x_vector, 'r.-'); hold on;
hGYy = plot(vibrations_time_vector,gyroscope_y_vector, 'g.-');
hGYz = plot(vibrations_time_vector,gyroscope_z_vector, 'b.-'); hold off;
legend('x','y','z'); title("Data logger (author: D. Łuczak)")
xlabel('Time (ms)'); ylabel('gyroscope (-)'); set(gcf,'color','w');

subplot(3,1,3);
hCUu = plot(currents_time_vector,current_u_vector, 'r.-'); hold on;
hCUv = plot(currents_time_vector,current_v_vector, 'g.-');
hCUw = plot(currents_time_vector,current_w_vector, 'b.-'); hold off;
legend('u','v','w'); title("Data logger (author: D. Łuczak)")
xlabel('Time (ms)'); ylabel('current (-)'); set(gcf,'color','w');

hACCx.YDataSource = 'accelerometer_x_vector';
hACCy.YDataSource = 'accelerometer_y_vector';
hACCz.YDataSource = 'accelerometer_z_vector';
hGYx.YDataSource  = 'gyroscope_x_vector';
hGYy.YDataSource  = 'gyroscope_y_vector';
hGYz.YDataSource  = 'gyroscope_z_vector';
hCUu.YDataSource  = 'current_u_vector';
hCUv.YDataSource  = 'current_v_vector';
hCUw.YDataSource  = 'current_w_vector';

% bufory pojedynczego okna z UART
n_accelerometer_x_vector = zeros(1,vibrations_axis_samples);
n_accelerometer_y_vector = zeros(1,vibrations_axis_samples);
n_accelerometer_z_vector = zeros(1,vibrations_axis_samples);
n_gyroscope_x_vector     = zeros(1,vibrations_axis_samples);
n_gyroscope_y_vector     = zeros(1,vibrations_axis_samples);
n_gyroscope_z_vector     = zeros(1,vibrations_axis_samples);
n_current_u_vector       = zeros(1,currents_axis_samples);
n_current_v_vector       = zeros(1,currents_axis_samples);
n_current_w_vector       = zeros(1,currents_axis_samples);

loop_index     = 1;
correct_byte_num = 0;
dataIncoming   = 0;

while true
    loop_index = loop_index + 1;
    if (dataIncoming == 0)
        test_byte = read(hSerial, 1, "uint8");
        if     (correct_byte_num == 0 && test_byte == 1),   correct_byte_num = 1;
        elseif (correct_byte_num == 1 && test_byte == 100), correct_byte_num = 2;
        elseif (correct_byte_num == 2 && test_byte == 200), correct_byte_num = 3;
        elseif (correct_byte_num == 3 && test_byte == 1),   dataIncoming = 1; correct_byte_num = 0;
        else, dataIncoming = 0; correct_byte_num = 0;
        end
    end

    if (dataIncoming == 1)
        T = read(hSerial, 5282,"int16");
        dataIncoming = 0;

        if ~(T(5281) == 7880 && T(5282) == -14321)
            disp("incorrect uart data");
        end

        n_accelerometer_x_vector = T(1:80);
        n_accelerometer_y_vector = T(81:160);
        n_accelerometer_z_vector = T(161:240);
        n_gyroscope_x_vector     = T(241:320);
        n_gyroscope_y_vector     = T(321:400);
        n_gyroscope_z_vector     = T(401:480);

        n_current_u_vector = T(481:2080);
        n_current_v_vector = T(2081:3680);
        n_current_w_vector = T(3681:5280);

        accelerometer_x_vector = circshift(accelerometer_x_vector, -vibrations_axis_samples);
        accelerometer_y_vector = circshift(accelerometer_y_vector, -vibrations_axis_samples);
        accelerometer_z_vector = circshift(accelerometer_z_vector, -vibrations_axis_samples);
        gyroscope_x_vector     = circshift(gyroscope_x_vector, -vibrations_axis_samples);
        gyroscope_y_vector     = circshift(gyroscope_y_vector, -vibrations_axis_samples);
        gyroscope_z_vector     = circshift(gyroscope_z_vector, -vibrations_axis_samples);
        current_u_vector       = circshift(current_u_vector, -currents_axis_samples);
        current_v_vector       = circshift(current_v_vector, -currents_axis_samples);
        current_w_vector       = circshift(current_w_vector, -currents_axis_samples);

        accelerometer_x_vector(end-79:end) = n_accelerometer_x_vector;
        accelerometer_y_vector(end-79:end) = n_accelerometer_y_vector;
        accelerometer_z_vector(end-79:end) = n_accelerometer_z_vector;
        gyroscope_x_vector(end-79:end)     = n_gyroscope_x_vector;
        gyroscope_y_vector(end-79:end)     = n_gyroscope_y_vector;
        gyroscope_z_vector(end-79:end)     = n_gyroscope_z_vector;
        current_u_vector(end-1599:end)     = n_current_u_vector;
        current_v_vector(end-1599:end)     = n_current_v_vector;
        current_w_vector(end-1599:end)     = n_current_w_vector;
 
        if save_enable == 1
            if is_file_open == 0
                fname = fullfile(OUTPUT_FOLDER, sprintf('tryb_%s.txt', datestr(now,'yyyy_mm_dd-HH_MM_SS_FFF')));
                hFile = fopen(fname, 'a');
                is_file_open = 1;
                fprintf(hFile,'curr_u,curr_v,curr_w,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z\r\n');
                log_t0 = tic;  
                fprintf("saving started: %s\n", fname);
            end

            for k=1:length(n_current_u_vector)
                if mod(k-1, 20) == 0
                    fprintf(hFile, '%d,%d,%d,',n_current_u_vector(k),n_current_v_vector(k),n_current_w_vector(k));
                    idx = floor(0.95 + k/20);
                    fprintf(hFile,'%d,%d,%d,', ...
                        n_accelerometer_x_vector(idx), ...
                        n_accelerometer_y_vector(idx), ...
                        n_accelerometer_z_vector(idx));
                    fprintf(hFile,'%d,%d,%d\r\n', ...
                        n_gyroscope_x_vector(idx), ...
                        n_gyroscope_y_vector(idx), ...
                        n_gyroscope_z_vector(idx));
                else
                    fprintf(hFile, '%d,%d,%d\r\n', ...
                        n_current_u_vector(k),n_current_v_vector(k),n_current_w_vector(k));
                end
            end

            if toc(log_t0) >= TOTAL_T_S
                save_enable   = 0;
                if is_file_open, fclose(hFile); is_file_open = 0; end
                allow_save_start = 1;
                fprintf("saving stopped (auto %d s)\n", TOTAL_T_S);
            end
        end

        refreshdata(hACCx); refreshdata(hACCy); refreshdata(hACCz);
        refreshdata(hGYx);  refreshdata(hGYy);  refreshdata(hGYz);
        refreshdata(hCUu);  refreshdata(hCUv);  refreshdata(hCUw);
    end

    ch = get(gcf,'currentcharacter');
    if ~isempty(ch)
        switch ch
            case 'q'  % exit
                close(gcf); break;

            case 'z'  % set MAX = 180°
                if ESP_ENABLED && toc(lastHttpTic) >= HTTP_MIN_DT
                    esp_servo('set', SERVO_MAX_DEG, ESP_BASE);
                    servo_deg_current = SERVO_MAX_DEG;
                    lastHttpTic = tic;
                    fprintf("SERVO -> %d° (MAX)\n", SERVO_MAX_DEG);
                end

            case 'b'  % set 0°
                if ESP_ENABLED && toc(lastHttpTic) >= HTTP_MIN_DT
                    esp_servo('set', SERVO_MIN_DEG, ESP_BASE);
                    servo_deg_current = SERVO_MIN_DEG;
                    profile_active    = false;
                    lastHttpTic  = tic;
                    fprintf("SERVO -> %d° (BACK)\n", SERVO_MIN_DEG);
                end

            case 's'  % START: 180° 
                save_enable      = 1;
                allow_save_start = 0;

                if ESP_ENABLED
                    if toc(lastHttpTic) >= HTTP_MIN_DT
                        esp_servo('set', SERVO_MAX_DEG, ESP_BASE);
                        servo_deg_current = SERVO_MAX_DEG;
                        lastHttpTic = tic;
                        pause(0.1);
                    end
                    profile_active   = true;
                    profile_t0       = tic;
                    profile_last_deg = NaN;
                    fprintf("START: %d° -> %d° w %d s (opuszczanie gazu)\n", ...
                        SERVO_MAX_DEG, SERVO_END_DEG, TOTAL_T_S);
                end

            case 'd'
                save_enable = 0;
                if is_file_open, fclose(hFile); is_file_open = 0; end
                allow_save_start = 1;
                profile_active  = false;
                fprintf("STOP saving");
        end
        set(gcf,'currentcharacter',char(0));
    end

    if ESP_ENABLED && profile_active
        t = toc(profile_t0);   

        if t < 0, t = 0; end
        if t > TOTAL_T_S, t = TOTAL_T_S; end

        u = t / TOTAL_T_S;   
        target_deg = SERVO_MAX_DEG + (SERVO_END_DEG - SERVO_MAX_DEG) * u; 
        target_deg = round(target_deg);
        target_deg = min(SERVO_MAX_DEG, max(SERVO_END_DEG, target_deg));

        if (isnan(profile_last_deg) || target_deg ~= profile_last_deg) && toc(lastHttpTic) >= HTTP_MIN_DT
            esp_servo('set', target_deg, ESP_BASE);
            servo_deg_current = target_deg;
            profile_last_deg  = target_deg;
            lastHttpTic       = tic;
            fprintf("PROFILE t=%.1fs -> %3d°\n", t, target_deg);
        end

        if t >= TOTAL_T_S
            profile_active = false;
            fprintf("PROFILE STOP: t=%.1fs, end deg=%d°\n", t, servo_deg_current);
        end
    end

    updateTitle();
end

if ~isempty(hSerial), hSerial = []; end
if is_file_open, fclose(hFile); end

function out = esp_servo(cmd, val, base)
    if nargin < 3 || isempty(base), base = 'http://192.168.4.1'; end
    opts = weboptions('Timeout', 2);
    switch lower(cmd)
        case 'status'
            url = [base '/status'];
        case 'set'
            assert(nargin>=2,'Enter the angle in degrees');
            url = sprintf('%s/set?deg=%d', base, round(val));
        case 'move'
            assert(nargin>=2,'Enter delta in degrees');
            url = sprintf('%s/move?rel=%d', base, round(val));
        otherwise
            error('Unknown command: %s', cmd);
    end
    out = struct();
    try
        out = webread(url, opts); 
    catch
        try
            txt = webread(url, weboptions('Timeout',2,'ContentType','text'));
            d = str2double(strtrim(txt));
            if ~isnan(d), out.deg = d; end
        catch
        end
    end
end

function y = ternary(cond, a, b)
    if cond, y = a; else, y = b; end
end
