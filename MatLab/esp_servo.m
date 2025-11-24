function out = esp_servo(cmd, val)
%esp_servo('status')        % check angle
%esp_servo('set', 90)       % set 90°
%esp_servo('move', +20)     % left +20°
%esp_servo('move', -10)     % right -10°
%   AP ESP (192.168.4.1).

base = 'http://192.168.4.1';
opts = weboptions('Timeout', 5);

switch lower(cmd)
    case 'status'
        url = [base '/status'];
    case 'set'
        assert(nargin==2, 'Enter the angle in degrees');
        url = sprintf('%s/set?deg=%d', base, round(val));
    case 'move'
        assert(nargin==2, 'Enter delta in degrees');
        url = sprintf('%s/move?rel=%d', base, round(val));
    otherwise
        error('Unknown command: %s', cmd);
end

try
    out = webread(url, opts);  
catch ME
    warning('No response from ESP: %s', ME.message);
    out = struct();
end
end
