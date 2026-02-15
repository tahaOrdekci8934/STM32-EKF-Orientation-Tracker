function F14_Visualizer(portName, baudRate)
% F14_VISUALIZER Real-time 3D orientation visualization via Serial Port.
%
%   F14_VISUALIZER(portName, baudRate) establishes a connection to a 
%   microcontroller, parses quaternion data, and updates a 3D F-14 model.
%
%   Inputs:
%       portName - String (e.g., '/dev/cu.usbmodem1103' or 'COM3')
%       baudRate - Numeric (e.g., 115200)
%
%   Data Packet Format:
%       [4x4 Bytes Float Quaternions] + [3x4 Bytes Spares/Other] + [4 Bytes Magic Tail]
%       Total Packet Size: 32 Bytes
%
%   Author: [Your Name]
%   License: MIT

%% --- 1. DEFAULT PARAMETERS ---
if nargin < 2
    baudRate = 115200;
end
if nargin < 1
    error('Port name must be specified (e.g., "COM3" or "/dev/tty...")');
end

%% --- 2. SERIAL PORT INITIALIZATION ---
try
    serialObj = serialport(portName, baudRate);
    flush(serialObj);
    
    % Ensure port is closed even if the script errors or is stopped manually
    cleanupObj = onCleanup(@() delete(serialObj));
    fprintf('Connected to %s at %d baud.\n', portName, baudRate);
catch ME
    error('Serial Connection Failed: %s', ME.message);
end

%% --- 3. GRAPHICAL SCENE SETUP ---
fig = figure(...
    'Color',            [0.05 0.1 0.2], ...
    'Name',             'F-14 Avionics Visualizer', ...
    'NumberTitle',      'off', ...
    'MenuBar',          'none', ...
    'ToolBar',          'none' ...
);

ax = axes(...
    'Parent',           fig, ...
    'Projection',       'perspective', ...
    'Color',            [0.1 0.15 0.3], ...
    'XColor',           'none', ...
    'YColor',           'none', ...
    'ZColor',           'none', ...
    'Visible',          'off' ...
);

hold(ax, 'on');
axis(ax, 'equal');
view(ax, 3);

try
    % Load and preprocess aircraft geometry
    TR = stlread('F14.stl');
    v  = double(TR.Points);
    f  = double(TR.ConnectivityList);
    
    % Reduce mesh complexity for real-time rendering performance
    [f, v] = reducepatch(f, v, 0.2);
    
    % Normalize geometry to unit scale and center origin
    v = v - mean(v);
    v = v / max(abs(v(:)));
    
    % Initialize transformation group
    hGroup = hgtransform('Parent', ax);
    
    % Create mesh object
    patch(...
        'Parent',           hGroup, ...
        'Vertices',         v, ...
        'Faces',            f, ...
        'FaceColor',        [0.6 0.6 0.65], ...
        'EdgeColor',        'none', ...
        'SpecularStrength', 0.8, ...
        'FaceLighting',     'phong' ...
    );
    
    % Cinematic Lighting Configuration
    light('Position', [ 5  5  5], 'Color', [1.0 0.6 0.2]);
    light('Position', [-5 -5  2], 'Color', [0.2 0.4 0.8]);
    material shiny;
    
    % Camera Configuration
    axis(ax, 'tight');
    set(ax, 'XLim', [-1 1], 'YLim', [-1 1], 'ZLim', [-1 1]);
    camva(7);
    
catch ME
    error('Model Initialization Failed: Check if F14.stl exists. Error: %s', ME.message);
end

%% --- 4. DATA PROCESSING LOOP ---
packetSize = 32;
lastValidT = eye(4);
magicTail  = uint32(2139062143); % 0x7F7FFFFF (Sentinel Value)

fprintf('Visualizer active. Close the figure window to terminate.\n');

while ishandle(fig)
    if serialObj.NumBytesAvailable >= packetSize
        % Read raw byte stream
        rawData = read(serialObj, packetSize, "uint8");
        
        % Cast to single-precision floating point
        floatData = typecast(uint8(rawData), 'single');
        
        % Validate packet integrity via tail sentinel
        if typecast(floatData(8), 'uint32') == magicTail
            q = floatData(1:4); % Quaternions [q0, q1, q2, q3]
            
            % Robustness check: filter NaNs and null vectors
            if any(isnan(q)) || any(isinf(q)) || all(q == 0)
                T = lastValidT;
            else
                % Construct Rotation Matrix from Quaternions
                R = [ ...
                    1 - 2*(q(3)^2 + q(4)^2),   2*(q(2)*q(3) - q(1)*q(4)),   2*(q(2)*q(4) + q(1)*q(3)); ...
                    2*(q(2)*q(3) + q(1)*q(4)),   1 - 2*(q(2)^2 + q(4)^2),   2*(q(3)*q(4) - q(1)*q(2)); ...
                    2*(q(2)*q(4) - q(1)*q(3)),   2*(q(3)*q(4) + q(1)*q(2)), 1 - 2*(q(2)^2 + q(3)^2) ...
                ];
                
                T = eye(4);
                T(1:3, 1:3) = R;
                lastValidT = T;
            end
            
            % Update 3D transformation
            try
                set(hGroup, 'Matrix', T);
                drawnow limitrate;
            catch
                % Suppress transient graphics errors
            end
        else
            % Synchronization: flush buffer if magic tail mismatch occurs
            flush(serialObj);
        end
    end
end

fprintf('Session terminated successfully.\n');

end