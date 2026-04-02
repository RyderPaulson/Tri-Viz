classdef TriVizStreamer < handle
    %TRIVIZSTREAMER  Streams pre-recorded beacon CSV data to TriViz via
    %  MATLAB events. Loops by walking back to start when recording ends.
    %
    %  Usage
    %    s = TriVizStreamer();
    %    addlistener(s, 'TrilateralizedData', @(~,e) disp(e.Position));
    %    s.connect();
    %    s.disconnect();
    %    delete(s);

    events
        RawData
        TrilateralizedData
    end

    properties (Access = private)
        % Data
        RawSteps % {Nx1} cell of [4x3] distance matrices
        TrilatXY % [Nx2] positions from output CSV
        NumSteps
        TypicalStep % median step distance

        % Playback
        CurrentStep
        PrevPos
        State % 'disconnected' | 'streaming' | 'homing'
        ResumeState

        % Homing
        HomePos
        HomingPos
        HomingVel
        MaxHomeDist

        % Timer
        StreamTimer timer
        StreamRate
    end

    methods (Access = public)

        function obj = TriVizStreamer(streamRate, inputsPath, outputsPath)
            if nargin < 1, streamRate = 10; end
            if nargin < 2, inputsPath = 'data/inputs.csv'; end
            if nargin < 3, outputsPath = 'data/output.csv'; end

            obj.StreamRate = streamRate;
            obj.State = 'disconnected';
            obj.ResumeState = 'streaming';
            obj.CurrentStep = 1;
            obj.PrevPos = [];

            loadData(obj, inputsPath, outputsPath);

            obj.HomePos = obj.TrilatXY(1, :);
            obj.MaxHomeDist = norm(obj.TrilatXY(end, :) - obj.HomePos);
            if obj.MaxHomeDist < 1e-6
                obj.MaxHomeDist = 1;
            end

            deltas = diff(obj.TrilatXY, 1, 1);
            stepDists = sqrt(sum(deltas .^ 2, 2));
            obj.TypicalStep = median(stepDists(stepDists > 1e-6));
            if isnan(obj.TypicalStep) || obj.TypicalStep < 1e-6
                obj.TypicalStep = 1;
            end
        end

        function connect(obj)
            if ~strcmp(obj.State, 'disconnected')
                return
            end

            obj.State = obj.ResumeState;

            if isempty(obj.StreamTimer) || ~isvalid(obj.StreamTimer)
                obj.StreamTimer = timer( ...
                    'ExecutionMode', 'fixedRate', ...
                    'Period', 1 / obj.StreamRate, ...
                    'TimerFcn', @(~,~) tick(obj));
            end
            start(obj.StreamTimer);
        end

        function disconnect(obj)
            obj.ResumeState = obj.State;
            obj.State = 'disconnected';

            if ~isempty(obj.StreamTimer) && isvalid(obj.StreamTimer)
                stop(obj.StreamTimer);
            end
        end

        function delete(obj)
            if ~isempty(obj.StreamTimer) && isvalid(obj.StreamTimer)
                stop(obj.StreamTimer);
                delete(obj.StreamTimer);
            end
        end

        function beaconPositions = calibrateBeacons(obj)
            %CALIBRATEBEACONS  Linearized least-squares solve for beacon XY
            %  from corner distances + known robot positions.

            N = obj.NumSteps;

            if N < 3
                warning('TriVizStreamer:calibrateBeacons:insufficientData', ...
                    'Need >= 3 time steps for calibration. Using R=5 equilateral fallback.');
                beaconPositions = TriVizStreamer.fallbackGeometry();
                return
            end

            % Average 4 corner distances per beacon per step -> [Nx3]
            D = zeros(N, 3);
            for i = 1 : N
                D(i, :) = mean(obj.RawSteps{i}, 1);
            end

            rx = obj.TrilatXY(:, 1);
            ry = obj.TrilatXY(:, 2);

            A = 2 * [(rx(1) - rx(2:N)), (ry(1) - ry(2:N))];

            if rank(A) < 2
                warning('TriVizStreamer:calibrateBeacons:degenerate', ...
                    'Trajectory is rank-deficient (collinear or stationary). Using R=5 equilateral fallback.');
                beaconPositions = TriVizStreamer.fallbackGeometry();
                return
            end

            scalarCol = -rx(2:N).^2 + rx(1)^2 ...
                        -ry(2:N).^2 + ry(1)^2;
            Bmat = D(2:N, :).^2 - D(1, :).^2 + scalarCol;

            beaconPositions = (A \ Bmat)';
        end

    end

    methods (Access = private)

        function loadData(obj, inputsPath, outputsPath)
            % Output CSV
            if ~isfile(outputsPath)
                error('TriVizStreamer: output file not found: %s', outputsPath);
            end
            outT = readtable(outputsPath, 'VariableNamingRule', 'preserve');
            cols = outT.Properties.VariableNames;

            xi = find(strcmpi(cols, 'x_coordinate'), 1);
            yi = find(strcmpi(cols, 'y_coordinate'), 1);
            if isempty(xi) || isempty(yi)
                error('TriVizStreamer: output CSV must contain x_coordinate and y_coordinate columns.');
            end
            obj.TrilatXY = [outT{:, xi}, outT{:, yi}];
            obj.NumSteps = size(obj.TrilatXY, 1);

            % Inputs CSV
            if ~isfile(inputsPath)
                error('TriVizStreamer: inputs file not found: %s', inputsPath);
            end
            obj.RawSteps = parseBlockCSV(obj, inputsPath);

            nRaw = numel(obj.RawSteps);
            if nRaw ~= obj.NumSteps
                warning('TriVizStreamer: inputs has %d blocks but output has %d rows. Using min.', ...
                    nRaw, obj.NumSteps);
                obj.NumSteps = min(nRaw, obj.NumSteps);
            end
        end

        function steps = parseBlockCSV(~, path)
            fid = fopen(path, 'r');
            if fid < 0
                error('TriVizStreamer: cannot open %s', path);
            end
            rawLines = {};
            while ~feof(fid)
                rawLines{end+1} = fgetl(fid); %#ok<AGROW>
            end
            fclose(fid);

            steps = {};
            block = [];

            for i = 2 : numel(rawLines) % skip header
                line = strtrim(rawLines{i});

                isSep = isempty(line) || all(line == ',') || all(ismember(line, {',', ' '}));

                if isSep
                    if ~isempty(block)
                        steps{end+1} = block; %#ok<AGROW>
                        block = [];
                    end
                else
                    parts = str2double(strsplit(line, ','));
                    % Corner(ignored), Anchor1, Anchor2, Anchor3
                    if numel(parts) >= 4 && ~any(isnan(parts(2:4)))
                        block = [block; parts(2:4)]; %#ok<AGROW>
                    end
                end
            end
            if ~isempty(block)
                steps{end+1} = block;
            end
        end

    end

    methods (Access = private)

        function tick(obj)
            switch obj.State
                case 'streaming'
                    streamStep(obj);
                case 'homing'
                    homingStep(obj);
            end
        end

        function streamStep(obj)
            i = obj.CurrentStep;
            xy = obj.TrilatXY(i, :);
            meanDists = mean(obj.RawSteps{i}, 1);

            notify(obj, 'RawData', RawDataEventData(obj.RawSteps{i}));
            notify(obj, 'TrilateralizedData', ...
                TrilatEventData(xy, headingFromDelta(obj, xy), meanDists));

            obj.PrevPos = xy;
            obj.CurrentStep = obj.CurrentStep + 1;

            if obj.CurrentStep > obj.NumSteps
                initHoming(obj);
            end
        end

        function hdg = headingFromDelta(obj, currentPos)
            if isempty(obj.PrevPos) || norm(currentPos - obj.PrevPos) < 1e-9
                hdg = 0;
            else
                d = currentPos - obj.PrevPos;
                hdg = atan2(d(2), d(1));
            end
        end

        % Homing walk

        function initHoming(obj)
            obj.State = 'homing';
            obj.HomingPos = obj.TrilatXY(end, :);
            obj.MaxHomeDist = norm(obj.HomingPos - obj.HomePos);
            if obj.MaxHomeDist < 1e-6
                obj.MaxHomeDist = 1;
            end

            toHome = obj.HomePos - obj.HomingPos;
            obj.HomingVel = obj.TypicalStep * toHome / max(norm(toHome), 1e-9);
            obj.PrevPos = obj.HomingPos;
        end

        function homingStep(obj)
            toHome = obj.HomePos - obj.HomingPos;
            dist = norm(toHome);

            % Close enough -> restart playback
            if dist <= obj.TypicalStep * 2
                obj.State = 'streaming';
                obj.CurrentStep = 1;
                obj.PrevPos = obj.HomePos;
                return
            end

            % Straight line toward home + small angular noise
            baseAngle = atan2(toHome(2), toHome(1));
            noisyAngle = baseAngle + 0.14 * randn();
            obj.HomingVel = obj.TypicalStep * [cos(noisyAngle), sin(noisyAngle)];
            obj.HomingPos = obj.HomingPos + obj.HomingVel;

            notify(obj, 'TrilateralizedData', ...
                TrilatEventData(obj.HomingPos, noisyAngle));

            obj.PrevPos = obj.HomingPos;
        end

    end

    methods (Static, Access = private)

        function bp = fallbackGeometry()
            ang = [90, 210, 330];
            bp = 5 * [cosd(ang)', sind(ang)'];
        end

    end

end
