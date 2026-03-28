classdef TriVizStreamer < handle
    %TRIVIZSTREAMER  Asynchronous data streamer for TriViz.
    %
    %  Reads pre-recorded beacon data from CSV files and emits MATLAB events
    %  at a fixed rate.  When the recording ends the streamer performs a
    %  biased random walk back to the starting position before seamlessly
    %  looping the recording from the beginning.
    %
    %  Events
    %    RawData            – fired on each CSV playback step.
    %                         Payload: RawDataEventData  (.Distances [4x3])
    %
    %    TrilateralizedData – fired on every step, including the homing walk.
    %                         Payload: TrilatEventData   (.Position [1x2],
    %                                                     .Heading  scalar,
    %                                                     .BeaconDists [1x3])
    %
    %  Usage
    %    s = TriVizStreamer('data/inputs.csv', 'data/output.csv');
    %    addlistener(s, 'TrilateralizedData', @(~,e) disp(e.Position));
    %    s.connect();
    %    ...
    %    s.disconnect();   % pause  – state is preserved
    %    s.connect();      % resume – picks up where it left off
    %    delete(s);        % clean up timer on teardown

    events
        RawData % one raw distance block per CSV time-step
        TrilateralizedData % one (x,y) + heading + beacon dists per step
    end

    properties (Access = private)
        % Loaded data
        RawSteps % {Nx1} cell – each entry is a [4x3] distance matrix
        TrilatXY % [Nx2] double – (x,y) positions from output CSV
        NumSteps % scalar integer
        TypicalStep % scalar – median step-to-step distance (for homing speed)

        % Playback state
        CurrentStep % integer index into RawSteps / TrilatXY
        PrevPos % [1x2] position from the previous tick (heading calc)
        State % 'disconnected' | 'streaming' | 'homing'
        ResumeState % state to restore when connect() is called

        % Homing walk state
        HomePos % [1x2] target (first row of output CSV)
        HomingPos % [1x2] current synthetic position during homing
        HomingVel % [1x2] current velocity during homing
        MaxHomeDist % scalar – distance from data-end to home (alpha scale)

        % Timer
        StreamTimer timer
        StreamRate % Hz (ticks per second)
    end

    methods (Access = public)

        function obj = TriVizStreamer(streamRate, inputsPath, outputsPath)
            %TRIVIZSTREAMER  Load CSV data and prepare for streaming.
            %
            %   obj = TriVizStreamer(inputsPath, outputsPath)
            %   obj = TriVizStreamer(inputsPath, outputsPath, streamRate)
            %
            %   streamRate – ticks per second, default 10 Hz 
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

            % Derive a typical step distance from the data so the homing walk
            % moves at the same pace as the recorded trajectory.
            deltas = diff(obj.TrilatXY, 1, 1);
            stepDists = sqrt(sum(deltas .^ 2, 2));
            obj.TypicalStep = median(stepDists(stepDists > 1e-6));
            if isnan(obj.TypicalStep) || obj.TypicalStep < 1e-6
                obj.TypicalStep = 1; % safe fallback
            end
        end

        function connect(obj)
            %CONNECT  Begin (or resume) streaming.
            %
            %  Safe to call multiple times.  If the streamer was paused mid-homing
            %  it resumes homing; if paused mid-playback it resumes playback.

            if ~strcmp(obj.State, 'disconnected')
                return % already running
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
            %DISCONNECT  Pause streaming; all state is preserved for resume.

            obj.ResumeState = obj.State; % remember where we are
            obj.State = 'disconnected';

            if ~isempty(obj.StreamTimer) && isvalid(obj.StreamTimer)
                stop(obj.StreamTimer);
            end
        end

        function delete(obj)
            %DELETE  Stop and destroy the internal timer on teardown.

            if ~isempty(obj.StreamTimer) && isvalid(obj.StreamTimer)
                stop(obj.StreamTimer);
                delete(obj.StreamTimer);
            end
        end

        function beaconPositions = calibrateBeacons(obj)
            %CALIBRATEBEACONS  Solve for beacon positions from the loaded CSV data.
            %
            %  Uses linearised least-squares: subtracting the i=1 reference
            %  equation from each subsequent row cancels the quadratic terms,
            %  leaving a (N-1)×2 linear system per beacon.  Because the LHS
            %  matrix A is shared across all three beacons a single backslash
            %  call solves all three simultaneously.
            %
            %  Returns:
            %    beaconPositions – [3×2] double, rows are [Bx, By] for beacons 1-3.

            N = obj.NumSteps;

            % Need at least 3 steps to form a rank-2 system.
            if N < 3
                warning('TriVizStreamer:calibrateBeacons:insufficientData', ...
                    'Need >= 3 time steps for calibration. Using R=5 equilateral fallback.');
                beaconPositions = TriVizStreamer.fallbackGeometry();
                return
            end

            % Average the 4 corner distances for each beacon at each step → [N×3]
            D = zeros(N, 3);
            for i = 1 : N
                D(i, :) = mean(obj.RawSteps{i}, 1);
            end

            rx = obj.TrilatXY(:, 1);
            ry = obj.TrilatXY(:, 2);

            % Shared LHS: 2*[rx_1 - rx_i, ry_1 - ry_i] for i = 2..N  → [(N-1)×2]
            A = 2 * [(rx(1) - rx(2:N)), (ry(1) - ry(2:N))];

            if rank(A) < 2
                warning('TriVizStreamer:calibrateBeacons:degenerate', ...
                    'Trajectory is rank-deficient (collinear or stationary). Using R=5 equilateral fallback.');
                beaconPositions = TriVizStreamer.fallbackGeometry();
                return
            end

            % RHS for all 3 beacons at once → [(N-1)×3]
            % b_col_j = d_ij^2 - d_1j^2 - rx_i^2 + rx_1^2 - ry_i^2 + ry_1^2
            scalarCol = -rx(2:N).^2 + rx(1)^2 ...
                        -ry(2:N).^2 + ry(1)^2; % [(N-1)×1], same for all j
            Bmat = D(2:N, :).^2 - D(1, :).^2 + scalarCol; % [(N-1)×3] (row broadcast)

            beaconPositions = (A \ Bmat)'; % [3×2]
        end

    end

    methods (Access = private)

        function loadData(obj, inputsPath, outputsPath)
            % output CSV
            if ~isfile(outputsPath)
                error('TriVizStreamer: output file not found: %s', outputsPath);
            end
            outT = readtable(outputsPath, 'VariableNamingRule', 'preserve');
            cols = outT.Properties.VariableNames;

            % Accept x_coordinate / y_coordinate (case-insensitive)
            xi = find(strcmpi(cols, 'x_coordinate'), 1);
            yi = find(strcmpi(cols, 'y_coordinate'), 1);
            if isempty(xi) || isempty(yi)
                error('TriVizStreamer: output CSV must contain x_coordinate and y_coordinate columns.');
            end
            obj.TrilatXY = [outT{:, xi}, outT{:, yi}];
            obj.NumSteps = size(obj.TrilatXY, 1);

            % inputs CSV
            if ~isfile(inputsPath)
                error('TriVizStreamer: inputs file not found: %s', inputsPath);
            end
            obj.RawSteps = parseBlockCSV(obj, inputsPath);

            % Validate that both files have the same number of time steps.
            nRaw = numel(obj.RawSteps);
            if nRaw ~= obj.NumSteps
                warning('TriVizStreamer: inputs has %d blocks but output has %d rows. Using min.', ...
                    nRaw, obj.NumSteps);
                obj.NumSteps = min(nRaw, obj.NumSteps);
            end
        end

        function steps = parseBlockCSV(~, path)
            %PARSEBLOCKSV  Parse an inputs CSV whose time-steps are separated
            %  by empty/comma-only rows (e.g. ",,,").
            %
            %  Returns a cell array of [4x3] distance matrices.

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

            for i = 2 : numel(rawLines) % skip header (row 1)
                line = strtrim(rawLines{i});

                % Separator row: empty or all commas/spaces
                isSep = isempty(line) || all(line == ',') || all(ismember(line, {',', ' '}));

                if isSep
                    if ~isempty(block)
                        steps{end+1} = block; %#ok<AGROW>
                        block = [];
                    end
                else
                    parts = str2double(strsplit(line, ','));
                    % Layout: Corner(ignored), Anchor1, Anchor2, Anchor3
                    if numel(parts) >= 4 && ~any(isnan(parts(2:4)))
                        block = [block; parts(2:4)]; %#ok<AGROW>
                    end
                end
            end
            % Flush the final block in case there is no trailing separator.
            if ~isempty(block)
                steps{end+1} = block;
            end
        end

    end

    methods (Access = private)

        function tick(obj)
            %TICK  Called by the internal timer on every period.

            switch obj.State
                case 'streaming'
                    streamStep(obj);
                case 'homing'
                    homingStep(obj);
                % 'disconnected' should never reach here, but guard anyway.
            end
        end

        function streamStep(obj)
            i = obj.CurrentStep;
            xy = obj.TrilatXY(i, :);

            % Mean distance from each beacon across all 4 corners.
            meanDists = mean(obj.RawSteps{i}, 1); % [1x3]

            % Fire raw event (distance matrix for this time-step).
            notify(obj, 'RawData', RawDataEventData(obj.RawSteps{i}));

            % Fire trilaterated event (position + heading + beacon distances).
            notify(obj, 'TrilateralizedData', ...
                TrilatEventData(xy, headingFromDelta(obj, xy), meanDists));

            obj.PrevPos = xy;
            obj.CurrentStep = obj.CurrentStep + 1;

            if obj.CurrentStep > obj.NumSteps
                initHoming(obj);
            end
        end

        function hdg = headingFromDelta(obj, currentPos)
            %HEADINGFROMDELTA  Derive heading from movement since last tick.

            if isempty(obj.PrevPos) || norm(currentPos - obj.PrevPos) < 1e-9
                hdg = 0;
            else
                d = currentPos - obj.PrevPos;
                hdg = atan2(d(2), d(1));
            end
        end

        function initHoming(obj)
            %INITHOMING  Transition into the biased random-walk phase.

            obj.State = 'homing';
            obj.HomingPos = obj.TrilatXY(end, :);
            obj.MaxHomeDist = norm(obj.HomingPos - obj.HomePos);
            if obj.MaxHomeDist < 1e-6
                obj.MaxHomeDist = 1;
            end

            % Seed velocity pointing toward home at typical playback speed.
            toHome = obj.HomePos - obj.HomingPos;
            obj.HomingVel = obj.TypicalStep * toHome / max(norm(toHome), 1e-9);
            obj.PrevPos = obj.HomingPos;
        end

        function homingStep(obj)
            %HOMINGSTEP  Walk toward home in a near-straight line with small
            %  angular noise, simulating measurement jitter rather than a
            %  random walk.  The base heading always points at the start
            %  position; randn() adds ~±8° of noise per step (1-sigma).

            toHome = obj.HomePos - obj.HomingPos;
            dist = norm(toHome);

            % Arrived: switch back to playback from the first row.
            if dist <= obj.TypicalStep * 2
                obj.State = 'streaming';
                obj.CurrentStep = 1;
                obj.PrevPos = obj.HomePos;
                return
            end

            % Head straight toward home, perturbed by small angular noise.
            baseAngle = atan2(toHome(2), toHome(1));
            noisyAngle = baseAngle + 0.14 * randn(); % ~±8° 1-sigma
            obj.HomingVel = obj.TypicalStep * [cos(noisyAngle), sin(noisyAngle)];
            obj.HomingPos = obj.HomingPos + obj.HomingVel;

            % BeaconDists are NaN during homing (no real sensor data).
            notify(obj, 'TrilateralizedData', ...
                TrilatEventData(obj.HomingPos, noisyAngle, NaN(1, 3)));

            obj.PrevPos = obj.HomingPos;
        end

    end

    methods (Static, Access = private)

        function bp = fallbackGeometry()
            %FALLBACKGEOMETRY  Equilateral triangle R=5 used when calibration fails.
            ang = [90, 210, 330];
            bp = 5 * [cosd(ang)', sind(ang)'];
        end

    end

end
