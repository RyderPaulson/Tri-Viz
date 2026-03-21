classdef TriViz < handle
    %TRIVIZ  Trilateralization Visualizer
    %
    %  Displays a polar axes centred on the beacon triangle with:
    %    - Labeled beacon markers (fixed)
    %    - Trilateralized robot position (heading-oriented triangle)
    %    - Robot movement trail
    %
    %  Requires TriVizStreamer, RawDataEventData, and TrilatEventData on the
    %  MATLAB path alongside this file.
    %
    %  Usage: app = TriViz();

    %% Properties
    properties (Access = private)
        % UI Components
        UIFigure matlab.ui.Figure
        PolarAxes matlab.graphics.axis.PolarAxes
        ConnectButton matlab.ui.control.Button
        StatusLabel matlab.ui.control.Label

        % Plot handles
        BeaconScatter % single Scatter - all 3 beacon positions
        BeaconLabels % [1x3] text handles
        RobotMarker % polarplot closed triangle - trilateralized robot position
        RobotLabel % text handle for robot position label
        TrailLine % polarplot line - robot movement history

        % Geometry (metres, Cartesian, relative to physical origin)
        BeaconPositions double  % [3x2]
        AxesCenter      double  % [1x2] centroid of beacons is the polar origin
        MaxBeaconRho    double  % scalar – farthest beacon rho; lower bound for auto-scale

        % Connection & streaming state
        Connected        logical
        Streamer         TriVizStreamer
        StreamListeners  % [1xN] event.listener – released on disconnect

        % Robot trail buffer
        TrailTheta double  % [1xN] radians
        TrailRho   double  % [1xN] metres
        MaxTrailLen double
    end

    %% Public Methods - App Lifecycle
    methods (Access = public)

        function app = TriViz(inputsCsv, outputCsv)
            %TRIVIZ  Create the visualizer and load the data streams.
            %
            %   app = TriViz()
            %   app = TriViz(inputsCsv, outputCsv)
            %
            %   Defaults to 'data/inputs.csv' and 'data/output.csv' relative
            %   to the working directory.

            if nargin < 1, inputsCsv = 'data/inputs.csv'; end
            if nargin < 2, outputCsv  = 'data/output.csv';  end

            app.Connected       = false;
            app.TrailTheta      = zeros(1, 0);
            app.TrailRho        = zeros(1, 0);
            app.MaxTrailLen     = 300;
            app.StreamListeners = [];

            app.Streamer = TriVizStreamer(inputsCsv, outputCsv);

            createComponents(app);
            initGeometry(app);
            drawStaticElements(app);
        end

        function delete(app)
            if ~isempty(app.Streamer) && isvalid(app.Streamer)
                delete(app.Streamer);
            end
            if isvalid(app.UIFigure)
                delete(app.UIFigure);
            end
        end

    end

    %% Private Methods
    methods (Access = private)

        % Component Creation

        function createComponents(app)
            W = 840;  H = 720;

            app.UIFigure = uifigure( ...
                'Name', 'TriViz - Trilateralization Visualizer', ...
                'Position', [80 80 W H], ...
                'Color', [0.12 0.12 0.15], ...
                'Visible', 'off');

            % Stop the data timer cleanly when the window is closed.
            app.UIFigure.CloseRequestFcn = @(~,~) delete(app);

            % Top toolbar 
            toolbar = uipanel(app.UIFigure, ...
                'Position', [0 H-54 W 54], ...
                'BackgroundColor', [0.09 0.09 0.12], ...
                'BorderType', 'none');

            app.ConnectButton = uibutton(toolbar, ...
                'Text', 'Connect', ...
                'Position', [16 12 114 30], ...
                'BackgroundColor', [0.055 0.514 0.361], ...
                'FontColor', [1.0 1.0 1.0], ...
                'FontWeight', 'bold', ...
                'FontSize', 13, ...
                'ButtonPushedFcn', @(~,~) onConnectToggle(app));

            app.StatusLabel = uilabel(toolbar, ...
                'Text', 'Status: Disconnected', ...
                'Position', [148 12 W-164 30], ...
                'FontColor', [0.55 0.55 0.60], ...
                'FontSize', 12, ...
                'BackgroundColor', [0.09 0.09 0.12]);

            % Polar axes
            app.PolarAxes = polaraxes(app.UIFigure, ...
                'Units', 'pixels', ...
                'Position', [50 20 W-90 H-100]);

            pa = app.PolarAxes;
            pa.Color = [0.10 0.10 0.13];
            pa.GridColor = [0.30 0.30 0.35];
            pa.GridAlpha = 0.1;
            pa.ThetaColor = [0.70 0.70 0.75];
            pa.RColor = [0.70 0.70 0.75];
            pa.FontSize = 9;

            app.UIFigure.Visible = 'on';
        end

        % Geometry

        function initGeometry(app)
            %INITGEOMETRY  Solve for beacon positions from the loaded CSV data.
            %
            %  Calls calibrateBeacons() on the already-loaded Streamer so that
            %  beacon layout and the polar axes centre reflect the actual physical
            %  setup rather than a hardcoded placeholder.

            app.BeaconPositions = calibrateBeacons(app.Streamer);  % [3x2]
            app.AxesCenter      = mean(app.BeaconPositions, 1);    % [1x2]
        end

        function [th, r] = toPolar(app, xy)
            % Convert absolute Cartesian position(s) [Nx2] to
            % polar [theta (rad), rho (m)] relative to the axes centre.
            dx = xy(:,1) - app.AxesCenter(1);
            dy = xy(:,2) - app.AxesCenter(2);
            [th, r] = cart2pol(dx, dy);
        end

        % Static Plot Elements

        function drawStaticElements(app)
            ax = app.PolarAxes;
            hold(ax, 'on');

            % beaconColor = [0.945 0.486 0.639]; % pink
            beaconColor = [0.055 0.514 0.361];
            robotColor  = [0.055 0.514 0.361]; % green

            % Compute polar coords first so max(bR) drives the initial axis 
            % limits (calibrated beacons may not be equidistant).
            [bTh, bR] = toPolar(app, app.BeaconPositions);
            app.MaxBeaconRho = max(bR);

            % Initial limits: 2× the farthest beacon, no robot position yet.
            setAxisLimits(app, app.MaxBeaconRho * 2);

            app.BeaconScatter = polarscatter( ...
                ax, bTh, bR, 230, ...
                beaconColor, 'o', ...
                'LineWidth', 2 ...
                );

            beaconLabelOffset = 0;

            app.BeaconLabels = gobjects(1, 3);
            for i = 1:3
                bX = bR(i) * cos(bTh(i));
                bY = bR(i) * sin(bTh(i)) + beaconLabelOffset;
                bLabelR = sqrt(bX^2 + bY^2);
                bLabelTh = atan2(bY, bX);
                app.BeaconLabels(i) = text(ax, bLabelTh, bLabelR, ...
                    sprintf('%d', i), ...
                    'Color', beaconColor, ...
                    'FontWeight', 'bold', ...
                    'FontSize', 10, ...
                    'HorizontalAlignment', 'center');
            end

            % Robot trail (empty until connected)
            app.TrailLine = polarplot(ax, NaN, NaN, ...
                'Color', [0.50 0.50 0.55], ...
                'LineWidth', 0.9);

            % Trilateralized robot position (heading-oriented triangle)
            % Drawn as a closed 3-vertex polarplot; vertices are recomputed
            % each frame in updateDisplay via robotTriVerts().
            app.RobotMarker = polarplot(ax, NaN(4,1), NaN(4,1), ...
                'Color', robotColor, ...
                'LineWidth', 2.5);
            
            % Removed label text because it overlapped. If in final 
            app.RobotLabel = text(ax, 0, 0, '', ...
                'Color', robotColor, ...
                'FontWeight', 'bold', ...
                'FontSize', 10, ...
                'HorizontalAlignment', 'center', ...
                'Visible', 'off');
        end

        % Connection Handling

        function onConnectToggle(app)
            if ~app.Connected
                connectStream(app);
            else
                disconnectStream(app);
            end
        end

        function connectStream(app)
            app.Connected = true;
            app.ConnectButton.Text        = 'Disconnect';
            app.ConnectButton.BackgroundColor = [0.65 0.18 0.18];
            app.StatusLabel.Text      = 'Status: Connecting...';
            app.StatusLabel.FontColor = [0.95 0.75 0.30];

            % Subscribe to trilaterated position + heading.
            % The RawData event is also available here if needed in future.
            app.StreamListeners = addlistener(app.Streamer, ...
                'TrilateralizedData', ...
                @(~, e) updateDisplay(app, e.Position, e.Heading));

            app.Streamer.connect();

            app.StatusLabel.Text      = 'Status: Streaming';
            app.StatusLabel.FontColor = [0.40 0.90 0.60];
        end

        function disconnectStream(app)
            app.Streamer.disconnect();

            % Release event listeners so no stale callbacks fire.
            if ~isempty(app.StreamListeners)
                delete(app.StreamListeners);
                app.StreamListeners = [];
            end

            app.Connected = false;
            app.ConnectButton.Text        = 'Connect';
            app.ConnectButton.BackgroundColor = [0.055 0.514 0.361];
            app.StatusLabel.Text      = 'Status: Disconnected';
            app.StatusLabel.FontColor = [0.55 0.55 0.60];
        end

        function updateDisplay(app, robotXY, heading)
            % Update all dynamic plot elements.
            %
            %   robotXY  [1x2] trilateralized robot position (metres, Cartesian)
            %   heading  scalar heading in radians (atan2: 0 = east, CCW)

            % Trilateralized robot
            [rTh, rR] = toPolar(app, robotXY);
            [vTh, vR] = robotTriVerts(app, robotXY, heading);
            app.RobotMarker.ThetaData = vTh;
            app.RobotMarker.RData     = vR;
            app.RobotLabel.Visible    = 'on';
            app.RobotLabel.Position   = [rTh, rR * 1.06, 0];

            % Trail
            app.TrailTheta(end+1) = rTh;
            app.TrailRho(end+1) = rR;
            if numel(app.TrailTheta) > app.MaxTrailLen
                keep = numel(app.TrailTheta) - app.MaxTrailLen + 1;
                app.TrailTheta = app.TrailTheta(keep:end);
                app.TrailRho = app.TrailRho(keep:end);
            end

            % Auto-scale: default headroom is 2× the farthest beacon.
            % Expand beyond that if any point in the trail exceeds it, with 10% padding,
            % so the full path history is always visible.
            setAxisLimits(app, max(app.MaxBeaconRho * 2, max(app.TrailRho) * 1.10));
            app.TrailLine.ThetaData = app.TrailTheta;
            app.TrailLine.RData = app.TrailRho;

            % Status bar with compass bearing and distance from system centre
            bearing = mod(90 - rad2deg(rTh), 360);
            app.StatusLabel.Text = sprintf( ...
                'Status: Streaming  |  Bearing: %05.1f deg  |  Distance: %.3f m', ...
                bearing, rR);
        end

        function setAxisLimits(app, newRMax)
            %SETAXISLIMITS  Set RLim to newRMax; update ticks only when the
            %  rounded bin changes to avoid label flicker.  Callers are
            %  responsible for applying any padding before passing newRMax.

            ax = app.PolarAxes;

            if abs(newRMax - ax.RLim(2)) < newRMax * 1e-6
                return   % negligible change – skip the redraw
            end
            ax.RLim = [0, newRMax];

            % Recompute nice-number ticks (1-2-5-10 series).
            rawStep = newRMax / 5;
            mag = 10 ^ floor(log10(max(rawStep, 1e-9)));
            niceMult = [1, 2, 5, 10];
            tickStep = mag * niceMult(find(mag * niceMult >= rawStep, 1));
            newTick = tickStep : tickStep : ceil(newRMax / tickStep) * tickStep;
            if ~isequal(ax.RTick, newTick)
                ax.RTick = newTick;
            end
        end

        function [vTh, vR] = robotTriVerts(app, centerXY, heading)
            % Polar coords of a closed equilateral triangle centred on
            % centerXY, with its tip pointing in the heading direction.
            % The four points form a closed polygon (first == last).
            % Scale marker to ~5.6% of beacon distance. 
            s = norm(app.BeaconPositions(1,:) - app.AxesCenter) * 0.056;
            va = heading + [0; 2*pi/3; 4*pi/3; 0]; % tip at heading, CCW
            vx = centerXY(1) + s * cos(va);
            vy = centerXY(2) + s * sin(va);
            [vTh, vR] = toPolar(app, [vx, vy]);
        end

    end

end
