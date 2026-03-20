classdef TriViz < handle
    %TRIVIZ  Trilateralization Visualizer
    %
    %  Displays a polar axes centred on the beacon triangle with:
    %    - Labeled beacon markers (fixed)
    %    - Raw corner scatter  (4 corners per robot ping, blue squares)
    %    - Trilateralized robot position (green triangle + label)
    %    - Robot movement trail
    %
    %  Usage:  app = TriViz();

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
        AxesCenter double  % [1x2] centroid of beacons is the polar origin

        % Connection & streaming state
        Connected logical
        DataTimer timer

        % Robot trail buffer
        TrailTheta double  % [1xN] radians
        TrailRho double  % [1xN] metres
        MaxTrailLen double

        % TODO Replace with real data reader
        SimPos double  % [1x2] current simulated position (metres)
        SimVel double  % [1x2] current simulated velocity (m/s)
    end

    %% Public Methods - App Lifecycle
    methods (Access = public)

        function app = TriViz()
            app.Connected = false;
            app.TrailTheta = zeros(1, 0);
            app.TrailRho = zeros(1, 0);
            app.MaxTrailLen = 300;
            app.SimPos = [0.0, 0.0];
            app.SimVel = [0.3, 0.2];

            createComponents(app);
            initGeometry(app);
            drawStaticElements(app);
        end

        function delete(app)
            if ~isempty(app.DataTimer) && isvalid(app.DataTimer)
                stop(app.DataTimer);
                delete(app.DataTimer);
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
                'Position', [50 20 W-80 H-90]);

            pa = app.PolarAxes;
            pa.Color = [0.10 0.10 0.13];
            pa.GridColor = [0.30 0.30 0.35];
            pa.GridAlpha = 0.45;
            pa.ThetaColor = [0.70 0.70 0.75];
            pa.RColor = [0.70 0.70 0.75];
            pa.FontSize = 9;

            app.UIFigure.Visible = 'on';
        end

        % Geometry

        function initGeometry(app)
            % Three beacons at the vertices of an equilateral triangle.
            % Beacon 1 is placed at the top (angle = 90 deg), the others
            % are spaced 120 deg apart.  Radius = 5 m - adjust to match
            % your physical setup.
            R = 5;
            ang = [90, 210, 330];   % degrees
            app.BeaconPositions = R * [cosd(ang)', sind(ang)'];
            app.AxesCenter = mean(app.BeaconPositions, 1);  % (0,0) by symmetry
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

            beaconR = norm(app.BeaconPositions(1,:) - app.AxesCenter);
            ax.RLim = [0, beaconR * 1.45];
            ax.RTick = 1:1:ceil(beaconR * 1.45);

            beaconColor = [0.945 0.486 0.639];   % pink
            robotColor = [0.055 0.514 0.361];   % green

            % Beacons
            [bTh, bR] = toPolar(app, app.BeaconPositions);
            app.BeaconScatter = polarscatter(ax, bTh, bR, 230, beaconColor, 'o', 'LineWidth', 2);

            app.BeaconLabels = gobjects(1, 3);
            for i = 1:3
                app.BeaconLabels(i) = text(ax, bTh(i), bR(i) * 1.16, ...
                    sprintf('Beacon %d', i), ...
                    'Color',               beaconColor, ...
                    'FontWeight',          'bold', ...
                    'FontSize',            10, ...
                    'HorizontalAlignment', 'center');
            end

            % Robot trail (empty until connected)
            app.TrailLine = polarplot(ax, NaN, NaN, ...
                'Color',     [0.50 0.50 0.55], ...
                'LineWidth', 0.9);

            % Trilateralized robot position (heading-oriented triangle)
            % Drawn as a closed 3-vertex polarplot; vertices are recomputed
            % each frame in updateDisplay via robotTriVerts().
            app.RobotMarker = polarplot(ax, NaN(4,1), NaN(4,1), ...
                'Color',     robotColor, ...
                'LineWidth', 2.5);
            
            % Removed label text because it overlapped. If in final 
            app.RobotLabel = text(ax, 0, 0, '', ...
                'Color', robotColor, ...
                'FontWeight', 'bold', ...
                'FontSize', 10, ...
                'HorizontalAlignment', 'center', ...
                'Visible', 'off');

            % Legend
            legend(ax, ...
                [app.BeaconScatter, app.RobotMarker], ...
                {'Beacons', 'Robot (trilat.)'}, ...
                'TextColor', [0.80 0.80 0.85], ...
                'Color', [0.11 0.11 0.14], ...
                'EdgeColor', [0.28 0.28 0.32], ...
                'FontSize', 9);
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
            app.ConnectButton.Text = 'Disconnect';
            app.ConnectButton.BackgroundColor = [0.65 0.18 0.18];
            app.StatusLabel.Text = 'Status: Connecting...';
            app.StatusLabel.FontColor = [0.95 0.75 0.30];

            % TODO: Establish data connection
            app.DataTimer = timer( ...
                'ExecutionMode', 'fixedRate', ...
                'Period', 0.10, ...
                'TimerFcn', @(~,~) simulatedUpdate(app));
            start(app.DataTimer);

            app.StatusLabel.Text = 'Status: Streaming';
            app.StatusLabel.FontColor = [0.40 0.90 0.60];
        end

        function disconnectStream(app)
            if ~isempty(app.DataTimer) && isvalid(app.DataTimer)
                stop(app.DataTimer);
                delete(app.DataTimer);
                app.DataTimer = timer.empty;
            end
            app.Connected = false;
            app.ConnectButton.Text = 'Connect';
            app.ConnectButton.BackgroundColor = [0.055 0.514 0.361];
            app.StatusLabel.Text = 'Status: Disconnected';
            app.StatusLabel.FontColor = [0.55 0.55 0.60];
        end

        % Data Update

        function simulatedUpdate(app)
            % Placeholder: performs a smooth random walk within the beacon arena.
            %
            % TODO: Replace with real data ingestion

            speed      = norm(app.SimVel);
            heading    = atan2(app.SimVel(2), app.SimVel(1)) + 0.15 * randn();
            speed      = min(max(speed + 0.05 * randn(), 0.1), 1.5);
            app.SimVel = speed * [cos(heading), sin(heading)];
            app.SimPos = app.SimPos + app.SimVel * 0.10;

            % Bounce off arena boundaries: reverse the relevant velocity
            % component so the heading immediately reflects the new direction.
            for dim = 1:2
                if app.SimPos(dim) > 3.5
                    app.SimPos(dim) = 3.5;
                    app.SimVel(dim) = -abs(app.SimVel(dim));
                elseif app.SimPos(dim) < -3.5
                    app.SimPos(dim) = -3.5;
                    app.SimVel(dim) =  abs(app.SimVel(dim));
                end
            end

            heading = atan2(app.SimVel(2), app.SimVel(1));
            updateDisplay(app, app.SimPos, heading);
        end

        function updateDisplay(app, robotXY, heading)
            % Update all dynamic plot elements.
            %
            %   robotXY  [1x2] trilateralized robot position (metres, Cartesian)
            %   heading  scalar heading in radians (atan2: 0 = east, CCW)

            % Trilateralized robot (heading-oriented triangle)
            [rTh, rR] = toPolar(app, robotXY);
            [vTh, vR] = robotTriVerts(app, robotXY, heading);
            app.RobotMarker.ThetaData = vTh;
            app.RobotMarker.RData     = vR;
            app.RobotLabel.Visible    = 'on';
            app.RobotLabel.Position   = [rTh, rR + 0.45, 0];

            % Trail
            app.TrailTheta(end+1) = rTh;
            app.TrailRho(end+1)   = rR;
            if numel(app.TrailTheta) > app.MaxTrailLen
                keep           = numel(app.TrailTheta) - app.MaxTrailLen + 1;
                app.TrailTheta = app.TrailTheta(keep:end);
                app.TrailRho   = app.TrailRho(keep:end);
            end
            app.TrailLine.ThetaData = app.TrailTheta;
            app.TrailLine.RData     = app.TrailRho;

            % Status bar with compass bearing and distance from system centre
            bearing = mod(90 - rad2deg(rTh), 360);
            app.StatusLabel.Text = sprintf( ...
                'Status: Streaming  |  Bearing: %05.1f deg  |  Distance: %.3f m', ...
                bearing, rR);
        end

        function [vTh, vR] = robotTriVerts(app, centerXY, heading)
            % Polar coords of a closed equilateral triangle centred on
            % centerXY, with its tip pointing in the heading direction.
            % The four points form a closed polygon (first == last).
            s = 0.28; % circumradius (metres)
            va = heading + [0; 2*pi/3; 4*pi/3; 0]; % tip at heading, CCW
            vx = centerXY(1) + s * cos(va);
            vy = centerXY(2) + s * sin(va);
            [vTh, vR] = toPolar(app, [vx, vy]);
        end

    end % private methods

end % classdef
