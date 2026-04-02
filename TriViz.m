classdef TriViz < handle
    %TRIVIZ  Trilateralization Visualizer
    %
    %   Visualizes the tri-laterized position of some robot using 3 beacons.
    %   Shows both the output location and has the option to visualizae the
    %   range circles around each beacon showing how the algorithm actually
    %   works.
    %
    % Usage: app = TriViz();

    properties (Access = private)
        % UI
        UIFigure matlab.ui.Figure
        CartAxes matlab.ui.control.UIAxes
        ConnectButton matlab.ui.control.Button
        StatusLabel matlab.ui.control.Label

        % Plot handles
        BeaconScatter
        DrawRange
        BeaconRanges
        BeaconLabels
        RobotMarker
        RobotLabel
        TrailLine

        % Geometry
        BeaconPositions double
        AxesCenter double
        MaxBeaconDist double

        % Streaming
        Connected logical
        Streamer TriVizStreamer
        StreamListeners

        % Trail buffer
        TrailX double
        TrailY double
        MaxTrailLen double
    end

    methods (Access = public)

        function app = TriViz(options)
            arguments
                options.DrawRange = "Off"
            end

            if strcmpi(options.DrawRange, "Off")
                app.DrawRange = false;
            elseif strcmpi(options.DrawRange, "On")
                app.DrawRange = true;
            else
                error("DrawRange must be set to 'on' or 'off'");
            end

            app.Connected = false;
            app.TrailX = zeros(1, 0);
            app.TrailY = zeros(1, 0);
            app.MaxTrailLen = 300;
            app.StreamListeners = [];

            app.Streamer = TriVizStreamer();

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

    methods (Access = private)

        function createComponents(app)
            W = 840; H = 720;

            app.UIFigure = uifigure( ...
                'Name', 'TriViz - Trilateralization Visualizer', ...
                'Position', [80 80 W H], ...
                'Color', [0.95 0.95 0.97], ...
                'Visible', 'off');

            app.UIFigure.CloseRequestFcn = @(~,~) delete(app);

            % Toolbar
            toolbar = uipanel(app.UIFigure, ...
                'Position', [0 H-54 W 54], ...
                'BackgroundColor', [0.87 0.87 0.91], ...
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
                'FontColor', [0.35 0.35 0.40], ...
                'FontSize', 12, ...
                'BackgroundColor', [0.87 0.87 0.91]);

            % Axes
            app.CartAxes = uiaxes(app.UIFigure, ...
                'Units', 'pixels', ...
                'Position', [60 30 W-80 H-90]);

            ax = app.CartAxes;
            ax.Color = [1.00 1.00 1.00];
            ax.XColor = [0.20 0.20 0.25];
            ax.YColor = [0.20 0.20 0.25];
            ax.GridColor = [0.70 0.70 0.75];
            ax.GridAlpha = 0.40;
            ax.MinorGridColor = [0.80 0.80 0.85];
            ax.FontSize = 9;
            ax.DataAspectRatio = [1 1 1];
            ax.Box = 'on';
            grid(ax, 'on');
            xlabel(ax, 'X (m)', 'Color', [0.20 0.20 0.25], 'FontSize', 9);
            ylabel(ax, 'Y (m)', 'Color', [0.20 0.20 0.25], 'FontSize', 9);

            app.UIFigure.Visible = 'on';
        end

        function initGeometry(app)
            app.BeaconPositions = calibrateBeacons(app.Streamer);
            app.AxesCenter = mean(app.BeaconPositions, 1);
        end

        function [th, r] = toPolar(app, xy)
            dx = xy(:,1) - app.AxesCenter(1);
            dy = xy(:,2) - app.AxesCenter(2);
            [th, r] = cart2pol(dx, dy);
        end

        function drawStaticElements(app)
            ax = app.CartAxes;
            hold(ax, 'on');

            beaconColor = [0.055 0.514 0.361];
            robotColor = [0.055 0.514 0.361];
            trailColor = [0.60 0.60 0.65];

            dists = sqrt(sum((app.BeaconPositions - app.AxesCenter).^2, 2));
            app.MaxBeaconDist = max(dists);
            setAxisLimits(app, app.MaxBeaconDist * 2);

            bX = app.BeaconPositions(:, 1);
            bY = app.BeaconPositions(:, 2);

            app.BeaconScatter = scatter(ax, bX, bY, 230, beaconColor, 'o', ...
                'LineWidth', 2);

            app.BeaconLabels = gobjects(1, 3);
            for i = 1:3
                app.BeaconLabels(i) = text(ax, bX(i), bY(i), ...
                    sprintf('%d', i), ...
                    'Color', beaconColor, ...
                    'FontWeight', 'bold', ...
                    'FontSize', 10, ...
                    'HorizontalAlignment', 'center');
            end

            drawRangeCircles(app, NaN(1, 3));

            app.TrailLine = plot(ax, NaN, NaN, ...
                'Color', trailColor, ...
                'LineWidth', 0.9);

            app.RobotMarker = plot(ax, NaN(4,1), NaN(4,1), ...
                'Color', robotColor, ...
                'LineWidth', 2.5);

            app.RobotLabel = text(ax, 0, 0, 'Robot', ...
                'Color', robotColor, ...
                'FontWeight', 'bold', ...
                'FontSize', 9, ...
                'HorizontalAlignment', 'center', ...
                'Visible', 'off');
        end

        % Connection

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
            app.StatusLabel.FontColor = [0.75 0.45 0.00];

            app.StreamListeners = addlistener(app.Streamer, ...
                'TrilateralizedData', ...
                @(~, e) updateDisplay(app, e.Position, e.Heading, e.BeaconDists));

            app.Streamer.connect();

            app.StatusLabel.Text = 'Status: Streaming';
            app.StatusLabel.FontColor = [0.02 0.50 0.25];
        end

        function disconnectStream(app)
            app.Streamer.disconnect();

            if ~isempty(app.StreamListeners)
                delete(app.StreamListeners);
                app.StreamListeners = [];
            end

            app.Connected = false;
            app.ConnectButton.Text = 'Connect';
            app.ConnectButton.BackgroundColor = [0.055 0.514 0.361];
            app.StatusLabel.Text = 'Status: Disconnected';
            app.StatusLabel.FontColor = [0.35 0.35 0.40];
        end

        % Display

        function updateDisplay(app, robotXY, heading, beaconDists)
            [vX, vY] = robotTriVerts(app, robotXY, heading);
            app.RobotMarker.XData = vX;
            app.RobotMarker.YData = vY;

            % Trail
            app.TrailX(end+1) = robotXY(1);
            app.TrailY(end+1) = robotXY(2);
            if numel(app.TrailX) > app.MaxTrailLen
                keep = numel(app.TrailX) - app.MaxTrailLen + 1;
                app.TrailX = app.TrailX(keep:end);
                app.TrailY = app.TrailY(keep:end);
            end
            app.TrailLine.XData = app.TrailX;
            app.TrailLine.YData = app.TrailY;

            % Range circles
            geomDists = sqrt(sum((app.BeaconPositions - robotXY).^2, 2))';
            drawRangeCircles(app, geomDists);

            % Auto-scale
            trailDists = sqrt((app.TrailX - app.AxesCenter(1)).^2 + ...
                              (app.TrailY - app.AxesCenter(2)).^2);
            setAxisLimits(app, max(app.MaxBeaconDist * 2, max(trailDists) * 1.10));

            % Status bar
            [rTh, rR] = toPolar(app, robotXY);
            app.StatusLabel.Text = sprintf( ...
                'Status: Streaming  |  Angle: %+06.1f\x00B0  |  Distance: %.3f m', ...
                rad2deg(rTh), rR);
        end

        function drawRangeCircles(app, dists)
            if ~app.DrawRange, return; end

            ax = app.CartAxes;
            rangeColor = [0.60 0.60 0.65];
            bX = app.BeaconPositions(:, 1);
            bY = app.BeaconPositions(:, 2);
            theta = linspace(0, 2*pi, 100);

            for i = 1:3
                if isnan(dists(i))
                    circX = NaN(1, 100);
                    circY = NaN(1, 100);
                else
                    circX = bX(i) + dists(i) * cos(theta);
                    circY = bY(i) + dists(i) * sin(theta);
                end

                if numel(app.BeaconRanges) < i || isempty(app.BeaconRanges{i}) || ~isgraphics(app.BeaconRanges{i})
                    app.BeaconRanges{i} = plot(ax, circX, circY, ...
                        'Color', rangeColor, ...
                        'LineWidth', 0.9, ...
                        'LineStyle', '--');
                else
                    h = app.BeaconRanges{i};
                    h.XData = circX;
                    h.YData = circY;
                end
                app.BeaconRanges{i}.Color(4) = 0.3;
            end
        end

        function setAxisLimits(app, halfSpan)
            ax = app.CartAxes;
            cx = app.AxesCenter(1);
            cy = app.AxesCenter(2);

            newXLim = [cx - halfSpan, cx + halfSpan];
            newYLim = [cy - halfSpan, cy + halfSpan];

            if max(abs(ax.XLim - newXLim)) < halfSpan * 1e-4 && ...
               max(abs(ax.YLim - newYLim)) < halfSpan * 1e-4
                return
            end

            ax.XLim = newXLim;
            ax.YLim = newYLim;
        end

        function [vX, vY] = robotTriVerts(app, centerXY, heading)
            s = norm(app.BeaconPositions(1,:) - app.AxesCenter) * 0.056;
            va = heading + [0; 2*pi/3; 4*pi/3; 0];
            vX = centerXY(1) + s * cos(va);
            vY = centerXY(2) + s * sin(va);
        end

    end

end
