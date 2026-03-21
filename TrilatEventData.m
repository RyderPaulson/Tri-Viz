classdef TrilatEventData < event.EventData
    %TRILATEVENTDATA  Payload for the TriVizStreamer 'TrilateralizedData' event.
    %
    %  Carries the robot's computed position and movement heading for one
    %  time step (real playback or synthetic homing walk).
    %
    %  Properties
    %    Position  [1x2] double  –  (x, y) in the output coordinate frame
    %    Heading   double        –  radians, atan2 convention (0 = east, CCW)

    properties
        Position   double   % [1x2]
        Heading    double   % scalar radians
    end

    methods
        function obj = TrilatEventData(position, heading)
            obj.Position = position;
            if nargin < 2
                obj.Heading = 0;
            else
                obj.Heading = heading;
            end
        end
    end

end
