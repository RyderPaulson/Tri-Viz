classdef TrilatEventData < event.EventData
    %TRILATEVENTDATA  Payload for the TriVizStreamer 'TrilateralizedData' event.
    %
    %  Carries the robot's computed position, movement heading, and mean beacon
    %  distances for one time step (real playback or synthetic homing walk).
    %
    %  Properties
    %    Position    [1x2] double  (x, y) in the output coordinate frame
    %    Heading     double        radians, atan2 convention (0 = east, CCW)
    %    BeaconDists [1x3] double  mean distance from each beacon; NaN during homing

    properties
        Position double % [1x2]
        Heading double % scalar radians
        BeaconDists double % [1x3], NaN during homing walk
    end

    methods
        function obj = TrilatEventData(position, heading, beaconDists)
            obj.Position = position;
            if nargin < 2
                obj.Heading = 0;
            else
                obj.Heading = heading;
            end
            if nargin < 3
                obj.BeaconDists = NaN(1, 3);
            else
                obj.BeaconDists = beaconDists;
            end
        end
    end

end
