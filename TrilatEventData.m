classdef TrilatEventData < event.EventData
    % Carries position, heading, and beacon distances for one time step.

    properties
        Position double
        Heading double
        BeaconDists double
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
