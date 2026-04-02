classdef RawDataEventData < event.EventData
    % Carries the [4x3] corner-to-anchor distance matrix for one time step.

    properties
        Distances double
    end

    methods
        function obj = RawDataEventData(distances)
            obj.Distances = distances;
        end
    end

end
