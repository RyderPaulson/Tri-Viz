classdef RawDataEventData < event.EventData
    %RAWDATAEVENTDATA  Payload for the TriVizStreamer 'RawData' event.
    %
    %  Carries the corner-to-anchor distance matrix for one time step.
    %
    %  Properties
    %  ----------
    %    Distances  [4x3] double
    %               Row i = corner i, columns = [Anchor1, Anchor2, Anchor3]

    properties
        Distances   double   % [4x3]
    end

    methods
        function obj = RawDataEventData(distances)
            obj.Distances = distances;
        end
    end

end
