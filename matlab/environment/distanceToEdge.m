function [I,d] = distanceToEdge(p1,p2,v1,v2)
% Function that returns the distance measured by the rangefinder to an
% edge. The rangefinder beam and the edge are both represented by vectors.
% If these vectors do not intersect, the method returns I=0 and d=0. If
% they do intersect, the method returns I=1 and the distance d.
% p1,v1: for edge
% p2,v2: for rangefinder
% For details about calculations, see handwritten notes.

I = (norm(cross(v1,v2))>1e-4);

if (I == 1)
    c1 = cross((p2-p1),v1);
    c2 = cross((p2-p1),v2);
    c3 = cross(v1,v2);
    u = (c1(3))/c3(3);
    t = -(c2(3))/c3(3);
    if ((0<=u) && (1>=u) && (0<=t) && (1>=t))
        d = u*norm(v2);
    else
        I = 0;
        d = 0;
    end
else
    d = 0;
end

end

