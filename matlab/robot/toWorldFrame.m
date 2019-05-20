function outVector = toWorldFrame(p,phi,inVector)
% Function that takes a state, a list of states or a list of 2D positions
% as input and returns the transformed list in the world frame
% The input states or positions must be supplied as columns of inVector
% (in cas eonly 1 state/position: in column format.

% check if vector of 2D positions or vector of states
adaptAngles = false;
if size(inVector,1) == 3
    adaptAngles = true;
end

% homogenous transform matrix local --> global
Tr          = homTrans(phi,[p(1);p(2)]);

% transform vector
outVector = zeros(size(inVector));
for i = 1:size(inVector,2)
    temp             = Tr*[inVector(1:2,i);1];
    outVector(1:2,i) = temp(1:2);
    if adaptAngles
        outVector(3,i) = outVector(3) - phi;
    end
end