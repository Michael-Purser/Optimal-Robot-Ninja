function measNew = processMeas(meas,gridDx)

% Function that takes the measurements and adds them to an imaginary grid
% followig a simple flooring procedure

measNew = zeros(size(meas));
for i=1:size(meas,1)
    % get grid coordinates
    measNew(i,1) = gridDx*floor(meas(i,1)/gridDx);
    measNew(i,2) = gridDx*floor(meas(i,2)/gridDx);
end

% remove duplicates
measNew = unique(measNew,'rows');

end