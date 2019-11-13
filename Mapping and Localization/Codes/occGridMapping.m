% Robotics: Estimation and Learning


function myMap = occGridMapping(ranges, scanAngles, pose, param)
myResol = param.resol;
myMap = zeros(param.size);
myorigin = param.origin;
lo_occ = param.lo_occ;
lo_free = param.lo_free;
lo_max = param.lo_max;
lo_min = param.lo_min;
N = size(pose,2);
lidar = size(scanAngles,1);

for j = 1:N % for each time,
    
    % Find grids hit by the rays (in the gird map coordinate)
    local_occ = [ranges(:,j).*cos(scanAngles + pose(3,j)), -ranges(:,j).*sin(scanAngles + pose(3,j))]+[pose(1,j),pose(2,j)] ;
    grid_occ = ceil(myResol .* local_occ);
    orig = ceil(myResol .* [pose(1,j),pose(2,j)]);
    % Find occupied-measurement cells and free-measurement cells
    
    
    for i=1: lidar
        
        [freex, freey] = bresenham(orig(1),orig(2),grid_occ(i,1),grid_occ(i,2));
        % convert to 1d index
        free = sub2ind(size(myMap),freey+myorigin(2),freex+myorigin(1));
        occ = sub2ind(size(myMap),( grid_occ(i,2)+myorigin(2) ),( grid_occ(i,1)+myorigin(1) ) );
        % set end point value
        
        % set free cell values
        myMap(free) = myMap(free)-param.lo_free;
        myMap(occ) = myMap(occ)+param.lo_occ;
        
        
        % Update the log-odds
        
        % Saturate the log-odd values
       
       
        
        
    end
  
    
end
 myMap(myMap < lo_min) = lo_min;
 myMap(myMap > lo_max) = lo_max;
        
end

