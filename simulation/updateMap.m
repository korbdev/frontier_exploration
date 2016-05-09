%Bresenham for Line Of Sight calculation
function map = updateMap(pose, map, occupancy_map, radius)
    [n,m] = size(map);
    for i = 1:n
        for j = 1:m
            x = pose(1);
            y = pose(2);

            x_end = i;
            y_end = j;

            dx = x_end - x;
            dy = y_end - y;

            distance = sqrt(dx^2+dy^2);
            
            x_step = 1;
            y_step = 1;

            if(dx < 0)
                dx = -dx;
                x_step = -1;
            end

            if(dy < 0)
                dy = -dy;
                y_step = -1;
            end

            a = 2*dx;
            b = 2*dy;

            if(dy <= dx)
                f = -dx;
                    while x ~= x_end
                        if(distance > radius)
                           break; 
                        end
                        if(occupancy_map(x,y) == 1)
                            map(x,y) = 1;
                            break;
                        else
                            map(x,y) = 3;
                        end
                        f = f + b;
                        if(f > 0)
                            y = y + y_step;
                            f = f - a;
                        end
                        x = x + x_step;
                    end
            else
                f = -dy;
                    while y ~= y_end
                         if(distance > radius)
                            break; 
                         end
                         if(occupancy_map(x,y) == 1)
                            map(x,y) = 1;
                            break;
                         else
                            map(x,y) = 3;
                         end
                         f = f + a;
                         if(f > 0)
                            x = x + x_step;
                            f = f - b;
                         end
                         y = y + y_step;
                    end
            end
            map(pose(1), pose(2)) = 4;
        end
    end
end