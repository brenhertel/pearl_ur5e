
for i=1:length(demos)
    eulerangles=[];
    % Position filter (only can work in positive values)
    valMin = min(min(demo_struct(i).ds_pos)); % Value to substract at the end
    disp(['ValMin: ' num2str(valMin)])
    x = (demo_struct(i).ds_pos(1,:) + abs(valMin))*100 + 1;
    y = (demo_struct(i).ds_pos(2,:) + abs(valMin))*100 + 1;
    z = demo_struct(1).ds_pos(3,:) * 100;
    teachPos{i} = [x;y;z];
    % Orientation filter
    for j=1:length(demo_struct(i).ds_rot)
        quat = demo_struct(i).ds_rot(:,j)';
        eulZYX = rad2deg(quat2eul(quat, "XYZ"));
        eulerangles(j, :) = eulZYX;
    end
    for j=1:3
        eulerangles(:, j) = unwrap(eulerangles(:, j));
        if any(eulerangles(:, j) < 0)
            eulerangles(:, j) = eulerangles(:, j) + 360;
        end
    end
    eulerangles = unique(eulerangles,"rows");
    teachOri{i}=(eulerangles'/10) + 1; % To avoid problems and facilitate the speed, we divid by 10 the resolution and ad 1 extra degrre to avoid 0
    visual{i}=eulerangles;
end
