function bezierPath = bezierCurvePath(waypoints)
    % This function generates a Bezier curve path from given 2D waypoints and plots it
    % with direction vectors at selected points along the curve.

    param.step = 0.1;
    param.offset = 3.0; % Offset used to calculate control points

    [numPoints, ~] = size(waypoints);
    
    bezierPath = [];
    for i = 1:numPoints - 1
        % Generate a segment for each pair of consecutive waypoints
        startPt = waypoints(i, :);
        endPt = waypoints(i + 1, :);
        segmentPath = bezierSegment2D(startPt, endPt, param); % Updated function for 2D
        bezierPath = [bezierPath; segmentPath];
    end

    % Plot Bezier curve
    figure;
    plot(bezierPath(:, 1), bezierPath(:, 2), 'b-', 'LineWidth', 2);
    hold on;
    title('Bezier Curve with Direction Vectors');
    xlabel('X (m)');
    ylabel('Y (m)');
    grid on;

    % Calculate and plot direction vectors
    dx = diff(bezierPath(:, 1));
    dy = diff(bezierPath(:, 2));
    % Append zeros to dx and dy to match the length of bezierPath
    dx = [dx; dx(end)];
    dy = [dy; dy(end)];
    
    % Select only 10 vectors at equal intervals
    numVectors = 10;
    pathLength = length(bezierPath);
    idx = round(linspace(1, pathLength, numVectors)); % Get 10 equally spaced indices

    quiver(bezierPath(idx, 1), bezierPath(idx, 2), dx(idx), dy(idx), 0.5, 'r', 'LineWidth', 2);

    % Ensure that no second curve is plotted
    hold off;
end

function segmentPath = bezierSegment2D(startPt, endPt, param)
    % Generate a Bezier curve segment between start and end points
    nPoints = hypot(startPt(1) - endPt(1), startPt(2) - endPt(2)) / param.step;
    
    % Compute control points for 2D (ignoring angles)
    controlPoints = calculateControlPoints2D(startPt, endPt, param);
    
    % Generate the Bezier curve segment
    segmentPath = [];
    for t = linspace(0, 1, nPoints)
        pointOnCurve = bezier2D(t, controlPoints);
        segmentPath = [segmentPath; pointOnCurve];
    end
end

function controlPoints = calculateControlPoints2D(startPt, endPt, param)
    % Calculate control points for Bezier curve in 2D
    sx = startPt(1); sy = startPt(2);
    gx = endPt(1); gy = endPt(2);
    
    d = hypot(sx - gx, sy - gy) / param.offset;
    controlPoints = [
        sx, sy;
        sx + d, sy;
        gx - d, gy;
        gx, gy
    ];
end

function pointOnCurve = bezier2D(t, controlPoints)
    % Evaluate the Bezier curve at parameter t for 2D points
    n = size(controlPoints, 1) - 1;
    pointOnCurve = [0, 0];
    
    for i = 0:n
        binomialCoeff = nchoosek(n, i);
        pointOnCurve = pointOnCurve + binomialCoeff * (t^i) * ((1 - t)^(n - i)) * controlPoints(i + 1, :);
    end
end
