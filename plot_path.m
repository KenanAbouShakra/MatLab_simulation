function plot_path(path)
    hold on;
    plot(path(:, 1), path(:, 2), 'r', 'LineWidth', 2);
    hold off;
end