function plot_grid(grid_map)
    imagesc(grid_map);  % Viser gridet som et bilde med fargekart
    colormap(gray);     % Setter fargene til gråtoner for hindringer og åpne områder
    axis equal;         % Sørger for at rutene er firkantede
end
