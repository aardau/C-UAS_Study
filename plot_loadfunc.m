function plot_loadfunc(loadfunc, grid_mesh_x, grid_mesh_y, figHandle)

figure(figHandle)
hold on

num_loadfunc = loadfunc.number;

xR = zeros(num_loadfunc,1);
yR = zeros(num_loadfunc,1);

gridnx = size(grid_mesh_x);
gridny = size(grid_mesh_y);
loadval = zeros(gridnx);

if( loadfunc.number > 0 )
    for k = 1:loadfunc.number
        xR(k) = loadfunc.location{k}(1);
        yR(k) = loadfunc.location{k}(2);
        for i = 1:gridnx(1)
            for j = 1:gridny(2)
                loadval(i,j) = loadfunc.eval{k}(grid_mesh_x(i, j), grid_mesh_y(i, j));
            end
        end
        contour(grid_mesh_x, grid_mesh_y, loadval, 75);
    end
    plot(xR, yR, 'k*', 'Linewidth', 2, 'MarkerSize', 12)
end

