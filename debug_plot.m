function debug_plot(X, GT, T, NU, fig1, fig2, fig3)
    ed = max(size(X,2), 1);
    st = max(ed - 30, 1);
    
    if size(X, 2) < 1
        return;
    end

    change_figure(fig1);
    clf();

    scatter3(X(1,st:ed), X(2,st:ed), X(3,st:ed), 'MarkerEdgeColor', 'blue')
    hold on;
    plot3(X(1,st:ed), X(2,st:ed), X(3,st:ed), 'Color', 'blue')
    
    scatter3(T(1,st:ed), T(2,st:ed), T(3,st:ed), 'MarkerEdgeColor', 'red')
    plot3(T(1,st:ed), T(2,st:ed),  T(3,st:ed), 'Color', 'red')
    
    scatter3(GT(1,st:ed), GT(2,st:ed), GT(3,st:ed), 'MarkerEdgeColor', 'green')
    plot3(GT(1,st:ed), GT(2,st:ed), GT(3,st:ed), 'Color', 'green')
    
    title('Position');
    
    change_figure(fig2);
    clf();

    scatter3(X(8,st:ed), X(9,st:ed), X(10,st:ed), 'MarkerEdgeColor', 'red')
    hold on;
    plot3(X(8,st:ed), X(9,st:ed), X(10,st:ed), 'Color', 'red')
    
    title('Velocity');
    
    % Plot the position
    %{
    change_figure(fig1);
    clf();
    scatter3(X(1,:), X(2,:), X(3,:), 'MarkerEdgeColor', 'red')
    hold on;
    plot3(X(1,:), X(2,:), X(3,:), 'Color', 'red')

    scatter3(T(1,:), T(2,:), T(3,:), 'MarkerEdgeColor', 'blue')
    plot3(T(1,:), T(2,:), T(3,:), 'Color', 'blue')
    title('Position');
    
    change_figure(fig2)
    clf();
    Xrot = quat_to_rotvec(X(4:7,:)')';
    Trot = quat_to_rotvec(T(4:7,:)')';

    scatter3(Xrot(1, :), Xrot(2,:), Xrot(3,:), 'MarkerEdgeColor', 'red')
    hold on;
    plot3(Xrot(1, :), Xrot(2,:), Xrot(3,:), 'Color', 'red')

    scatter3(Trot(1, :), Trot(2,:), Trot(3,:), 'MarkerEdgeColor', 'blue')
    plot3(Trot(1, :), Trot(2,:), Trot(3,:), 'Color', 'blue')

    title('Orientation (Rot. Vector)');

    change_figure(fig3);
    clf();
    
    scatter3(X(8, :), X(9,:), X(10,:), 'MarkerEdgeColor', 'red')
    hold on;
    plot3(X(8, :), X(9,:), X(10,:), 'Color', 'red')

    scatter3(T(8, :), T(9,:), T(10,:), 'MarkerEdgeColor', 'blue')
    plot3(T(8, :), T(9,:), T(10,:), 'Color', 'blue')
    
    title('Velocity');
    
    change_figure(fig4);
    clf();
    
    scatter3(X(11, :), X(12,:), X(13,:), 'MarkerEdgeColor', 'red')
    hold on;
    plot3(X(11, :), X(12,:), X(13,:), 'Color', 'red')

    scatter3(T(11, :), T(12,:), T(13,:), 'MarkerEdgeColor', 'blue')
    plot3(T(11, :), T(12,:), T(13,:), 'Color', 'blue')

    title('Angular Velocity (Rot. Vector)');
    
    change_figure(fig5);
    clf();

    scatter3(NU(1,:), NU(2,:), NU(3,:), 'MarkerEdgeColor', 'red')
    hold on;
    plot3(NU(1,:), NU(2,:), NU(3,:), 'Color', 'red')

    title('Position nu');
    %}
end

function change_figure(h)
    set(0,'CurrentFigure',h)
end