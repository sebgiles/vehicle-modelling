function [] = tile(x, Y)
[~,n] = size(Y);
axes = gobjects(n,1);
for i=1:n
    axes(i) = subplot(n,1,i);
    plot(x, Y(:,i));
    legend
end
linkaxes(axes, 'x');
