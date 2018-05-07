function [] = math(eqn)

% Make a figure with only a text object for rendering an equation with
% latex.

% Make the figure (with minimal junk)
fig = figure('Name', 'Equation Viewer', ...
    'NumberTitle', 'off', ...
    'MenuBar', 'none', ...
    'ToolBar', 'none', ...
    'Units', 'normalized', ...
    'Position', [0 0 1 1], ...
    'Color', [1, 1, 1]);
% Expand Axes to completely fill the figure (this is just a
% canvas for a text box, so we don't care about axes label space)
axes('position', [0, 0, 1, 1], 'Units', 'Normalized');

% Hide the axes
axis off

% Make the textbox with latex equation
htext = text(0.5, 0.5, ['$' latex(sym(eqn)) '$'], ...
    'interpreter','latex', ...
    'Units', 'normalized', ...
    'FontSize', 20, ...
    'HorizontalAlignment', 'center');

% Scale up the equation font to fill the figure
%fit_text_to_figure(htext)

% Eliminate the dead space in the figure by pulling it in to match the
% equation dimensions
fit_figure_to_text(fig, htext)

end

function fit_text_to_figure(htext)
% Scales a text object's FontSize until it fills 95% of it's parent
    
    % Temporarily set the units to Normalized
    ax = htext.Parent;
    units = ax.Units;
    ax.Units = 'Normalized';
    
    while max(htext.Extent) < 0.95
        % Use ratio of current size to parent to scale up the font quickly
        size = max(htext.Extent);
        scale = 0.95 / size;
        htext.FontSize = htext.FontSize * scale;
    end
    
    % Reset the axes units to avoid causing side-effects
    ax.Units = units;
end

function fit_figure_to_text(fig, htext)
% Scale down the dimension of a figure that is creating dead space relative
% to the text box
    
    % Get the textbox dimensions
    pos_text = htext.Extent;
    pos = fig.Position;

    w = pos_text(3);
    h = pos_text(4);
    
    fig.Position = [(1-w)/2, (1-h)/2, w, h];

    %movegui(fig, 'center');
end
