function visualize_workspace(csv_paths, titles)
% VISUALIZE_WORKSPACE  3D scatter of the arm's Monte Carlo-sampled
% reachable end-effector workspace (see ../docs/workspace_identification.md).
%
% visualize_workspace({'workspace_points_50k.csv','workspace_points_200k.csv'}, ...
%                      {'N = 50k','N = 200k'})
%
% Each CSV has a header row "x,y,z" (meters, base_link frame), one row per
% sampled configuration's end_effector position -- generated independently
% from arm_interface/urdf/arm.urdf (same joint chain as
% docs/fk_reference.py / the C++ ForwardKinematicsSolver), not read from a
% live ROS topic, so this runs standalone, no ROS/catkin needed.
%
% Points are colored by height (Z) with a sequential colormap -- the
% mapping is by magnitude (low to high), not by identity, matching the
% same convention used in the rest of this analysis. A wireframe sphere at
% the independent geometric reach bound (see docs/workspace_identification.md:
% sum of the chain's own segment lengths, base_link->end_effector) is drawn
% for scale -- no sample can fall outside it, by the triangle inequality on
% a chain of rigid segments, regardless of joint limits.

if nargin < 1
    csv_paths = {'workspace_points_50k.csv', 'workspace_points_200k.csv'};
end
if nargin < 2
    titles = repmat({''}, size(csv_paths));
end

CHAIN_REACH_BOUND_M = 1.281;  % see docs/workspace_identification.md

n_sets = numel(csv_paths);
figure('Name', 'Reachable workspace', 'Color', 'w', ...
       'Position', [100, 100, 560 * n_sets, 520]);

for i = 1:n_sets
    T = readtable(csv_paths{i});
    x = T.x; y = T.y; z = T.z;
    r = sqrt(x.^2 + y.^2 + z.^2);

    subplot(1, n_sets, i);
    scatter3(x, y, z, 4, z, 'filled', 'MarkerFaceAlpha', 0.5);
    hold on;
    % A dense cloud like this one is largely opaque from outside (the
    % near-side points occlude the far side/interior), so the bound sphere
    % mostly only shows where it pokes past the cloud's edge -- confirmed
    % headless that it needs a fairly strong EdgeAlpha to read at all
    % against that, unlike a sparser scatter where a faint line would do.
    plot_wireframe_sphere(CHAIN_REACH_BOUND_M, [0.15 0.15 0.15], 0.45);
    hold off;

    colormap(gca, 'parula');  % sequential: one hue ramp, magnitude-encoded
    cb = colorbar;
    cb.Label.String = 'height z (m)';

    axis equal vis3d;
    grid on;
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    view(135, 20);

    n_pts = numel(x);
    subtitle_str = sprintf('N = %s samples  |  reach [%.3f, %.3f] m', ...
                            format_count(n_pts), min(r), max(r));
    if ~isempty(titles{i})
        title({titles{i}, subtitle_str}, 'FontWeight', 'normal');
    else
        title(subtitle_str, 'FontWeight', 'normal');
    end

    fprintf('%s: N=%d  reach=[%.4f, %.4f] m  bbox x=[%.3f,%.3f] y=[%.3f,%.3f] z=[%.3f,%.3f]\n', ...
            csv_paths{i}, n_pts, min(r), max(r), ...
            min(x), max(x), min(y), max(y), min(z), max(z));
end

end

function plot_wireframe_sphere(radius, rgb, alpha)
% mesh(), not surf()+FaceColor='none': under software OpenGL (no GPU, as
% in a headless check) a surf whose face color is cleared after creation
% can still rasterize as a filled, lit surface that hides the scatter
% cloud underneath it -- confirmed the hard way rendering this script
% headless. mesh() defaults to wireframe-only with no such fallback path.
[sx, sy, sz] = sphere(24);
h = mesh(radius * sx, radius * sy, radius * sz);
set(h, 'FaceColor', 'none', 'EdgeColor', rgb, 'EdgeAlpha', alpha, 'LineWidth', 0.5);
end

function s = format_count(n)
if n >= 1000
    s = sprintf('%dk', round(n / 1000));
else
    s = sprintf('%d', n);
end
end
