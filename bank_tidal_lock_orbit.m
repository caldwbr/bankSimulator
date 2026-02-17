function bankOrbitTrue
    %% BANIM Engine - UPRIGHT ORBIT (NO FLIP)
    clear; clc; close all;

    filename = 'bank_orbit_true.mp4';
    v = VideoWriter(filename, 'MPEG-4');
    v.FrameRate = 30;
    open(v);

    % --- CONFIG ---
    n_shells = 18; cube_w = 8.0; sphere_diam = cube_w * 1.3; alpha_base = 0.28;
    glass_r = (cube_w * 0.3) / 2; glass_h = cube_w * 0.7;
    glass_offset_val = cube_w * 0.2; glass_tilt_y = deg2rad(-20);
    orbit_radius = 12.0; n_revs = 2; n_frames = 180;

    fig = figure('Color', 'black', 'Position', [50 50 1200 900]);
    hold on; axis equal; axis off; view(45, 30);
    set(gca, 'Color', 'k', 'Projection', 'perspective');

    % --- PRE-CALC GEOMETRY ---
    [sx, sy, sz] = sphere(40); [gx, gy, gz] = get_broken_glass(glass_r, glass_h); 
    
    % Weld glass to point "Front" (+Y) with tilt
    R_align = [0 -1 0; 1 0 0; 0 0 1]; 
    R_tilt = [cos(glass_tilt_y) 0 sin(glass_tilt_y); 0 1 0; -sin(glass_tilt_y) 0 cos(glass_tilt_y)];
    glass_pts = [gx(:), gy(:), gz(:)] * R_align * R_tilt;
    gx = reshape(glass_pts(:,1), size(gx)) + glass_offset_val;
    gy = reshape(glass_pts(:,2), size(gy));
    gz = reshape(glass_pts(:,3), size(gz));

    fprintf('Rendering bankOrbitTrue...\n');

    try
        for f = 1:n_frames
            cla;
            % 'angle' is defined HERE inside the loop
            angle = (f/n_frames) * 2 * pi * n_revs;
            centroid = [orbit_radius * cos(angle), orbit_radius * sin(angle), 0];
            
            % THE ONE-LINER (Keeps Z upright, turns Left face to center)
            R_lock = [cos(angle + pi/2) -sin(angle + pi/2) 0; sin(angle + pi/2) cos(angle + pi/2) 0; 0 0 1];
            
            for s = linspace(0.3, 1.0, n_shells)
                opacity = alpha_base * (1.2 - (s-0.5)^2);
                col = [0.95 0.95 1];
                
                % Standard Vertices * R' multiplication
                draw_cube_wire(cube_w*s, R_lock, centroid, [1 1 1], opacity);
                draw_manifold(sx*(sphere_diam/2)*s, sy*(sphere_diam/2)*s, sz*(sphere_diam/2)*s, R_lock, centroid, col, opacity*0.6, 'mesh');
                draw_manifold(gx*s, gy*s, gz*s, R_lock, centroid, col, opacity*2.0, 'surf');
            end
            
            plot3(0,0,0, 'w+', 'MarkerSize', 15, 'LineWidth', 2);
            xlim([-20 20]); ylim([-20 20]); zlim([-10 10]);
            drawnow limitrate;
            writeVideo(v, getframe(fig));
        end
    catch ME
        fprintf('Error during render: %s\n', ME.message);
    end

    close(v);
    fprintf('Done. Video saved as %s\n', filename);
end

% --- ENGINES ---
function draw_cube_wire(w, R, pos, col, alp)
    r = w/2;
    v = [-r -r -r;  r -r -r;  r  r -r; -r  r -r;
         -r -r  r;  r -r  r;  r  r  r; -r  r  r];
    edges = [1 2; 2 3; 3 4; 4 1; 5 6; 6 7; 7 8; 8 5; 1 5; 2 6; 3 7; 4 8];
    v_rot = v * R'; 
    for i = 1:12
        p = v_rot(edges(i,:), :) + pos;
        line(p(:,1), p(:,2), p(:,3), 'Color', [col alp], 'LineWidth', 1.8);
    end
end

function draw_manifold(x, y, z, R, pos, col, alp, type)
    pts = [x(:), y(:), z(:)] * R'; 
    rx = reshape(pts(:,1), size(x)) + pos(1);
    ry = reshape(pts(:,2), size(y)) + pos(2);
    rz = reshape(pts(:,3), size(z)) + pos(3);
    if strcmp(type, 'mesh')
        mesh(rx, ry, rz, 'EdgeColor', col, 'FaceColor', 'none', 'EdgeAlpha', alp);
    else
        surf(rx, ry, rz, 'FaceColor', col, 'EdgeColor', col, 'FaceAlpha', alp, 'EdgeAlpha', alp*0.2);
    end
end

function [x, y, z] = get_broken_glass(r, h)
    [x, y, z] = cylinder(r, 60); z = z * h - (h/2); v_d = h * 0.66; theta_vals = linspace(0, 2*pi, 61);
    for i = 1:length(theta_vals)
        th_n = mod(theta_vals(i) + pi, 2*pi) - pi;
        if abs(th_n) < pi/3, z(2,i) = (h/2) - v_d * (1 - abs(th_n)/(pi/3)) + (rand*0.09); end
    end
end
