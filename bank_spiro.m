function bank
    %% BANIM Engine v2.2 - Unified Spectral White & Calibrated Orientation
    clear; clc; close all;

    % --- VIDEO SETUP ---
    filename = 'bank_simulation_white.mp4';
    v = VideoWriter(filename, 'MPEG-4');
    v.FrameRate = 30;
    open(v);

    % --- CONFIGURATION ---
    n_shells = 18;           
    cube_w = 8.0;            
    sphere_diam = cube_w * 1.3; 
    alpha_base = 0.28;       
    
    % Dimensions and Orientation
    glass_r = (cube_w * 0.3) / 2;
    glass_h = cube_w * 0.7;
    glass_offset_val = cube_w * 0.2; 
    glass_tilt_y = deg2rad(-20);     

    orbit_radius = 10.0;     
    n_revs = 2;              
    n_frames = 120;

    fig = figure('Color', 'black', 'Position', [50 50 1200 900]);
    hold on; axis equal; axis off; 
    set(gca, 'Color', 'k', 'Projection', 'perspective');
    view(45, 30);

    % --- PRE-CALCULATE INVARIANTS ---
    [sx, sy, sz] = sphere(40); 
    [gx, gy, gz] = get_broken_glass(glass_r, glass_h); 

    % --- WELDING & ALIGNMENT ---
    % CCW 90 deg spin (Z-axis) to face the break FRONT
    R_align = [cos(pi/2) -sin(pi/2) 0; sin(pi/2) cos(pi/2) 0; 0 0 1];
    % -20 deg Tilt (Y-axis)
    R_tilt_y = [cos(glass_tilt_y) 0 sin(glass_tilt_y); 0 1 0; -sin(glass_tilt_y) 0 cos(glass_tilt_y)];
    
    glass_pts = [gx(:), gy(:), gz(:)] * R_align * R_tilt_y;
    
    gx = reshape(glass_pts(:,1), size(gx)) + glass_offset_val;
    gy = reshape(glass_pts(:,2), size(gy));
    gz = reshape(glass_pts(:,3), size(gz));

    fprintf('Rendering Unified White Bank to %s...\n', filename);

    try
        for f = 1:n_frames
            cla;
            angle = (f/n_frames) * 2 * pi * n_revs;
            centroid = [orbit_radius * cos(angle), orbit_radius * sin(angle), 0];
            roll_angle = angle * (orbit_radius / (cube_w/2));
            R_roll = [cos(roll_angle) -sin(roll_angle) 0; sin(roll_angle)  cos(roll_angle) 0; 0 0 1];
            
            scales = linspace(0.3, 1.0, n_shells);
            for s = scales
                opacity = alpha_base * (1.2 - (s-0.5)^2);
                % Unified Color Palette: Whitish/Blue Spectral
                cube_col = [1 1 1];
                sphere_col = [0.9 0.95 1];
                glass_col = [0.95 0.95 1]; 
                
                % 1. THE CUBE
                draw_cube_wire(cube_w*s, R_roll, centroid, cube_col, opacity);
                
                % 2. THE SPHERE (1.3x Poking)
                draw_manifold(sx*(sphere_diam/2)*s, sy*(sphere_diam/2)*s, sz*(sphere_diam/2)*s, R_roll, centroid, sphere_col, opacity*0.6, 'mesh');
                
                % 3. THE BROKEN GLASS (Unified Translucent White)
                draw_manifold(gx*s, gy*s, gz*s, R_roll, centroid, glass_col, opacity*2.0, 'surf');
            end
            
            xlim([-18 18]); ylim([-18 18]); zlim([-10 10]);
            drawnow limitrate;
            writeVideo(v, getframe(fig));
        end
    catch ME
        fprintf('Error: %s\n', ME.message);
    end

    close(v);
    fprintf('Success! Unified White Video saved.\n');
end

% --- GEOMETRY ENGINES ---

function draw_cube_wire(w, R, pos, col, alp)
    r = w/2;
    v = [-r -r -r;  r -r -r;  r  r -r; -r  r -r;
         -r -r  r;  r -r  r;  r  r  r; -r  r  r];
    edges = [1 2; 2 3; 3 4; 4 1; 5 6; 6 7; 7 8; 8 5; 1 5; 2 6; 3 7; 4 8];
    v_rot = v * R;
    for i = 1:12
        p = v_rot(edges(i,:), :) + pos;
        line(p(:,1), p(:,2), p(:,3), 'Color', [col alp], 'LineWidth', 1.8);
    end
end

function draw_manifold(x, y, z, R, pos, col, alp, type)
    pts = [x(:), y(:), z(:)] * R;
    rx = reshape(pts(:,1), size(x)) + pos(1);
    ry = reshape(pts(:,2), size(y)) + pos(2);
    rz = reshape(pts(:,3), size(z)) + pos(3);
    if strcmp(type, 'mesh')
        mesh(rx, ry, rz, 'EdgeColor', col, 'FaceColor', 'none', 'EdgeAlpha', alp, 'LineWidth', 0.5);
    else
        % Surface manifold now uses the same EdgeAlpha logic for a "Ghostly" shell
        surf(rx, ry, rz, 'FaceColor', col, 'EdgeColor', col, 'FaceAlpha', alp, 'EdgeAlpha', alp*0.2);
    end
end

function [x, y, z] = get_broken_glass(r, h)
    [x, y, z] = cylinder(r, 60);
    z = z * h - (h/2);
    v_depth = h * 0.66;
    theta = linspace(0, 2*pi, 61);
    for i = 1:length(theta)
        th_norm = mod(theta(i) + pi, 2*pi) - pi;
        if abs(th_norm) < pi/3
            z(2,i) = (h/2) - v_depth * (1 - abs(th_norm)/(pi/3)) + (rand*0.09);
        end
    end
end
