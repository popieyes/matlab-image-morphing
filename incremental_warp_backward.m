function incremental_warp_backward(src_img_path, dst_img_path)
    clc;
    
    % points to click
    points = 3;
    
    % output gifs for warping animation
    gif_path_s_to_t = 'source_to_target_warp.gif';
    gif_path_t_to_s = 'target_to_source_warp.gif'; 
    gif_path_morphed = 'morphed_warp.gif'; 
    
    % loads image 1
    im1_original = imread(src_img_path);
    
    % rescales image 1 to 256x256
    im1(:,:,1) = imresize(im1_original(:,:,1), [256 256]); 
    im1(:,:,2) = imresize(im1_original(:,:,2), [256 256]); 
    im1(:,:,3) = imresize(im1_original(:,:,3), [256 256]); 
    
    % loads image 2 
    im2_original = imread(dst_img_path);
    
    % rescales image 1 to 256x256
    im2(:,:,1) = imresize(im2_original(:,:,1), [256 256]); 
    im2(:,:,2) = imresize(im2_original(:,:,2), [256 256]); 
    im2(:,:,3) = imresize(im2_original(:,:,3), [256 256]);
    
    figure;
    imagesc(im1);
    axis equal;
    hold on;
    [x1,y1] = ginput(points);
    
    % Appends points (1,1), (1,256), (256,1), and (256,256) to the clicked
    % points. This is needed for a Delauny triangulation that affects the
    % entire image
    x1 = vertcat(x1, 1, 1, 256, 256);
    y1 = vertcat(y1, 1, 256, 1, 256);
    fprintf('x1: %.5f , y1: %.5f\n',x1(1), y1(1));
    fprintf('x1: %.5f , y1: %.5f\n',x1(2), y1(2));
    fprintf('x1: %.5f , y1: %.5f\n',x1(3), y1(3));
    hold off;
    
    figure;
    imagesc(im2);
    axis equal;
    hold on;
    [x2,y2] = ginput(points);
    x2 = vertcat(x2, 1, 1, 256, 256);
    y2 = vertcat(y2, 1, 256, 1, 256);
    fprintf('x2: %.5f , y2: %.5f\n',x2(1), y2(1));
    fprintf('x2: %.5f , y2: %.5f\n',x2(2), y2(2));
    fprintf('x2: %.5f , y2: %.5f\n',x2(3), y2(3));
    hold off;
    
    close all

    % Computes triangulation using Delauny at the mid point
    x_mean = (x1 + x2) / 2;
    y_mean = (y1 + y2) / 2;
    triangles = delaunay(x_mean, y_mean);
    
    % Shows triangulation
    figure;
    imagesc(im1);
    axis equal;
    hold on;
    triplot(triangles, x1, y1);
    hold off;
    
    figure;
    imagesc(im2);
    axis equal;
    hold on;
    triplot(triangles, x2, y2);
    hold off;
    
    % we are going to interpolate between im1 and im2 with 0.1 steps
    for t=0:0.1:1
        
        % allocates matrix to store warped images
        warp_src_to_target = zeros(256, 256, 3, 'uint8');
        warp_target_to_src = zeros(256, 256, 3, 'uint8');
      
        num_triangles = size(triangles, 1);
      
        % allocates memory to store the affine transformation per triangle
        affine_transf_src = zeros(3, 3, num_triangles);
        affine_transf_target = zeros(3, 3, num_triangles);
        
        % Find affine transformation of each triangle
        for tri=1:num_triangles
           %
           % !!!!!!!!!!! TO COMPLETE !!!!!!!!!!!!!!
           %
           % !! You must take into account that you need to find TWO transformation
           % !! matrices per triangle:
           % - a transformation to go from im1 to the 
           %   current interpolated point in time (affine_transf_src)
           % - a transformation to go from im2 to the 
           %   current interpolated point in time affine_transf_target
           %
            
           %%

           %% ----- MY CODE ------%%

           % Get a triangle from Delauny
            triangle_id = triangles(tri, :);
    
           % Establish the source position and the target position
           % v_src contains in the first vector the x positions of the triangle
           % points.
           % v_trg contains in the first vector the x positions of the
           % triangle points. 
           % Same goes for the vector 2 but with y positions
            v_src = [x1(triangle_id(1)) x1(triangle_id(2)) x1(triangle_id(3));
                 y1(triangle_id(1)) y1(triangle_id(2)) y1(triangle_id(3));  
                 1                  1                  1                ];

            v_trg = [x2(triangle_id(1)) x2(triangle_id(2)) x2(triangle_id(3));
                 y2(triangle_id(1)) y2(triangle_id(2)) y2(triangle_id(3));  
                 1                  1                  1                ];
            
            % Compute the interpolated position
            v_interp = (1-t) * v_src + (t) * v_trg;
            
            % Backwards transformation to Source Image from Interpolated
            % Position
            affine_transf_src(:, :, tri) = v_src * inv(v_interp);
            % Backwards transformation to Target Image from Interpolated
            % Position
            affine_transf_target(:,:,tri) = v_trg * inv(v_interp);
            %% ----- MY CODE ----- %%
        end
        
        % Compute interpolated x and y
        interpolated_x = (1-t) * x1 + (t) * x2;
        interpolated_y = (1-t) * y1 + (t) * y2;

        for i = 1:size(im1,1)
            for j = 1:size(im1,2)

                tn = tsearchn([interpolated_x interpolated_y], triangles, [i, j]);
                T = affine_transf_src(:, :, tn);
               
                trg_px = [i; j; 1];
             
                src_px = T * trg_px;
                src_px = round(src_px);

                % ----- PIXEL CLAMP 1, 256 ------
                src_px(1) = max(1, min(256, src_px(1)));
                src_px(2) = max(1, min(256, src_px(2)));
                % -------------------------------
                warp_src_to_target(j, i, :) = im1(src_px(2), src_px(1), :);
            end
        end
        
        % for all image pixels of the target image
        for i = 1:size(im2,1)
            for j = 1:size(im2,2)
                
                tn = tsearchn([interpolated_x interpolated_y], triangles, [i, j]);
           
                T = affine_transf_target(:, :, tn);
                trg_px = [i; j; 1];

                src_px = T * trg_px;
                src_px = round(src_px);

                % ----- PIXEL CLAMP 1, 256 ------
                src_px(1) = max(1, min(256, src_px(1)));
                src_px(2) = max(1, min(256, src_px(2)));
              
            
                 warp_target_to_src(j, i, :) = im2(src_px(2), src_px(1), :);

            end
        end
        
        % creates gif of source to target animation
        morphed = warp_src_to_target;
        [A, map] = rgb2ind(morphed, 256);
        if t == 0
            imwrite(A, map, gif_path_s_to_t, 'gif','LoopCount',...
                Inf,'DelayTime',0.1);
        else
            imwrite(A, map, gif_path_s_to_t, 'gif','WriteMode',...
                'append','DelayTime',0.1);
        end
        
        % creates gif of target to source animation
        morphed = warp_target_to_src;
        [A, map] = rgb2ind(morphed, 256);
        if t == 0
            imwrite(A, map, gif_path_t_to_s, 'gif','LoopCount',...
                Inf,'DelayTime',0.1);
        else
            imwrite(A, map, gif_path_t_to_s, 'gif','WriteMode',...
                'append','DelayTime',0.1);
        end
        
        % blends the morphed images and creates a gif
        %
        morphed = warp_src_to_target * (1-t) + (t) * warp_target_to_src;
      
        
        [A, map] = rgb2ind(morphed, 256);
        if t == 0
            imwrite(A, map, gif_path_morphed, 'gif','LoopCount',...
                Inf,'DelayTime',0.1);
        else
            imwrite(A, map, gif_path_morphed, 'gif','WriteMode',...
                'append','DelayTime',0.1);
        end
        
    end
end
