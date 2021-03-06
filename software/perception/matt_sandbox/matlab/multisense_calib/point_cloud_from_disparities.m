function pts = point_cloud_from_disparities(disparities, K, baseline, depth_range, use_median)

if (~exist('use_median','var'))
    use_median = false;
end

stack = cat(3,disparities.img);
good = stack>0;
count_img = sum(good,3);
stack(~good) = 0;
good_ind = count_img>0;
if (use_median)
    disparity_img = zeros(size(count_img));
    for i = 1:size(stack,1)
        for j = 1:size(stack,2)
            disparity_img(i,j) = median(squeeze(stack(i,j,good(i,j,:))));
        end
    end
else
    sum_img = sum(stack,3);
    disparity_img = sum_img./count_img;
end


% form depths and mask for calibration object
% TODO: can zero out smooth runs shorter than certain length for speedup
%  (in both rows and cols?)
max_depth_delta = 0.02;
min_region_size = 10000;
depths = baseline*K(1,1)./disparity_img;
labels = find_connected_regions(depths,max_depth_delta,min_region_size);
ctr = round(size(depths)/2);
patch = labels(ctr(1)-20:ctr(1)+20, ctr(2)-20:ctr(2)+20);
good_label = round(median(patch(:)));
mask = labels==good_label;
%mask = imread('~/mask-02.gif')>0;
depths(~mask) = 0;

% unproject points
good_ind = good_ind & depths>=depth_range(1) & depths<=depth_range(2);
[x,y] = meshgrid(0:size(depths,2)-1,0:size(depths,1)-1);
rays = [x(good_ind),y(good_ind),ones(sum(good_ind(:)),1)]*inv(K)';
pts = rays.*repmat(depths(good_ind)./rays(:,3),[1,3]);
