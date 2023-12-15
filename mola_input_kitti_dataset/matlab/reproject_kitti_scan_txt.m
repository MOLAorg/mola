function [proj_y] = reproject_kitti_scan_txt(inputTxtFile)
D=load(inputTxtFile);

% Based on: 
% https://github.com/TixiaoShan/LIO-SAM/blob/master/config/doc/kitti2bag/kitti2bag.py

% (JLBC) Note that this code assumes scan deskew has not been already
% applied!

% depth = np.linalg.norm(scan, 2, axis=1)
% pitch = np.arcsin(scan[:, 2] / depth) # arcsin(z, depth)
% fov_down = -24.8 / 180.0 * np.pi
% fov = (abs(-24.8) + abs(2.0)) / 180.0 * np.pi
% proj_y = (pitch + abs(fov_down)) / fov  # in [0.0, 1.0]
% proj_y *= 64  # in [0.0, H]
% proj_y = np.floor(proj_y)
% proj_y = np.minimum(64 - 1, proj_y)
% proj_y = np.maximum(0, proj_y).astype(np.int32)  # in [0,H-1]
% proj_y = proj_y.reshape(-1, 1)
% scan = np.concatenate((scan,proj_y), axis=1)
% scan = scan.tolist()
% for i in range(len(scan)):
%     scan[i][-1] = int(scan[i][-1])

depth = sqrt(D(:,1).^2 + D(:,2).^2);  % (x,y) only
pitch = asin(D(:,3) ./ depth);

fov_down = -24.8 / 180.0 * pi;
fov = (abs(-24.8) + abs(2.0)) / 180.0 * pi;
proj_y = (pitch + abs(fov_down)) / fov;  % in [0.0, 1.0]
proj_y = proj_y * 64;  % in [0.0, H]
proj_y = floor(proj_y);
proj_y = min(proj_y,63);
proj_y = max(proj_y,0);

%plot(proj_y,'.');

end

