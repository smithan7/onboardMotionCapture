close all
clear all
clc

redThresh = 0.1;
videoFReader = vision.VideoFileReader('20150619_ft_4_wc_cut.AVI');
videoPlayer = vision.VideoPlayer;
videoFWriter = vision.VideoFileWriter('20150619_ft_1_cut_out.AVI');
hblob = vision.BlobAnalysis('AreaOutputPort', false, ... % Set blob analysis handling
    'CentroidOutputPort', true, ...
    'BoundingBoxOutputPort', true', ...
    'MinimumBlobArea', 100, ...
    'MaximumBlobArea', 3000, ...
    'MaximumCount', 10);
hshapeinsBox = vision.ShapeInserter('BorderColorSource', 'Input port', ... % Set box handling
    'Fill', true, ...
    'FillColorSource', 'Input port', ...
    'Opacity', 0.4);
htextinsRed = vision.TextInserter('Text', 'Red   : %2d', ... % Set text for number of blobs
    'Location',  [5 2], ...
    'Color', [1 0 0], ... // red color
    'Font', 'Courier New', ...
    'FontSize', 14);
htextinsCent = vision.TextInserter('Text','+X:%2d,Y:%2d', ... % set text for centroid
    'LocationSource', 'Input port', ...
    'Color', [0 0 0], ... // black color
    'Font', 'Courier New', ...
    'FontSize', 20);
htextinsCent2 = vision.TextInserter('Text','+X:%4d,Y:%4d', ... % set text for centroid
    'LocationSource', 'Input port', ...
    'Color', [0 0 0], ... // black color
    'Font', 'Courier New', ...
    'FontSize', 20);
y_pix_max = 480;%1080;
x_pix_max = 640;%1920;
x_pix = x_pix_max/2;
y_pix = y_pix_max/2;
hVideoIn = vision.VideoPlayer('Name','Final Video','Position', [0 0 y_pix_max x_pix_max]);


% initialize variables
centRed_prev = [];
nFrame = 1;
frame_rate = 30; % Hz

% initialize kalman filter - x, y, z, and theta kinematic model with an assumed
% constant velocity
phi = [1 1/frame_rate 0 0 0 0 0 0; % x' = x + xd*dt 
       0 1 0 0 0 0 0 0; % xd' = xd
       0 0 1 1/frame_rate 0 0 0 0; % y' = y + yd*dt
       0 0 0 1 0 0 0 0; % yd' = yd
       0 0 0 0 1 1/frame_rate 0 0; % z' = z + zd*dt
       0 0 0 0 0 1 0 0; % zd' = zd
       0 0 0 0 1 1/frame_rate 0 0; % th' = th + thd*dt
       0 0 0 0 0 1 0 0]; % thd' = thd
% can only sense position and orientation
h = [1 0 0 0 0 0 0 0; % x
     0 0 1 0 0 0 0 0; % y
     0 0 0 0 1 0 0 0; % z
     0 0 0 0 0 0 1 0]; % th
r = 0.25;
q = 0.1*eye(8);
pm = eye(8);

% initial guess at the orientation (10,10,2,0)
xm = [10; 0; 10; 0; 2; 0; 0 ; 0];

qLoc = [10,10,2,0]; % initial location

time = 0;
l_mean = 260;
index = 0;
while ~isDone(videoFReader)
    index = index + 1;
    rgbFrame = step(videoFReader);
    % rgbFrame = step(htextinsCent, rgbFrame, [uint16(x_pix) uint16(y_pix)], [uint16(x_pix-6) uint16(y_pix-9)]);
    clear centroidRed bboxRed
    diffFrameRed = imsubtract(rgbFrame(:,:,1), rgb2gray(rgbFrame)); % Get red component of the image
    diffFrameRed = medfilt2(diffFrameRed, [3 3]); % Filter out the noise by using median filter
    binFrameRed = im2bw(diffFrameRed, redThresh); % Convert the image into binary image with the red objects as white
    [centroidRed, bboxRed] = step(hblob, binFrameRed); % Get the centroids and bounding boxes of the red blobs
    
    % this section looks for red disks that are adjacent to each other
    clear l dLoc
    iter = 0;
    l_sum = 0;
    for i=1:length(centroidRed)-1 % go through all centroids of red disks
        for j=i+1:length(centroidRed(:,1)) % against all other centroids of red disks
            lt = sqrt((centroidRed(i,1)-centroidRed(j,1))^2 + (centroidRed(i,2)-centroidRed(j,2))^2); % distance between markers
            if lt < l_mean*1.3 && lt > l_mean * 0.7 % is it possible that the disks are adjacent
                l_sum = l_sum + lt; % used to calc next mean length
                
                iter = iter + 1;
                
                dLoc(iter,1) = abs(centroidRed(i,1)-centroidRed(j,1)); % distance in x
                dLoc(iter,2) = abs(centroidRed(i,2)-centroidRed(j,2)); % distance in y
                
                % this is horribly wrong, should be based off of marker location
                % marker location would require predicting marker
                % location based off of the expected angle and then
                % using the angles between markers and angle to the
                % markers to calculate my actual angle
                
                if dLoc(iter,1) > dLoc(iter,2) % calc the angle of the connection
                    pTheta(iter) = atan2(dLoc(iter,2),dLoc(iter,1));
                else
                    pTheta(iter) = atan2(dLoc(iter,1),dLoc(iter,2));
                end
                
            end
        end
    end
    
    mean(pTheta)*180/pi;
    qLoc(4) = mean(pTheta); % this is the horribly wrong bit!
    l_mean = l_sum/(iter); % used to calculate the height and future dists
    l_m(nFrame) = l_mean;
    
    % used to estimate my location and the locations of each disk
    clear pRed pRed2 qLocP
    for i=1:length(centroidRed)
        cTheta = cos(qLoc(4));
        sTheta = sin(qLoc(4));
        pRed(i,1) = qLoc(1) + ((centroidRed(i,1) - x_pix)/l_mean)*cTheta + ((centroidRed(i,2) - y_pix)/l_mean)*sTheta;
        pRed(i,2) = qLoc(2) - ((centroidRed(i,2) - y_pix)/l_mean)*sTheta + ((centroidRed(i,2) - y_pix)/l_mean)*cTheta;
        pRed2(i,1) = round(pRed(i,1)); % round disk locations
        pRed2(i,2) = round(pRed(i,2)); % round disk locations
        eP(i) = sqrt((pRed(i,1) - pRed2(i,1))^2+(pRed(i,1) - pRed2(i,1))^2);
        qLocP(i,1) = pRed2(i,1) - ((centroidRed(i,1) - x_pix)/l_mean)*cTheta - ((centroidRed(i,2) - y_pix)/l_mean)*sTheta;
        qLocP(i,2) = pRed2(i,2) + ((centroidRed(i,2) - y_pix)/l_mean)*sTheta - ((centroidRed(i,2) - y_pix)/l_mean)*cTheta;
        rgbFrame = step(htextinsCent, rgbFrame, [uint16(pRed(i,1)) uint16(pRed(i,2))], [centroidRed(i,1) centroidRed(i,2)]);
    end
    e_m(nFrame) = mean(eP(:))
    
    %rgbFrame = step(htextinsCent2, rgbFrame, [uint16(mean(qLocP(:,1))*100) uint16(mean(qLocP(:,2))*100)], [x_pix y_pix]);
    
    %% kalman filter
    % compute Kalman gain, figure 4.1
    ka = pm*h'/(h*pm*h'+r);
    % take measurement z(k)
    z = [mean(qLocP(:,1)) mean(qLocP(:,2)) 1/l_mean qLoc(4)]';
    zL(nFrame,1:3) = z(1:3);
    % update estimate with measurement z(k), figure 4.1
    xh = xm+ka*(z-h*xm);
    % Compute error covariance for updated estimate, figure 4.1
    p = (eye(8)-ka*h)*pm;
    % Project ahead, figure 4.1
    xm = phi*xh;
    pm = phi*p*phi'+q;
    p_trace(nFrame) = trace(nFrame);
    % locate corners from current position
    rgbFrame = step(htextinsCent2, rgbFrame, [uint16(xh(1)*100) uint16(xh(3)*100)], [uint16(x_pix) uint16(y_pix)]);

    qLoc(1) = xh(1) + xh(2)*1/frame_rate;
    qLoc(2) = xh(3) + xh(4)*1/frame_rate;
    qLoc(3) = xh(5) + xh(6)*1/frame_rate;
    qLoc(4) = xh(7) + xh(8)*1/frame_rate;

    x(nFrame,1:4) = qLoc(1:4);
    v(nFrame,1) = xh(2);
    v(nFrame,2) = xh(4);
    v(nFrame,3) = xh(6);
    v(nFrame,4) = xh(8);
    
    plot(x(nFrame,1),x(nFrame,2),'rx')
    grid on
    axis([0 20 0 20])

    step(videoFWriter, rgbFrame);
    step(hVideoIn, rgbFrame); % Output video stream
    nFrame = nFrame + 1;
    time(nFrame) = time(nFrame-1) + 1/frame_rate;
end
release(videoPlayer);
release(videoFReader);
release(videoFWriter);
% clear empty variables
time(1) = [];

figure
subplot(4,1,1)
plot(time,x(:,1),'b')
subplot(4,1,2)
plot(time,x(:,2),'b')
subplot(4,1,3)
plot(time,x(:,3),'b')
subplot(4,1,4)
plot(time,x(:,4),'b')
grid on

figure
subplot(4,1,1)
plot(time,v(:,1),'g')
subplot(4,1,2)
plot(time,v(:,2),'g')
subplot(4,1,3)
plot(time,v(:,3),'g')
subplot(4,1,4)
plot(time,v(:,4),'g')
grid on

figure
plot(time,e_m,'r')