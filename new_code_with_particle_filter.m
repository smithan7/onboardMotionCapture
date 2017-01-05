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
markerSpacing = 1;
time = 0;
l_mean = 190;


x = [10.4; 0; 10.3; 0; 2; 0; 0 ; 0]; % initial guess at the orientation (10,10,2,0)
nParticles = 100;

qLoc(1) = x(1);
qLoc(2) = x(3);
qLoc(3) = x(5);
qLoc(4) = x(7);

R_translate = 0.05;
R_altitude = 0.05;
R_theta = 0.0001;
R_total = 2*R_translate + R_altitude + R_theta;

index = 1;
while ~isDone(videoFReader)
    
    if index > 146
        waitforbuttonpress
    end
    
    % this sections finds the red disks
    index = index + 1;
    rgbFrame = step(videoFReader);
    % rgbFrame = step(htextinsCent, rgbFrame, [uint16(x_pix) uint16(y_pix)], [uint16(x_pix-6) uint16(y_pix-9)]);
    clear centroidRed bboxRed
    diffFrameRed = imsubtract(rgbFrame(:,:,1), rgb2gray(rgbFrame)); % Get red component of the image
    diffFrameRed = medfilt2(diffFrameRed, [3 3]); % Filter out the noise by using median filter
    binFrameRed = im2bw(diffFrameRed, redThresh); % Convert the image into binary image with the red objects as white
    [centroidRed, bboxRed] = step(hblob, binFrameRed); % Get the centroids and bounding boxes of the red blobs
    
    % this section looks for red disks that are adjacent to each other and
    % estimates l_mean
    iter = 0;
    l_sum = 0;
    for i=1:length(centroidRed)-1 % go through all centroids of red disks
        for j=i+1:length(centroidRed(:,1)) % against all other centroids of red disks
            lt = sqrt((centroidRed(i,1)-centroidRed(j,1))^2 + (centroidRed(i,2)-centroidRed(j,2))^2); % distance between markers
            if lt < l_mean*1.3 && lt > l_mean * 0.7 % is it possible that the disks are adjacent
                l_sum = l_sum + lt; % used to calc next mean length
                iter = iter + 1;
            end
        end
    end
    l_mean = l_sum/(iter); % used to calculate the height and future dists
    
    % this section identifies the 9 closest markers to the quad in the Newtonian Frame
    qMarkersP = zeros(nParticles,9,2);
    P_w = zeros(nParticles,1);
    qLocP = zeros(nParticles,4);
    errorTracker = zeros(nParticles,1);
    for np=1:nParticles
        
        qLocP(np,1) = qLoc(1) + sqrt(R_translate)*randn;
        qLocP(np,2) = qLoc(2) + sqrt(R_translate)*randn;
        qLocP(np,3) = qLoc(3) + sqrt(R_altitude)*randn;
        qLocP(np,4) = qLoc(4) + sqrt(R_theta)*randn;
        
        myMarker(1) = round(qLocP(np,1)); % which marker am I closest to x?
        myMarker(2) = round(qLocP(np,2)); % which marker am I closest to y?
        mx = [-markerSpacing 0 markerSpacing -markerSpacing 0 markerSpacing -markerSpacing 0 markerSpacing]; % markers grid location from center - x
        my = [-markerSpacing -markerSpacing -markerSpacing 0 0 0 markerSpacing markerSpacing markerSpacing]; % markers grid location from venter - y

        nMarkers = zeros(9,2);
        for i=1:9
            nMarkers(i,1) = myMarker(1) + mx(i); % marker location in Newtonian Frame
            nMarkers(i,2) = myMarker(2) + my(i); % marker location in Newtonian Frame
        end

        % this section moves the markers into the quadrotor's video frame and
        % predicts their location in pixels  
        for i=1:9
            nDx = nMarkers(i,1)-qLocP(np,1);
            nDy = nMarkers(i,2)-qLocP(np,2);
            qMarkersP(np,i,1) = x_pix + l_mean * (nDx * cos(qLocP(np,4)) + nDy * sin(qLocP(np,4)) );
            qMarkersP(np,i,2) = y_pix + l_mean * (nDx * -sin(qLocP(np,4)) + nDy * cos(qLocP(np,4)) );
        end

        % this section identifies which markers are actually present
        sumError = 0;
        for i=1:9
            error_tolerance = 0.4 * l_mean; % predicted marker must be within 30% of marker spacing in order to be accepted as a match
            errorMarkersX = 2*l_mean; % make the error the max acceptable as default
            errorMarkersY = 2*l_mean;
            for j=1:length( centroidRed )
                d = sqrt( (centroidRed(j,1)-qMarkersP(np,i,1))^2 + (centroidRed(j,2)-qMarkersP(np,i,2))^2 );
                if d < error_tolerance
                    error_tolerance = d; % want the closest one, not just one under tolerance                    
                    errorMarkersX = abs( centroidRed(j,1)-qMarkersP(np,i,1) );
                    errorMarkersY = abs( centroidRed(j,2)-qMarkersP(np,i,2) );
                end
            end
            sumError = sumError + errorMarkersX + errorMarkersY;
        end
        errorTracker(np) = sumError;
        
        %Generate the weights for each of these particles.
        sumError = sumError / 1000;
        P_w(np) = (1/sqrt(2*pi*R_total)) * exp(-(sumError)^2/(2*R_total)); 
    end
    
    P_w = P_w./sum(P_w);
    
    figure(36)
    clf
    for i=1:length( centroidRed )
        plot(centroidRed(i,1), centroidRed(i,2), 'ro')
        hold on
    end
    for np=1:nParticles
        for i=1:9
            plot(qMarkersP(np,i,1), qMarkersP(np,i,2), 'b.')
            hold on
        end
    end

    
    temp = zeros(np,9,2);
    temp2 = zeros(np,4);
    for np = 1:nParticles
        ind = find(rand <= cumsum(P_w),1);
        temp(np,:,:) = qMarkersP(ind,:,:);
        temp2(np,:) = qLocP(ind,:);
    end
    qMarkersP = temp;
    qLocP = temp2;
    
    qLoc(1) = mean(qLocP(:,1));
    qLoc(2) = mean(qLocP(:,2));
    qLoc(3) = mean(qLocP(:,3));
    qLoc(4) = mean(qLocP(:,4));
    
    %R_translate = mean( var(qLocP(:,1) ) + var(qLocP(:,2)) );
    %R_altitude = var(qLocP(:,3));
    %R_theta = var(qLocP(:,4)); 
    
    figure(26)
    clf
    for i=1:length( centroidRed )
        plot(centroidRed(i,1), centroidRed(i,2), 'ro')
        hold on
    end

    for np=1:nParticles
        for i=1:9
            plot(qMarkersP(np,i,1), qMarkersP(np,i,2), 'b.')
            hold on
        end
    end
    
    qMarkers = zeros(9,2);
    for i=1:9
        qMarkers(i,1) = mean(qMarkersP(:,i,1));
        qMarkers(i,2) = mean(qMarkersP(:,i,2));
    end
    
    for i=1:9
        qMarkers(i,:)
        plot(qMarkers(i,1), qMarkers(i,2), 'gx')
        hold on
    end
    
    plot(x_pix, y_pix, 'kx')
    axis([0 x_pix_max 0 y_pix_max])
    rgbFrame = step(htextinsCent2, rgbFrame, [uint16(qLoc(1)*100) uint16(qLoc(2)*100)], [uint16(x_pix) uint16(y_pix)]);
    
    x(nFrame,1) = qLoc(1);
    x(nFrame,2) = qLoc(2);
    x(nFrame,3) = qLoc(3);
    x(nFrame,4) = qLoc(4);

    index
    
    %waitforbuttonpress
    
    
    figure(1)
    plot(x(nFrame,1),x(nFrame,2),'rx')
    grid on
    axis([0 20 0 20])

    step(videoFWriter, rgbFrame);
    step(hVideoIn, rgbFrame); % Output video stream
    nFrame = nFrame + 1;
    time(nFrame) = time(nFrame-1) + 1/frame_rate;
end
    
    %{
    
    waitforbuttonpress
    
    iter = 0;
    sumPN = 0;        
    % this sections calculates l_mean and the qauds z
    for i=1:8 % compare all found markers
        if mRedMarkers(i,1) >= 0 % only found markers
            for j=i+1:9 % against all other found markers
                if mRedMarkers(j,1) >= 0 % only found markers
                    iter = iter+1;
                    % get expected newtonian length
                    nL = sqrt( (nMarkers(i,1) - nMarkers(j,1))^2 + (nMarkers(i,2) - nMarkers(j,2))^2 );

                    % get pixel length
                    nP = sqrt( (mRedMarkers(i,1) - mRedMarkers(j,1))^2 + (mRedMarkers(i,2) - mRedMarkers(j,2))^2 );

                    % get p / n
                    sumPN = sumPN + nP / nL;
                end
            end
        end
    end
    if iter > 0
        l_mean = sumPN / iter;
        qLoc(3) = l_mean * 95;
    end
    % this section estimates the quads x/y

    eX = 0;
    eY = 0;
    iter =0;
    for i=1:9
        if mRedMarkers(i,1) >= 0
            eX = eX + errorMarkers(i,1);
            eY = eY + errorMarkers(i,2);
            iter = iter + 1;
        end
    end

    qLoc(1) = qLoc(1) - ( eX / iter )/l_mean;
    qLoc(2) = qLoc(2) - ( eY / iter )/l_mean;

    % this sections estimates the quads yaw
    eT = 0;
    iter =0;

    for i=1:9
        if mRedMarkers(i,1) >= 0 % get the angle from the center of the screen to each marker
            mDx = x_pix - mRedMarkers(i,1);
            mDy = y_pix - mRedMarkers(i,2);
            mAngle = atan2(mDy,mDx);

            plot([x_pix, mRedMarkers(i,1)],[y_pix, mRedMarkers(i,2)], 'b');
            hold on
            plot([x_pix, qMarkersP(i,1)],[y_pix, qMarkersP(i,2)], 'k');

            qDx = x_pix - qMarkersP(i,1);
            qDy = y_pix - qMarkersP(i,2);
            qAngle = atan2(qDy, qDx);

            eT = eT + mAngle - qAngle;
            iter = iter + 1;
        end
    end

    if iter > 0
        if eT / iter < 0.2
            qLoc(4) = qLoc(4) - eT / iter;
        end
    end

    qLoc(:)

    rgbFrame = step(htextinsCent2, rgbFrame, [uint16(qLoc(1)*100) uint16(qLoc(2)*100)], [uint16(x_pix) uint16(y_pix)]);
    
    x(nFrame,1) = qLoc(1);
    x(nFrame,2) = qLoc(2);
    x(nFrame,3) = qLoc(3);
    x(nFrame,4) = qLoc(4);

    index
    
    %waitforbuttonpress
    
    
    %figure(1)
    %plot(x(nFrame,1),x(nFrame,2),'rx')
    %grid on
    %axis([0 20 0 20])

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
    
    %}