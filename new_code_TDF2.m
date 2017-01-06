close all
clear all
clc

redThresh = 0.1;
videoFReader = vision.VideoFileReader('20150619_ft_4_wc_cut2.mp4');
videoPlayer = vision.VideoPlayer;
videoFWriter = vision.VideoFileWriter('20150619_ft_1_cut_out.avi');
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
       0 0 0 0 0 0 1 1/frame_rate; % th' = th + thd*dt
       0 0 0 0 0 0 0 1]; % thd' = thd
% can only sense position and orientation
h = [1 0 0 0 0 0 0 0; % x
     0 0 1 0 0 0 0 0; % y
     0 0 0 0 1 0 0 0; % z
     0 0 0 0 0 0 1 0]; % th
xh = zeros(8,1);
r = 0.1;
q = 0.1*eye(8);
pm = eye(8);

% initial guess at the orientation (10,10,2,0)
xm = [10; 0; 10; 0; 2.4; 0; 0 ; 0];
qLoc = [10.2,10.3,2.4,0]; % initial location
markerSpacing = 1;

time = 0;
l_median = 200;
index = 0;
while ~isDone(videoFReader) && index < 900
    if (index == 1001)
        asfsa
    end
    % this sections finds the red disks
    index = index + 1;
    [index l_median]
    rgbFrame = step(videoFReader);
    % rgbFrame = step(htextinsCent, rgbFrame, [uint16(x_pix) uint16(y_pix)], [uint16(x_pix-6) uint16(y_pix-9)]);
    clear centroidRed bboxRed
    diffFrameRed = imsubtract(rgbFrame(:,:,1), rgb2gray(rgbFrame)); % Get red component of the image
    diffFrameRed = medfilt2(diffFrameRed, [3 3]); % Filter out the noise by using median filter
    binFrameRed = im2bw(diffFrameRed, redThresh); % Convert the image into binary image with the red objects as white
    [centroidRed, bboxRed] = step(hblob, binFrameRed); % Get the centroids and bounding boxes of the red blobs
    
    
    % this section looks for red disks that are adjacent to each other and
    % estimates l_mean
    clear l_list;
    for i=1:length(centroidRed)-1 % go through all centroids of red disks
        for j=i+1:length(centroidRed(:,1)) % against all other centroids of red disks
            lt = sqrt((centroidRed(i,1)-centroidRed(j,1))^2 + (centroidRed(i,2)-centroidRed(j,2))^2); % distance between markers
            if lt < l_median*1.205 && lt > l_median * 0.795 % is it possible that the disks are adjacent
                l_list(i) = lt; % used to calc next mean length
            end
        end
    end
    l_median = median(l_list); % used to calculate the height and future dists
    
    % this section identifies the 9 closest markers to the quad in the Newtonian Frame
    myMarker(1) = round(qLoc(1)); % which marker am I closest to x?
    myMarker(2) = round(qLoc(2)); % which marker am I closest to y?
    
    mx = [-markerSpacing 0 markerSpacing
        -markerSpacing 0 markerSpacing
        -markerSpacing 0 markerSpacing]; % markers grid location from center - x
    
    my = [-markerSpacing -markerSpacing -markerSpacing
        0 0 0
        markerSpacing markerSpacing markerSpacing]; % markers grid location from center - y
    nMarkers = zeros(9,2);
    for i=1:9
        nMarkers(i,1) = myMarker(1) + mx(i); % marker location in Newtonian Frame
        nMarkers(i,2) = myMarker(2) + my(i); % marker location in Newtonian Frame
    end
    
    % this section moves the markers into the quadrotor's video frame and
    % predicts their location in pixels  
    qMarkers = zeros(9,2);
    cTheta = cos(qLoc(4));
    sTheta = sin(qLoc(4));
    for i=1:9
        nDx = nMarkers(i,1)-qLoc(1);
        nDy = nMarkers(i,2)-qLoc(2);
        qMarkers(i,1) = x_pix + l_median * (nDx * cTheta + nDy * sTheta );
        qMarkers(i,2) = y_pix + l_median * (nDx * -sTheta + nDy * cTheta );
    end
    
    % this section identifies which markers are actually present
    mRedMarkers = ones(9,2)*-1;
    errorMarkers = ones(9,2)*-1;
    isMarkers = false;
    for i=1:9
        error_tolerance = 0.4 * l_median;  % predicted marker must be within 30% of marker spacing in order to be accepted as a match
        for j=1:length( centroidRed )
            d = sqrt( (centroidRed(j,1)-qMarkers(i,1))^2 + (centroidRed(j,2)-qMarkers(i,2))^2 );
            if d < error_tolerance
                error_tolerance = d;
                mRedMarkers(i,1) = centroidRed(j,1);
                mRedMarkers(i,2) = centroidRed(j,2);
                
                errorMarkers(i,1) = centroidRed(j,1)-qMarkers(i,1);
                errorMarkers(i,2) = centroidRed(j,2)-qMarkers(i,2);
                
                isMarkers = true;
            end
        end
    end
    
    figure(26)
    clf
    for i=1:length( centroidRed )
        plot(centroidRed(i,1), centroidRed(i,2), 'ro')
        hold on
    end
    
    for i=1:9
        plot(qMarkers(i,1), qMarkers(i,2), 'b.')
        hold on
    end
    
    for i=1:9
        if mRedMarkers(i,1) >= 0
            plot([mRedMarkers(i,1), qMarkers(i,1)], [mRedMarkers(i,2),qMarkers(i,2)], 'g')
            hold on
        end
    end
    
    plot(x_pix, y_pix, 'kx')
    axis([0 x_pix_max 0 y_pix_max])
    
    % if markers are present continue, else kinematic predict
    if isMarkers
        
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
                        if (index == 1001)
                            [sumPN nP nL mRedMarkers(i,1) mRedMarkers(i,2) mRedMarkers(j,1) mRedMarkers(j,2) nMarkers(i,1) nMarkers(i,2) nMarkers(j,1) nMarkers(j,2)]
                        end
                    end
                end
            end
        end
        if iter > 0
            l_median = sumPN / iter;
            qLoc(3) = l_median/85;
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
        
        qLoc(1) = qLoc(1) - ( eX / iter )/l_median;
        qLoc(2) = qLoc(2) - ( eY / iter )/l_median;
        
        % this sections estimates the quads yaw
        eT = 0;
        iter =0;
        
        thetaList = [];
        for i=1:9
            if mRedMarkers(i,1) >= 0 % get the angle from the center of the screen to each marker
                mDx = x_pix - mRedMarkers(i,1);
                mDy = y_pix - mRedMarkers(i,2);
                mAngle = atan2(mDy,mDx);
                mAngleD = mAngle*180/pi;
                
                plot([x_pix, mRedMarkers(i,1)],[y_pix, mRedMarkers(i,2)], 'b');
                hold on
                plot([x_pix, qMarkers(i,1)],[y_pix, qMarkers(i,2)], 'k');
                
                qDx = x_pix - qMarkers(i,1);
                qDy = y_pix - qMarkers(i,2);
                qAngle = atan2(qDy, qDx);
                qAngleD = qAngle*180/pi;
 
                iter = iter + 1;
                thetaList(iter) = mAngle - qAngle;
            end
        end
        
        if iter > 0
            qLoc(4) = qLoc(4) - median(thetaList);
        end
    
        
        %% kalman filter
        % compute Kalman gain, figure 4.1
        ka = pm*h'/(h*pm*h'+r);
        % take measurement z(k)
        z = [qLoc(1) qLoc(2) qLoc(3) qLoc(4)]';
        zL(nFrame,1:4) = z(1:4);
        % update estimate with measurement z(k), figure 4.1
        xh = xm+ka*(z-h*xm);
        % Compute error covariance for updated estimate, figure 4.1
        p = (eye(8)-ka*h)*pm;
        % Project ahead, figure 4.1
        xm = phi*xh;
        pm = phi*p*phi'+q;
        p_trace(nFrame) = trace(nFrame);
        % locate corners from current position
        
         
        %rgbFrame = step(htextinsCent2, rgbFrame, [uint16(mean(qLocP(:,1))*100) uint16(mean(qLocP(:,2))*100)], [x_pix y_pix]);
        rgbFrame = step(htextinsCent2, rgbFrame, [uint16(xh(1)*100) uint16(xh(3)*100)], [uint16(x_pix) uint16(y_pix)]);

        qLoc(1) = xh(1) + xh(2)*1/frame_rate;
        qLoc(2) = xh(3) + xh(4)*1/frame_rate;
        qLoc(3) = xh(5) + xh(6)*1/frame_rate;
        qLoc(4) = xh(7) + xh(8)*1/frame_rate;

    end
    
    if index > 109
        %waitforbuttonpress
    end
    
    x(nFrame,1:4) = qLoc(1:4);
    v(nFrame,1) = xh(2);
    v(nFrame,2) = xh(4);
    v(nFrame,3) = xh(6);
    v(nFrame,4) = xh(8);
    
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

figure(10)
subplot(4,1,1)
plot(time,x(:,1),'b')
subplot(4,1,2)
plot(time,x(:,2),'b')
subplot(4,1,3)
plot(time,x(:,3),'b')
subplot(4,1,4)
plot(time,x(:,4),'b')
grid on

figure(11)
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


%% Analyzing Motion Capture Data for comparison
load('Trial06.mat');
output(:,2) = output(:,2)/1000;
output(:,3:4) = output(:,3:4)/1000;

output2 = output;
output2(:,2) = -output(:,3);
output2(:,3) = -output(:,2);

x(:,1) = x(:,1) - x(1,1);
x(:,2) = x(:,2) - x(1,2);

x2 = x.*0.3048;

output2(:,2) = output2(:,2) - output2(1,2);
output2(:,3) = output2(:,3) - output2(1,3);

qXYZ  = quaternion.eulerangles('XYZ',output2(:,5)*pi/180,output2(:,6)*pi/180,output2(:,7)*pi/180);
angles = EulerAngles(qXYZ,'ZYX')';
angles = angles*180/pi;

qXYZ2=SpinCalc('EA123toQ',output2(:,5:7)*pi/180,1e-5,0);
angles2 = SpinCalc('QtoEA321',qXYZ2, 1e-5, 0);
angles2 = angles*180/pi;

figure(5)
subplot(3,1,1)
plot(output2(:,1), output2(:,2))
ylabel('X-Pos MoCap')
subplot(3,1,2)
plot(output2(:,1), output2(:,3))
ylabel('Y-Pos MoCap')
subplot(3,1,3)
plot(output2(:,1), output2(:,4))
ylabel('Z-Pos MoCap')

[r, lag] = xcorr(x2(:,1),output2(:,2));

[~,I] = max(abs(r));
lagDiff = -lag(I);

%output2(1:lagDiff,:) = [];
%% for video 4 and workspace 6
output2(:,1) = output2(:,1) - output2(1,1)-1.69;

for k = 1:length(output2)
    if (output2(k,1) >= time(end))
        break;
    end
end
output2(k:end,:) = [];
angles(k:end,:) = [];
angles2(k:end,:) = [];

for k = 1:length(output2)
    if (output2(k,1) >= 0)
        break;
    end
end

output2(1:k,:) = [];
angles(1:k,:) = [];
angles2(1:k,:) = [];

figure(9)
subplot(3,1,1)
set(gca,'fontsize',14)
hold on
plot(output2(:,1), output2(:,2), time, x2(:,1))
xlabel('Time [sec]')
ylabel('Distance [ft]')
legend('Motion Capture','Cam Estimate')
subplot(3,1,2)
set(gca,'fontsize',14)
hold on
plot(output2(:,1), output2(:,3), time, x2(:,2)*480/640)
xlabel('Time [sec]')
ylabel('Distance [ft]')
subplot(3,1,3)
plot(output2(:,1), output2(:,4), time,x2(:,3), time, (322.5806./l_median)^-1)
xlabel('Time [sec]')
ylabel('Altitude [ft]')
legend('Motion Capture','Cam Estimate', '322.5806/l_m')

o2(:,1) = interp1(output2(:,1),output2(:,2),time);
o2(:,2) = interp1(output2(:,1),output2(:,3),time);
o2(:,3) = interp1(output2(:,1),output2(:,4),time);

E(:,1) = o2(1:end-1,1)-x2(1:end-1,1);
E(:,2) = o2(1:end-1,2)-x2(1:end-1,2);
E(:,3) = o2(1:end-1,3)-x2(1:end-1,3);

mE(1) = mean(E(:,1));
mE(2) = mean(E(:,2));
mE(3) = mean(E(:,3));

x2(:,1) = x2(:,1) + mE(1);
x2(:,2) = x2(:,2) + mE(2);
x2(:,3) = x2(:,3) + mE(3);

E(:,1) = o2(1:end-1,1)-x2(1:end-1,1);
E(:,2) = o2(1:end-1,2)-x2(1:end-1,2);
E(:,3) = o2(1:end-1,3)-x2(1:end-1,3);

mE(1) = mean(E(:,1));
mE(2) = mean(E(:,2));
mE(3) = mean(E(:,3));

sE(1) = std(E(:,1));
sE(2) = std(E(:,2));
sE(3) = std(E(:,3));

maE(1) = mean(abs(E(:,1)));
maE(2) = mean(abs(E(:,2)));
maE(3) = mean(abs(E(:,3)));
%%
figure(8)
set(gca,'fontsize',16, 'fontname', 'Times')
plot(time,o2(:,1),'g',time,x2(:,1),'b',time(1:end-1),abs(E(:,1)),'r','LineWidth',2)
legend('Motion Capture','Cam Estimate','Residual')
grid on
xlabel('Time (s)')
ylabel('Position X (m)')
figure(9)
set(gca,'fontsize',16, 'fontname', 'Times')
plot(time,o2(:,2),'g',time,x2(:,2),'b',time(1:end-1),abs(E(:,2)),'r','LineWidth',2)
legend('Motion Capture','Cam Estimate','Residual')
grid on
xlabel('Time (s)')
ylabel('Position Y (m)')
figure(10)
set(gca,'fontsize',16, 'fontname', 'Times')
plot(time,o2(:,3),'g',time,x2(:,3),'b',time(1:end-1),abs(E(:,3)),'r','LineWidth',2)
legend('Motion Capture','Cam Estimate','Residual')
grid on
xlabel('Time (s)')
ylabel('Position Z (m)')

figure(12)
set(gca,'fontsize',16,'fontname','Times')
plot(output2(:,1), angles(:,1), time,x(:,4)*180/3.14, 'linewidth',2)
legend('MoCap', 'Cam Estimate')
grid on
xlabel('Time (s)')
ylabel('Yaw Angle (deg)')