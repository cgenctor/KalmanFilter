%% CRV_24_ImageSequenceAnalysisKalman
% name : Candas Genctor

%% clean up
clear;
close all;
clc;
%% Select sequence
imgSeqDir = ['C:\Users\canda\OneDrive\Belgeler\MATLAB\robWall\robWall', filesep];
imgSeqN = length(dir(imgSeqDir))-2;

%% Create matrix for measurements
m = zeros(2,imgSeqN);
%% Load pattern
pattern = imread('pattern.png');
[mPattern,nPattern] = size(pattern);
%% Process image sequence
success = zeros(1,imgSeqN);
f = figure();
for k= 1:imgSeqN
    % Load image
    I = rgb2gray(imread([imgSeqDir,'image_',int2str(k),'.png']));
    % Pattern matching and measurements
    c = normxcorr2(pattern,I);
    [ypeak, xpeak] = find(c==max(c(:)));
    
    m(:,k) = ([xpeak, ypeak] - [mPattern,nPattern]/2)'; 
% Find a threshold v, according to the Max Correlation
    v = 0.58366;    % below this threshold approach failed to find the robot,
                    % above or equal robot was found
    % Visualization
    h = figure(f);
    imshow(I);
    hold on;
    if c(ypeak, xpeak) >= v
        plot(m(1,k),m(2,k),'rx','MarkerSize',15,'LineWidth',3);
    end
    hold off;
    title(['Max. correlation: ',num2str(c(ypeak, xpeak))]);
    drawnow;
    pause(0.001);

    % saveas(h,sprintf('FIG%d.png',k)); % will create FIG1, FIG2,...

    if c(ypeak, xpeak) >= v
        success(1,k) = 1; % when succeeded finding the robot
        % R = 10^-3.*[500 1; 1 500]; 
    else
        success(1,k) = 0; % when it did not succeed finding the robot
        % R = 10^-3.*[50000 1; 1 50000]; 
    end
end
%% given
fi_k = [1 0 1 0;0 1 0 1;0 0 1 0;0 0 0 1];   % state transition matrix 
H_k = [1 0 0 0;0 1 0 0];                    % measurement model matrix
Q = 10^-6.*[100 1 1 1;1 100 1 1;1 1 100 1;1 1 1 100];   % system noise covariance matrix
% measurement noise covariance matrix depends on if the robot is found or not
estCk_1 = 10^-6.*[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];     % estimated process noise covariance matrix
%estSk_1 = [0;0;0;0];          % initial state estimation
estSk_1 = [1470;700;0;0];       % initial state estimation
%% For every image except the first one:
estSk = zeros(4,imgSeqN); % Create a matrix to save the state estimations 
IdentityMatrix = eye(size(H_k,2));   % Identity matrix (axa) H_k(bxa)
for ii = 1:imgSeqN
    prediction_Sk = fi_k * estSk_1;                 % Prediction of state
    prediction_Ck = fi_k * estCk_1 * fi_k' + Q;     % Prediction of state covariance matrix
    if success(1,ii) == 1
        R = 10^-3.*[500 1; 1 500]; % when succeeded finding the robot
    else
        R = 10^-3.*[50000 1; 1 50000]; % when it did not succeed finding the robot 
    end
    GkM = prediction_Ck * H_k' * ( H_k * prediction_Ck * H_k' + R)^(-1);    % Gain matrix
    pred_m_k = H_k * fi_k * estSk_1;                % new predicted (expected) measurement
    estSk(:,ii) = prediction_Sk + GkM * (m(:,ii) - pred_m_k);    % Correction of the predicted state
    estCk = (IdentityMatrix - GkM * H_k) * prediction_Ck * ...
        (IdentityMatrix - GkM * H_k)' + GkM * R * GkM'; % Correction of predicted state covariance matrix 
    estSk_1 = estSk(:,ii);
    estCk_1 = estCk;
end
%% Plot the results in one figure showing the true (unknown)
% positions, the measurements and the estimated positions. 
figure
imshow(I)
hold on 
plot(estSk(1,:),estSk(2,:),'g.','MarkerSize',15,'LineWidth',3)    %estimated 
plot(m(1,:),m(2,:),'r.','MarkerSize',15,'LineWidth',3)            %measured
legend('Estimated','Measured')
