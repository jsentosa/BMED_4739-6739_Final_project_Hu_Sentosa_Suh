clear
clc

cir = imread('Enhanced_cropped.tif');
cir = cir(:,:,1:3);

[BW,~] = createMask(cir);
[row, column, ~] = find(BW==0);
BW_cropped = BW(min(row):max(row), min(column):max(column));

[r,c] = size(BW_cropped);

BW_resized = imresize(BW_cropped,[c+1 c+1]);
% Estimation -> 351 pixels = 10 cm

BW_center = BW_resized;
BW_center(c/2+1,c/2+1) = 0;

center = c/2+1;
[row, column, dataValues] = find(BW_resized==0);
index = [row,column];

% Distance from center
DfC = zeros(size(index,1),1);
Predicted = ones(size(index,1),1)*5;

for i = 1:size(index,1)
    DfC(i) = sqrt((center-index(i,1))^2+(center-index(i,2))^2)/35.1;
end

Diff = abs(Predicted-DfC)

RMSE = rmse(Predicted,DfC)


%%

function [BW,maskedRGBImage] = createMask(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder app. The colorspace and
%  range for each channel of the colorspace were set within the app. The
%  segmentation mask is returned in BW, and a composite of the mask and
%  original RGB images is returned in maskedRGBImage.

% Auto-generated by colorThresholder app on 30-Apr-2023
%------------------------------------------------------


% Convert RGB image to chosen color space
I = rgb2ycbcr(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 32.000;
channel1Max = 255.000;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.000;
channel2Max = 255.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.000;
channel3Max = 255.000;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

end
