%% 清理屏幕、工作区、窗口
clear; clc; close all;

%% 参数
gray_div = 0.85;    % 转二值图像时的阈值
fps = 1000;          % 每秒帧数
fpf = 1;           % 每帧间隔采样点数
arr_num = 100;      % 向量数量（总数为 arr_num*2+1）
start_delay = 0;    % 开始做图前等的秒数


%% 原始数据准备================================================================
%% GUI选择需要处理的图形文件
% uigetfile：图形界面打开文件
% [FILENAME, PATHNAME, FILTERINDEX] = uigetfile(FILTERSPEC, TITLE)
[filename, pathname, filterindex] = uigetfile(...
   {'*.*',      '所有文件(*.*)'; ...
    '*.bmp',    '图片(*.bmp)'; ...
    '*.jpg',    '图片(*.jpg)'}, ...
    '请选择要处理的选择图片');

%% imread 读取选择的文件，并保存到 input_image，利用 imshow 显示原始图像
input_image = imread([pathname filename]);      %RGB类型是uint8
input_image = im2double(input_image);           %归一化，转成double [0, 1]
img_gray = rgb2gray(input_image);               %灰度化
%figure; imshow(input_image); title('原始图像'); %显示原始图像

%% 图像处理================================================================
%% 二值化
% im2bw Convert image to binary image by thresholding.
main_fig = figure;
set(main_fig, 'position', get(0, 'ScreenSize'));    %全屏
img_bw = im2bw(img_gray, gray_div);
subplot(3, 2, 1); imshow(img_gray); title('灰度图像');
subplot(3, 2, 2); imshow(img_bw); title('二值化图像');
img_bw_inv = ~img_bw;

%% 轮廓提取
[M, N] = size(img_bw);  %图像大小
img_edge = edge(img_bw, 'sobel');
%{
for i = 2:M-1
    for j = 2:N-1
        if (img_bw(i,j) && img_bw(i+1,j) && img_bw(i-1,j) && img_bw(i,j+1) && img_bw(i,j-1) && img_bw(i+1,j+1) && img_bw(i+1,j-1) && img_bw(i-1,j+1) && img_bw(i-1,j-1) )
            img_edge(i,j) = 0;
        end
    end
end
%}
%subplot(2, 2, 3); imshow(img_edge); title('边缘图像');

%% 填充
F1 = imfill(img_bw_inv, 'holes');
subplot(3, 2, 3); imshow(F1); title('填充图像');

%% 边界跟踪
img_bw_label = bwlabel(F1);
[pr, pc] = find(img_bw_label);
row = pr(4);
col = pc(4);
connectivity = 8;
contour = bwtraceboundary(img_bw_label, [row, col], 'S', connectivity);
route = contour;
route(:,1)=route(:,1)*-1+N/2;   %<---contour 出来的坐标是图片左上角为原点
route(:,2)=route(:,2)-M/2;      %<---调整一下方向并使图片中心作为原点
subplot(3, 2, 4);
plot(route(:,2), route(:,1), 'g', 'LineWidth', 2), axis('equal');
title('边界跟踪');

%% 展示一下边界跟踪的效果
%{
figure;
imshow(input_image);
hold on;
plot(contour(:,2), contour(:,1), 'g', 'LineWidth', 2);
title('边界跟踪图像');
%}

%% 傅里叶分析================================================================
%% 生成复平面路径
%figure;
mapsize = max(N,M);
route_c = (route(:,1)*1i/mapsize+route(:,2)/mapsize);
subplot(3, 2, 5);
plot(route_c), axis('equal');
title('复平面图像');

%% 傅里叶变换
% 复数的傅里叶变换，正频率与负频率不再相等，箭头应该一正一反。
% 一个周期是1秒，一共 tot_num 个轨迹采样点
tot_num = size(route_c, 1);
dt = 1/tot_num;
%arr_num = 100;
c=zeros(arr_num, 2);    %每个箭头的系数，c[k][1] 是正（顺时针），c[k][2]是负（逆时针）
for k = 1:arr_num
    for sign = 1:2
        tmp = 0;        %积分
        for t_id = 1:tot_num
            tmp = tmp + route_c(t_id)*exp((-1)^sign*k * 2i*pi *(t_id*dt))*dt;
        end
        c(k, sign) = tmp;
    end
end
c0 = sum(route_c)*dt;


%% 绘制动画
%subplot(3, 2, 6);
figure;
draw_orbit = plot([0, 0], [0, 0], 'LineWidth', 1, 'Color', [.6 .6 .6]);  % 轨迹
hold on;
draw_arrows = plot([0, 0], [0, 0], 'LineWidth', 1, 'Color', [1 0 0]);    % 向量    
hold on;
draw_endpoint = plot(0, 0, 'k.', 'MarkerSize', 10);
map_min = min(min(real(route_c)), min(imag(route_c)))*1.2;
map_max = max(max(real(route_c)), max(imag(route_c)))*1.2;
xlim([map_min, map_max]);
ylim([map_min, map_max]);
%xlim([min(real(route_c))*1.2, max(real(route_c))*1.2]);
%ylim([min(imag(route_c))*1.2, max(imag(route_c))*1.2]);  %固定坐标范围
axis square;
title({'动画', ['arrnum=',num2str(arr_num)]});


data_orbit = [];
data_arrows = zeros(arr_num*2, 1);
data_endpoint = 0;
tot_arrow = arr_num*2+1;

pause(start_delay);
for t_id = 1:fpf:tot_num %每个时刻
    t = t_id * dt;
    %c0 重心
    data_arrows(1) = c0;
    %当前时刻每个频率的向量
    for k = 1:arr_num
        data_arrows(k*2) = c(k, 1)*exp(k*2i*pi*t);
        data_arrows(k*2+1)   = c(k, 2)*exp(-k*2i*pi*t); 
    end
    %叠加后每个频率的向量终点的位置
    for k = 2:tot_arrow
        data_arrows(k) = data_arrows(k-1) + data_arrows(k);
    end
    %轨迹与终点
    data_endpoint = data_arrows(tot_arrow);
    data_orbit = [data_orbit, data_endpoint];
    
    %更新图像
    set(draw_orbit, 'xdata', real(data_orbit), 'ydata', imag(data_orbit));
    set(draw_endpoint, 'xdata', real(data_endpoint), 'ydata', imag(data_endpoint));
    set(draw_arrows, 'xdata', real(data_arrows), 'ydata', imag(data_arrows));
    
    pause(1/fps);
end
