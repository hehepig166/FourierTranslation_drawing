%% ������Ļ��������������
clear; clc; close all;

%% ����
gray_div = 0.85;    % ת��ֵͼ��ʱ����ֵ
fps = 1000;          % ÿ��֡��
fpf = 1;           % ÿ֡�����������
arr_num = 100;      % ��������������Ϊ arr_num*2+1��
start_delay = 0;    % ��ʼ��ͼǰ�ȵ�����


%% ԭʼ����׼��================================================================
%% GUIѡ����Ҫ�����ͼ���ļ�
% uigetfile��ͼ�ν�����ļ�
% [FILENAME, PATHNAME, FILTERINDEX] = uigetfile(FILTERSPEC, TITLE)
[filename, pathname, filterindex] = uigetfile(...
   {'*.*',      '�����ļ�(*.*)'; ...
    '*.bmp',    'ͼƬ(*.bmp)'; ...
    '*.jpg',    'ͼƬ(*.jpg)'}, ...
    '��ѡ��Ҫ�����ѡ��ͼƬ');

%% imread ��ȡѡ����ļ��������浽 input_image������ imshow ��ʾԭʼͼ��
input_image = imread([pathname filename]);      %RGB������uint8
input_image = im2double(input_image);           %��һ����ת��double [0, 1]
img_gray = rgb2gray(input_image);               %�ҶȻ�
%figure; imshow(input_image); title('ԭʼͼ��'); %��ʾԭʼͼ��

%% ͼ����================================================================
%% ��ֵ��
% im2bw Convert image to binary image by thresholding.
main_fig = figure;
set(main_fig, 'position', get(0, 'ScreenSize'));    %ȫ��
img_bw = im2bw(img_gray, gray_div);
subplot(3, 2, 1); imshow(img_gray); title('�Ҷ�ͼ��');
subplot(3, 2, 2); imshow(img_bw); title('��ֵ��ͼ��');
img_bw_inv = ~img_bw;

%% ������ȡ
[M, N] = size(img_bw);  %ͼ���С
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
%subplot(2, 2, 3); imshow(img_edge); title('��Եͼ��');

%% ���
F1 = imfill(img_bw_inv, 'holes');
subplot(3, 2, 3); imshow(F1); title('���ͼ��');

%% �߽����
img_bw_label = bwlabel(F1);
[pr, pc] = find(img_bw_label);
row = pr(4);
col = pc(4);
connectivity = 8;
contour = bwtraceboundary(img_bw_label, [row, col], 'S', connectivity);
route = contour;
route(:,1)=route(:,1)*-1+N/2;   %<---contour ������������ͼƬ���Ͻ�Ϊԭ��
route(:,2)=route(:,2)-M/2;      %<---����һ�·���ʹͼƬ������Ϊԭ��
subplot(3, 2, 4);
plot(route(:,2), route(:,1), 'g', 'LineWidth', 2), axis('equal');
title('�߽����');

%% չʾһ�±߽���ٵ�Ч��
%{
figure;
imshow(input_image);
hold on;
plot(contour(:,2), contour(:,1), 'g', 'LineWidth', 2);
title('�߽����ͼ��');
%}

%% ����Ҷ����================================================================
%% ���ɸ�ƽ��·��
%figure;
mapsize = max(N,M);
route_c = (route(:,1)*1i/mapsize+route(:,2)/mapsize);
subplot(3, 2, 5);
plot(route_c), axis('equal');
title('��ƽ��ͼ��');

%% ����Ҷ�任
% �����ĸ���Ҷ�任����Ƶ���븺Ƶ�ʲ�����ȣ���ͷӦ��һ��һ����
% һ��������1�룬һ�� tot_num ���켣������
tot_num = size(route_c, 1);
dt = 1/tot_num;
%arr_num = 100;
c=zeros(arr_num, 2);    %ÿ����ͷ��ϵ����c[k][1] ������˳ʱ�룩��c[k][2]�Ǹ�����ʱ�룩
for k = 1:arr_num
    for sign = 1:2
        tmp = 0;        %����
        for t_id = 1:tot_num
            tmp = tmp + route_c(t_id)*exp((-1)^sign*k * 2i*pi *(t_id*dt))*dt;
        end
        c(k, sign) = tmp;
    end
end
c0 = sum(route_c)*dt;


%% ���ƶ���
%subplot(3, 2, 6);
figure;
draw_orbit = plot([0, 0], [0, 0], 'LineWidth', 1, 'Color', [.6 .6 .6]);  % �켣
hold on;
draw_arrows = plot([0, 0], [0, 0], 'LineWidth', 1, 'Color', [1 0 0]);    % ����    
hold on;
draw_endpoint = plot(0, 0, 'k.', 'MarkerSize', 10);
map_min = min(min(real(route_c)), min(imag(route_c)))*1.2;
map_max = max(max(real(route_c)), max(imag(route_c)))*1.2;
xlim([map_min, map_max]);
ylim([map_min, map_max]);
%xlim([min(real(route_c))*1.2, max(real(route_c))*1.2]);
%ylim([min(imag(route_c))*1.2, max(imag(route_c))*1.2]);  %�̶����귶Χ
axis square;
title({'����', ['arrnum=',num2str(arr_num)]});


data_orbit = [];
data_arrows = zeros(arr_num*2, 1);
data_endpoint = 0;
tot_arrow = arr_num*2+1;

pause(start_delay);
for t_id = 1:fpf:tot_num %ÿ��ʱ��
    t = t_id * dt;
    %c0 ����
    data_arrows(1) = c0;
    %��ǰʱ��ÿ��Ƶ�ʵ�����
    for k = 1:arr_num
        data_arrows(k*2) = c(k, 1)*exp(k*2i*pi*t);
        data_arrows(k*2+1)   = c(k, 2)*exp(-k*2i*pi*t); 
    end
    %���Ӻ�ÿ��Ƶ�ʵ������յ��λ��
    for k = 2:tot_arrow
        data_arrows(k) = data_arrows(k-1) + data_arrows(k);
    end
    %�켣���յ�
    data_endpoint = data_arrows(tot_arrow);
    data_orbit = [data_orbit, data_endpoint];
    
    %����ͼ��
    set(draw_orbit, 'xdata', real(data_orbit), 'ydata', imag(data_orbit));
    set(draw_endpoint, 'xdata', real(data_endpoint), 'ydata', imag(data_endpoint));
    set(draw_arrows, 'xdata', real(data_arrows), 'ydata', imag(data_arrows));
    
    pause(1/fps);
end
