%%
clc;
close all;
clear;
%%
for i = 15997:-1:1
    if isempty(Cells.cell(i).CCV.V)
        Cells.cell(i) = [];
    end
    if isempty(Cells.cell(i).CCD.V)
        Cells.cell(i) = [];
    end
end
%%
% feature_clean
arows = all(feature == 0,2);
feature(arows,:) = [];
label(arows,:) = [];
%%
% 零值替换
for col = 1:size(feature, 2)
    column = feature(:, col);
    zeroIndices = find(column == 0);
    nonZeroIndices = find(column ~= 0);
    
    % 如果存在零值
    if ~isempty(zeroIndices)
        % 计算非零值的均值或其他统计量
        nonZeroMean = mean(column(nonZeroIndices));
        
        % 将零值替换为非零值的均值
        column(zeroIndices) = nonZeroMean;
        
        % 更新特征集中的列
        feature(:, col) = column;
    end
end