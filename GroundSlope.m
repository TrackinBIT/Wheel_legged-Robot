
function gama_all = GroundSlope(xi, zi)
% zi  : [4x1] 四个轮子的 z 轴高度
% xi  : [4x1] 四个轮子的 x 轴位置
% gamma : 估计地面倾角（rad）
% 可优化 
    % 最小二乘拟合坡度
    x_mean = mean(xi);
    z_mean = mean(zi);
    
    numerator = sum((xi - x_mean) .* (zi - z_mean));
    denominator = sum((xi - x_mean).^2);
    
    a = numerator / denominator;
    gama = atan(a);  
    gama_all = [gama;gama;gama;gama]
end
