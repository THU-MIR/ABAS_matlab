y = 26;
x = 41;
empty_world = ones(int32(150/y),int32(250/y));
%set(gcf,'Position',[100 100 400 400]);

save environment

imshow(empty_world,'InitialMagnification','fit')
%imshow(img,'InitialMagnification','fit');
%set(gcf,'Position',[100 100 400 400]);
axis on 
grid on
grid minor