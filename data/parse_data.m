%%
v= VideoReader('2G4A2499.MOV');
i = 1;
x = 1;

 list = 1 : 20 : 2000;

while hasFrame(v)
    video = readFrame(v);
%       imshow(video);
     i
     if(i == list(x))
         imwrite(video, ['data', num2str(x), '.png']);
             x = x + 1
     end
% if(i>400)
%       pause;
% end
    i = i+1;
%     
end
