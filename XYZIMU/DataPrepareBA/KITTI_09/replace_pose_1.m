load ORB_2808_09.mat
load GT_P0_PA.mat

GT_P0 = GT_camimu; 
save GT_P0_PA.mat GT_P0 

for i = 1: size(GT_P0,1)
    file=strcat('Image',int2str(i),'.mat');
    load(file);   
    Image(1,:) = est_camimu_(i,:);
    save(file, 'Image');
end 

