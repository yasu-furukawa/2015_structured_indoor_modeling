function alignment_batch(data_directory, start_index, end_index)

dynamic_range = 3;

for p = start_index:end_index
    image_filename = sprintf('%s/panorama/%03d_img_%d.png', data_directory, p, dynamic_range);
    ply_filename = sprintf('%s/ply/%03d.ply', data_directory, p);
    points_filename = sprintf('%s/calibration/%03d.points', data_directory, p);
    
    alignment(image_filename, ply_filename, points_filename);
end





