function save_images(input, output_dir)
    c = 1;
    while input.hasImage()
        img = input.readImage();
        file = sprintf('%s/%04d.png', output_dir, c);
        imwrite(img, file);
        c = c + 1;
    end
end