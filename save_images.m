function save_images(input, output_dir)
    for c=1:length(input)
        img = input{c}.cdata;
        file = sprintf('%s/%04d.png', output_dir, c);
        imwrite(img, file);
    end
end
%{
function save_images(input, output_dir)
    c = 1;
    while input.hasImage()
        img = input.readImage();
        file = sprintf('%s/%04d.png', output_dir, c);
        imwrite(img, file);
        c = c + 1;
    end
end
%}