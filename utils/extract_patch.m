function p = extract_patch(patchSize, coords, img, tag)
    p = zeros(patchSize);
    
    H = tag.homography;

    c = homography_project(H, coords);
    
    x = round(c(1, :));
    y = round(c(2, :));

    if length(find(x < 1)) > 0 || ...
       length(find(y < 1)) > 0 || ...
       length(find(x > size(img, 2))) > 0 || ...
       length(find(y > size(img, 1))) > 0
        p = [];
        return
        %{
        yob = find(y < 1);
        y(yob) = ones(1, length(yob));
        yob = find(y > size(img, 1));
        y(yob) = ones(1, length(yob));
        
        xob = find(x < 1);
        x(xob) = ones(1, length(xob));
        xob = find(x > size(img, 2));
        x(xob) = ones(1, length(xob));        

        %length(ob)
        %c(:, ob(1:10))
        %coords(:, ob(1:10))
        %return;
        %}
    end
    idx = sub2ind(size(img), y, x);
    v = img(idx);
    p = reshape(v, size(p))';
end

