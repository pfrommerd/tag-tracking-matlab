function [ tags ] = load_tags( file )
    [ids, sizes, border, posX, posY, posZ, rotX, rotY, rotZ, rotW] = ...
        textread(file, '%f %f %f %f %f %f %f %f %f %f', 'commentstyle', 'shell');

    tags = cell(1, length(ids));
    
    for i=1:length(ids)
        tag(1).id = ids(i);
        tag(1).color = 'r';
        tag(1).size = 0.001 * [sizes(i) sizes(i)];
        tag(1).border = 0.001 * [border(i) border(i)];

        pos = (0.001 .* [posX(i) posY(i) posZ(i)])';
        rot = vrrotvec_to_quat([rotX(i) rotY(i) rotZ(i) rotW(i)])';
        tag(1).state = [pos; rot; 0; 0; 0; 0; 0; 0];

        tags{i} = tag;
    end
end

