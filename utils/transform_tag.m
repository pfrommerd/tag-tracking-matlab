function transState = tag_transform(state, x)
    transState = state;
    transState(1:3) =  x(1:3) + state(1:3);
    transState(4:7) = qmult(x(4:7)', state(4:7)')';
end