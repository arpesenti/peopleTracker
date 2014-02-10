function h = subplottight(n,m,i)
    [c,r] = ind2sub([m n], i);
    h = subplot('Position', [(c-1)/m, 1-(r)/n, 1/m, 1/n]);