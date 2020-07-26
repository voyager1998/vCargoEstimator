function h = Cal_h(model_g, model_u, pc_g, pc_u)
    %% Calculate height
    pg_homo = [pc_g ones(size(pc_g,1),1)];
    pu_homo = [pc_u ones(size(pc_u,1),1)];
    h1 = abs((model_g*pu_homo')/norm(model_g(1:3)));
    h2 = abs((model_u*pg_homo')/norm(model_u(1:3)));
    h = (sum(h1)+sum(h2))/(length(h1)+length(h2));
end