function [l, w, h] = Cal_dis(pc, models)
    %% Calculate height
    normal_dp = abs(models(1:3,1)'*models(1:3,2:end));
    ground = 1;
    index = 2:4;
    up = find(normal_dp == max(normal_dp))+1;
    index(index==up) = [];
    left = index(1); right = index(2);
    pg = pc(pc(:,4)==ground,1:3);
    pu = pc(pc(:,4)==up,1:3);
    pg_homo = [pg ones(size(pg,1),1)];
    pu_homo = [pu ones(size(pu,1),1)];
    h1 = abs((models(:,ground)'*pu_homo')/norm(models(1:3,ground)));
    h2 = abs((models(:,up)'*pg_homo')/norm(models(1:3,up)));
    h = (sum(h1)+sum(h2))/(length(h1)+length(h2));
    %% Calculate length and width
    point0 = models(1:3,2:4)'\-models(4,2:4)';
    dir_l = cross(models(1:3,ground),models(1:3,left));
    dir_l = dir_l/norm(dir_l);
    dir_r = cross(models(1:3,ground),models(1:3,right));
    dir_r = dir_r/norm(dir_r);
    v_u = pu - repmat(point0',size(pu,1),1);
    d_l = abs(v_u*dir_l);
    d_r = abs(v_u*dir_r);
    d1 = max(d_l);
    d2 = max(d_r);
    l = max(d1,d2);
    w = min(d1,d2);
end