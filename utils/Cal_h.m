<<<<<<< HEAD
function h = Cal_h(pc_g, pc_u, model_g, model_u)
% Calculate the distance between the two parallel planes.
% 
%% Input Arguments:
% - pc_g: points belonging to the ground plane
% - pc_u: points belonging to the upper plane
% - model_g: mathematical equation of the ground plane
% - model_u: mathematical equation of the upper plane
% 
%% Output Arguments:
% - h: height

%% Calculate height
pg_homo = [pc_g ones(size(pc_g,1),1)];
pu_homo = [pc_u ones(size(pc_u,1),1)];
h1 = abs((model_g*pu_homo')/norm(model_g(1:3)));
h2 = abs((model_u*pg_homo')/norm(model_u(1:3)));
h = (sum(h1)+sum(h2))/(length(h1)+length(h2));
=======
function h = Cal_h(model_g, model_u, pc_g, pc_u)
    %% Calculate height
    pg_homo = [pc_g ones(size(pc_g,1),1)];
    pu_homo = [pc_u ones(size(pc_u,1),1)];
    h1 = abs((model_g*pu_homo')/norm(model_g(1:3)));
    h2 = abs((model_u*pg_homo')/norm(model_u(1:3)));
    h = (sum(h1)+sum(h2))/(length(h1)+length(h2));
>>>>>>> c2a2dde69958efa2f9fdb4ffa939c788fa2a83af
end