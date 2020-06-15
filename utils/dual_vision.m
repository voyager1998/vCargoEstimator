% pt3d_ir in IR frame
% irTrgb * pt3d_rgb = pt3d_ir
% pt2d_ir: 1 * 2
% dx, dy: rgb camera relative to IR camera
function pt3d_ir = dual_vision(C_ir, C_rgb, pt2d_ir, pt2d_rgb, dx, dy)
%     syms x y z
    % IR frame:
    xoverz = (pt2d_ir(1)-C_ir(1,3))/C_ir(1,1);
    yoverz = (pt2d_ir(2)-C_ir(2,3))/C_ir(2,2);
    
    % RGB frame:
    uoverz = (pt2d_rgb(1)-C_rgb(1,3))/C_rgb(1,1);
    voverz = (pt2d_rgb(2)-C_rgb(2,3))/C_rgb(2,2);
    
    dxoverz = xoverz - uoverz;
    z = 1/dxoverz*dx;
    x = xoverz*z;
    y = yoverz*z;
    pt3d_ir = [x;y;z];
%     eqns = [x/z == xoverz, y/z == yoverz, u/w == uoverw, v/w == voverw, ...
%         irTrgb * [u;v;w;1] == [x;y;z;1]];
%     S = solve(eqns,[x y z])
%     eqns = [x/z == xoverz, y/z == yoverz, (x-dx)/z == uoverw, (y-dy)/z == voverw];
end
