function[I, u, rc] = planes_intersection(n1, M1, n2, M2, verbose)
% planes_intersection : function to compute the intersection
% between P1(M1,n1) and P2(M2,n2) planes of the 3D space.
%
% Author & support : nicolas.douillet (at) free.fr, 2019.
%
%% Syntax
%
% [I, u, rc] = planes_intersection(n1, M1, n2, M2);
% [I, u, rc] = planes_intersection(n1, M1, n2, M2, verbose);
%
%% Description
%
% [I, u, rc] = planes_intersection(n1, M1, n2, M2) compute the
% intersection between the planes P1 and P2 defined by (M1,n1) and (M2,n2).
% The value of the return code rc provides the nature of the intersection
% (line, plane, or void). When the intersection is a line, its description
% is given by the point I and the director vector u.
%
% [I, u, rc] = planes_intersection(n1, M1, n2, M2, verbose) enables
% verbose mode when verbose is set to logical true or numeric 1,
% and disables it when it is set to logical *false or numeric *0.
%
%% Principle : Descartes plane equation & 3D line parametric equation
%
% - (1) : line director vector is the cross product of n1 & n2.
%
% - (2) : looking for a point belonging to the line (common between (M1,n1)
% and (M2,n2) planes.
%
% Solving the system :
% * a1+x + b1*y + c1*z + d1 = 0 (P1)
% * a2+x + b2*y + c2*z + d2 = 0 (P2)
%
% with Kramer technique. 
%
%% Inputs arguments
%
% - M1 : double numeric row or column vector, a point belonging to P1. numel(M1) = 3.
%
% - n1 : double numeric row or column vector, one P1 normal vector. numel(n1) = 3.
%
% - M2 : double numeric row or column vector, a point belonging to P2. numel(M2) = 3.
%
% - n2 : double numeric row or column vector, one P2 normal vector. numel(n2) = 3.
%
% - verbose : either logical *false/true or numeric *0/1. 
%
%% Output arguments
%
% - I : double numeric vector, one point belonging to the line
%       intersection. size(I) = size(n1).
%
% - u : double numeric vector, one line director vector.
%       size(u) = size(n1).
%
% - rc : return code, numeric integer in the set {1,2,3}.
%        1 = line intersection,
%        2 = void / [] intersection
%        3 = plane intersection, P2 = P1
%
%        rc return code is necessary to distinguish between cases where P1 and P2 intersecton
%        is a line and where it is the plane itself (P1 and P2 coincide).
%
%% Example #1 : intersection is a line
% n1 = [1 1 1];
% M1 = ones(1,3)/3;
% n2 = [-1 1 0];
% M2 = [1 1 0];
% [I, u, rc] = planes_intersection(n1, M1, n2, M2)
% % expected : u = k*[1 1 -2], rc = 1
%
%% Example #2 : intersection is void : P1 // P2
%
% n1 = [0 0 1];
% M1 = n1;
% n2 = -n1;
% M2 = n2;
% [I, u, rc] = planes_intersection(n1, M1, n2, M2, true)
% % expected : I = [], u = [], rc = 2
%
%% Example #3  intersection is a plane : P2 = P1
% n1 = [1 0 0];
% M1 = n1;
% n2 = [-2 0 0];
% M2 = M1;
% [I, u, rc] = planes_intersection(n1, M1, n2, M2, true)
% % expected : rc = 3


%% Input parsing
assert(nargin > 3,'Not enough input arguments.');
assert(nargin < 6,'Too many input arguments.');

if nargin < 5
   verbose = false; 
end

% Check vectors have the same format
assert(isequal(size(n1,1),size(n2,1),size(n1,1),size(M1,1),size(n1,1),size(M2,1)),'M1,n1,M2,n2 input arguments must have the same size.');

d1 = -dot(n1,M1); % -a1*x1 - b1*y1 - c1*z1
d2 = -dot(n2,M2); % -a2*x2 - b2*y2 - c2*z2

u = cross(n1,n2);

if norm(u) == 0 % (M1,n1) = (M2,n2) or (M1,n1) // (M2,n2)
   
    if (dot(n1,M2) + d1) == 0 && (dot(n2,M1) + d2) == 0 % (a1*M2(1) + b1*M2(2) + c1*M2(3) + d1) == 0
        
        if verbose
            disp('Planes (M1,n1) and (M2,n2) are actually one unique same plane : (I,u).');
        end
        
        I = M1;
        u = M2 - M1;
        rc = 3;
        
    else
        
        if verbose
            disp('Planes (M1,n1) and (M2,n2) are strictly parallel. Their intersection is the empty set.');
        end
        
        I = [];
        u = [];
        rc = 2;
        
    end
    
else 
          
     dir = find((abs(u) == max(abs(u))));     
     dir = dir(1);
     
     % => the line does exist in this direction, and then it can be set to t = 0.
     
     switch dir
         
         case 1 % setting : x = 0
             
             dx0y = (n1(3)*d2 - n2(3)*d1); % c1*d2 - c2*d1
             dx0z = (n2(2)*d1 - n1(2)*d2); % b2*d1 - b1*d2
             
             xI = 0;           
             yI = dx0y/u(1); 
             zI = dx0z/u(1);
             
         case 2 % setting : y = 0
             
             dxy0 = (n1(3)*d2 - n2(3)*d1); % c1*d2 - c2*d1
             dy0z = (n2(1)*d1 - n1(1)*d2); % a2*d1 - a1*d2
             
             xI = -dxy0/u(2);
             yI = 0;
             zI = -dy0z/u(2);
             
         case 3 % setting : z = 0
             
             dxz0 = (n1(2)*d2 - n2(2)*d1); % b1*d2 - b2*d1
             dyz0 = (n2(1)*d1 - n1(1)*d2); % a2*d1 - a1*d2
             
             xI = dxz0/u(3);
             yI = dyz0/u(3);
             zI = 0;                         
             
     end
     
     I = zeros(size(M1));
     I(1) = xI;
     I(2) = yI;
     I(3) = zI;
     
     rc = 1;
     
     u=u/norm(u);
     
end


end