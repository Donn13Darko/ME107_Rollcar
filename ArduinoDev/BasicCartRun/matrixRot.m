%% ME107 Rotation Tests

syms x y z tx ty tz

aVec = [x; y; z];

xrot = [[1 0 0];
        [0 cos(tx) -sin(tx)];
        [0 sin(tx) cos(tx)]];

yrot = [[cos(ty) 0 sin(ty)];
        [0 1 0];
        [-sin(ty) 0 cos(ty)]];
    
zrot = [[cos(tz) -sin(tz) 0];
        [sin(tz) cos(tz) 0]
        [0 0 1]];
    
At = xrot*yrot*zrot;
Ati = At';

Ati * aVec

%% Alternate way
syms sx sy sz cx cy cz

A2 = [[cy*cz cz*sx*sy-cx*sz cx*cz*sy+sx*sz];
      [cy*sz cx*cz+sx*sy*sz cx*sy*sz-cz*sx];
      [-sy cy*sx cx*cy]];
  
A2i = A2';