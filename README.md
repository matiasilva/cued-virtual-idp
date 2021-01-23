# cued-virtual-idp
Collection of code and files used for L103's virtual IDP

Note on the coordinate system in webots, and related conventions in code:
  The default is "NUE" which means the axis directions {x, y, z} are in the direction {north, up, east}, where north and east are in the horizontal plane.
  When working in the  horizontal plane, this is counter-intuitive as the z axis is 90 clockwise from the x axis, so locating an {x, z} coordinate means going, say, along by x then DOWN by z, rather than UP by z like you would expect.
  In the code for estimating orientation/bearing using distance sensor measurements, I use this coordinate system and use the convention that angles are measured counterclockwise positive from East/the z-axis. To make this work easily, I have made a custom arctan function that takes an x and z distance and converts into a bearing between -pi and pi, measured according to this convention. The algorithm uses this convention throughout.
  Also, the function 'MakePPMP' is used throughout to keep bearings in the range -pi to pi to avoid mistakes.
