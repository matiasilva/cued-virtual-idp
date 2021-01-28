# cued-virtual-idp

Collection of code and files used for L103's virtual IDP

## Development

To upload/change files in the Git repository (for a Windows system):

* Make sure you have [Git installed](https://git-scm.com/downloads)
* Use your preferred terminal application (Git will install `Git Bash` automatically or you can use Microsoft Terminal (highly recommend)) to clone the repo
  * `git clone https://github.com/matiasilva/cued-virtual-idp.git`
  * then change into that directory `cd cued-virtual-idp`
  * optionally [set up SSH access](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh) for more security/convenience
* Make your desired changes, by adding, editing or removing files in the repo
* Add those changes: `git add .` in the root directory, eg. `cued-virtual-idp`
  * the `.` adds all changes to the repo in the current folder (should be the root as above)
  * you can add only certain files by specifying the path `git add controllers/blahblah.c`
* Commit those changes with an informative message `git commit -m "Added more unicorns and cupcakes`
* Push those changes to the remote (ie, GitHub) `git push origin master`

## Contributors

Matias, Jacob, Henry, Edmund, Ilora, Howard, Richard -- L103

## License

MIT; feel free to reuse as you wish but do let us know!

Note on the coordinate system in webots, and related conventions in code:
  The default is "NUE" which means the axis directions {x, y, z} are in the direction {north, up, east}, where north and east are in the horizontal plane.
  When working in the  horizontal plane, this is counter-intuitive as the z axis is 90 clockwise from the x axis, so locating an {x, z} coordinate means going, say, along by x then DOWN by z, rather than UP by z like you would expect.
  In the controller code this coordinate system is used with the convention that angles are measured counterclockwise positive from East/the z-axis. To make this work easily, the struct 'vec' has a member function to get the bearing of the vector between -pi and pi, according to this convention. The struct also stores its values in the intuitive order - along, up: {z, x}.
  Also, the function 'MakePPMP' is used throughout to keep bearings and angles in the range -pi to pi to avoid mistakes.
