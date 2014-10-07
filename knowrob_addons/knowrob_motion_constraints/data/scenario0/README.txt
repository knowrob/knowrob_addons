Contents:
---------
* mesh_filenames.mat
* .obj files
* frame%05.d.mat files
* solution%05d.mat files

Description:
------------

The mesh_filenames.mat contains a cell array of strings (filenames), where
each string points to a mesh file. All strings together constitute a
1-based array of mesh references.

Each frame%05d.mat file contains variables that regard the i-th tracked
frame. These variables are:
* mats: A Nx4x4x4 array that defines N 4x4 homogeneous transforms.
* issue_ids: A N-element array that associates an issue id with each transform in mats.
* instance_ids: A N-element array that associates an instance id with each transform.

The issue_ids should be used to lookup the meshes reference array, in order
to figure out which mesh each transform refers to. The instance_id can be
ignored. It refers to the ordinal of each instance of one of the referenced
meshes.

Each solution%05d.mat file contains the solution vector (state_i) for the i-th frame.
The solution is 27 + 7 * N numbers:
* The hand pose takes up 27 values, i.e. hand pose(position (3D) + orientation (4D, quaternion)),
  and articulation (5 fingers x 4 angles each (2 angles for finger base, 2 x 1 angles for finger bend))
* Each object is assummed to be rigid and is thus localized using 7-D poses (position + quaternion)
In the exampel shown there are 4 objects, hence 27 + 4 * 7 = 55 parameters per frame.
