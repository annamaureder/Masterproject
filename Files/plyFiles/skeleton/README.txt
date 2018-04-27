------------------------------------------------------------------------
This dataset is provided as is, without any explicit warranties. Use
of this dataset for commercial purposes, or re-distributing the 
dataset is prohibited. When using the dataset, please acknowledge 
Dragomir Anguelov and the Stanford Biomechanics Lab.
------------------------------------------------------------------------

Instructions: Unzip file poses.tar.gz. Contents: 
scans/ contains 250000 polygon meshes in binary ply format. They can
be viewed with scanalyze software.




morphs/ contains 25000 polygon meshes containing morphs done with the
Correlated Correspondence algorithm. The meshes are in ascii ply
format. They can also be viewed with scanalyze, and can be converted
with the ply2bin utility to binary ply.

All meshes in Morphs/ are in correspondence: points and polygons in
those meshes correspond. The correspondence is not ideal in all cases,
as you shall see, but it's pretty close. These can be used as a
starting point for more accurate processing. 

scapecomp/ contains the shape-completions obtained using SCAPE, also all in correspondence. 


skeleton/ contains a description of the articulated human
model learned automatically using those 70 morphs.
The contents are as follows:

a) mesh.ply  ascii ply file, of the template mesh

b) partsTree.out    an assignment of each model point to a rigid part
Format of the file is

[numModelPoints] [numParts]
List of part ids for each mesh point...

c) jointsTree.out   the location of the part joints (in mesh.ply)

Format is as follows:
[numJoints]
[jointPart1]     // parts to whom the joint belongs
[jointPart2]
jointCoordinates

d) partsGraph.out and jointsGraph.out this was the actual articulated model learned, with
18 parts, I didn't know how to place the joints there automatically,
but maybe it will still be useful for you.