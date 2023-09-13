------------------------------
-    GPR LAB PROGRESS LOG    -
------------------------------
	
NOTE: the exercises EXCEPT THE 1st ONE have had their mains modified
	  so a default model (the moai) is loaded by default, so passing the mesh arguments is optional - at least to execute the program. 
	  To make this work, please put the mesh folders on the same root as the numbered files. In the .zip, those come BUT are empty (so you know where to copypaste them or link them)


Exercises sorted by session 1..5

01-PCA
 [Y] PCA normals 

02-ICP
 [Y] Cloud setup
 [Y] ICP (one step)
 [?] ICP (multi-step)
 [Y] Border points [the borders appear when the maxDelta is *smaller* than an arbitrary value, though]
 [Y] Correspondence

03-reconstruction
 [Y] Simple Distance
 [N] RBF function

04-curvatures
 [Y] Init Monge patch 
 [Y] Principal curvatures

05-smoothing
 [Y] Iterative Laplacian
 [Y] Iterative Bilaplacian
 [Y] Iterative LambdaNu
 [N] Global Laplacian (there's a bit of code but the commented part is as such because it won't compile)
 [ ] Global Bilaplacian

-------------------------------------------------------------------------------
LEGEND:
  'Y' = Implemented, and working as it should
  '?' = Implemented, but something is amiss (and the fix is probably a small one)
  'N' = Implemented, but does not work or compile 
  ' ' = Not implemented :(
-------------------------------------------------------------------------------