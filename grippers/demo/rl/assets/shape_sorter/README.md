# Shape sorter pieces

The six loose pieces of the shape sorter, one USD asset each, for the tactile recognition demo. Each piece is a 4 cm thick
extrusion of an outline:

| asset | outline | size (cm) | mass (g) |
|---|---|---|---|
| `piece_0_rectangle` | rectangle | 5.5 x 4.0 x 4.5 | 33 |
| `piece_1_pentagon` | pentagon | 5.3 x 5.1 x 4.0 | 43 |
| `piece_2_cross` | cross | 4.9 x 4.9 x 4.0 | 38 |
| `piece_3_cylinder` | circle | dia 5.0 x 4.0 | 43 |
| `piece_4_star` | five-pointed star | 5.0 x 4.7 x 4.0 | 29 |
| `piece_5_square` | square | 4.0 x 3.8 x 3.8 | 31 |

- **Source:** an FBX model of the toy, converted to OBJ in Blender (the OBJ holds five arrangements of the toy; the
  pieces of `shapes_pose01` are one of each type and are the source here; the sorter house is not used).
- **Frame:** Z up, metres, origin at the centre of mass, resting on its most stable face.
- **Collision:** convex decomposition (a convex hull would fill the notches of the star and the cross, which is what the pads feel).
- **Mass:** wood density 550 kg/m3. Friction 0.5.
- `piece_*.obj`: the same clean meshes (for a depth based tactile model), `poses.json`: size, volume, mass and the stable resting poses
  (quaternion w x y z, probability, height of the centre of mass above the floor), `textures/`: the diffuse atlas of the OBJ.

Regenerate with `scripts/convert_shape_sorter.py <shape_sorter.obj> <out_dir>`; check with `scripts/check_shape_sorter.py --assets <dir>`.
