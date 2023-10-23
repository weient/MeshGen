import numpy as np
import torch
import math
import open3d as o3d
import trimesh
from pixel2world import pixel2world

def as_mesh(scene_or_mesh):
    """
    Convert a possible scene to a mesh.

    If conversion occurs, the returned mesh has only vertex and face data.
    """
    if isinstance(scene_or_mesh, trimesh.Scene):
        if len(scene_or_mesh.geometry) == 0:
            mesh = None  # empty scene
        else:
            # we lose texture information here
            mesh = trimesh.util.concatenate(
                tuple(trimesh.Trimesh(vertices=g.vertices, faces=g.faces)
                    for g in scene_or_mesh.geometry.values()))
    else:
        assert(isinstance(mesh, trimesh.Trimesh))
        mesh = scene_or_mesh
    return mesh

# load mesh
mesh = trimesh.load('/mnt/home_6T/public/weien/lightglue/area1/rgb.obj')
mesh = as_mesh(mesh)
assert(isinstance(mesh, trimesh.Trimesh))
# setup cam pos
h = 512
w = 1024
campos = np.array([-1, 5, 1.3])
ray_origins = np.tile(campos, (h*w, 1))
# return (h*w, 3)
ray_directions = pixel2world(w, h)

ray_origins = ray_origins[100000:100005, :]
ray_directions = ray_directions[100000:100005, :]

locations, index_ray, index_tri = mesh.ray.intersects_location(
        ray_origins=ray_origins, ray_directions=ray_directions
    )
print(locations)
print(index_ray)
print(index_tri)



