import unittest
import numpy as np
from arm_practice.depth_volume import depth_covers_voxels, obb_volume_cells


class DepthVolumeTests(unittest.TestCase):
    def query(self, depth, center=(0.,0.,2.), half=.02):
        return bool(depth_covers_voxels([center], half, np.eye(4), depth)[0])

    def test_fully_observed_box_is_free(self):
        self.assertTrue(self.query(np.full((480,640), 4.)))

    def test_unobserved_interior_pixel_rejects_box(self):
        for value in [0., float('nan'), float('inf'), 11.]:
            depth = np.full((480,640), 4.)
            depth[240,320] = value
            self.assertFalse(self.query(depth))

    def test_interior_obstacle_between_corner_rays_rejects_box(self):
        depth = np.full((480,640), 4.)
        depth[240,320] = 1.9
        self.assertFalse(self.query(depth))

    def test_center_clearance_does_not_certify_back_corner(self):
        self.assertFalse(self.query(np.full((480,640),2.04)))

    def test_partial_fov_and_camera_plane_crossing_rejected(self):
        depth = np.full((480,640),4.)
        self.assertFalse(self.query(depth, (1.59,0.,2.), .05))
        self.assertFalse(self.query(depth, (0.,0.,.04), .02))

    def test_rigid_transform_changes_reference_coordinates_only(self):
        transform = np.eye(4)
        transform[:3,3] = [1.,2.,3.]
        self.assertTrue(depth_covers_voxels([[1.,2.,5.]], .02, transform,
                                           np.full((480,640),4.))[0])

    def test_invalid_volume_rejected(self):
        with self.assertRaises(ValueError):
            depth_covers_voxels([[0.,0.,2.]], -.01, np.eye(4), np.ones((480,640)))

    def test_exact_rectangle_excludes_only_pixels_outside_projected_bounds(self):
        depth=np.full((480,640),4.)
        depth[240,319]=0.
        center=[[.001,.0025,2.]]
        self.assertFalse(depth_covers_voxels(center,.0001,np.eye(4),depth)[0])
        self.assertTrue(depth_covers_voxels(center,.0001,np.eye(4),depth,
                                          exact_rectangle=True)[0])
        depth[240,320]=0.
        self.assertFalse(depth_covers_voxels(center,.0001,np.eye(4),depth,
                                           exact_rectangle=True)[0])

    def test_exact_rectangle_still_rejects_an_interior_obstacle(self):
        depth=np.full((480,640),4.)
        depth[240,320]=1.9
        self.assertFalse(depth_covers_voxels([[0.,0.,2.]],.02,np.eye(4),depth,
                                           exact_rectangle=True)[0])

    def test_rounded_clearance_samples_are_covered_by_whole_cells(self):
        rng = np.random.default_rng(420)
        transform = np.eye(4)
        c,s = np.cos(.7), np.sin(.7)
        transform[:3,:3] = [[c,-s,0],[s,c,0],[0,0,1]]
        transform[:3,3] = [.137,-.093,.42]
        size = np.array([.13,.06,.09])
        points = rng.uniform(-1,1,(20000,3))*size/2
        directions = rng.normal(size=(len(points),3))
        directions /= np.linalg.norm(directions,axis=1)[:,None]
        points += directions*.03
        points = points@transform[:3,:3].T+transform[:3,3]
        cells = set(map(tuple,obb_volume_cells(transform,size,.01,.03)))
        self.assertTrue(all(tuple(cell) in cells for cell in np.floor(points/.01).astype(int)))


if __name__ == '__main__':
    unittest.main()
