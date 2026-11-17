import unittest
from arm_practice.scene_coverage import audit_scene_coverage

class SceneCoverageTests(unittest.TestCase):
    def test_invisible_wall_must_be_represented(self):
        wall='/World/Mission/Maze/wall_edge/collision/geo_0'
        r=audit_scene_coverage(['/World/box',wall],['/World/box'])
        self.assertFalse(r['coverage_pass'])
        self.assertEqual(r['unrepresented_paths'],[wall])
    def test_parent_prim_is_not_child_geometry_coverage(self):
        self.assertFalse(audit_scene_coverage(['/World/wall/mesh'],['/World/wall'])['coverage_pass'])
    def test_equal_counts_do_not_prove_coverage(self):
        self.assertFalse(audit_scene_coverage(['/World/wall'],['/World/box'])['coverage_pass'])
    def test_empty_or_unresolved_scene_fails(self):
        self.assertFalse(audit_scene_coverage([],[])['coverage_pass'])
        self.assertFalse(audit_scene_coverage(['/World/a'],['/World/a'],unresolved_references=['maze.usd'])['coverage_pass'])
    def test_complete_inventory_is_not_motion_permission(self):
        r=audit_scene_coverage(['/World/a'],['/World/a'],reference_audit_complete=True)
        self.assertTrue(r['coverage_pass']);self.assertFalse(r['motion_authorized'])
    def test_malformed_inventory_rejected(self):
        for paths in ('/World/a',['relative'],['/'],['/World/a','/World/a']):
            with self.assertRaises(ValueError):audit_scene_coverage(paths,[])

    def test_unchecked_references_cannot_pass_by_default(self):
        self.assertFalse(audit_scene_coverage(['/World/a'],['/World/a'])['coverage_pass'])
        self.assertFalse(audit_scene_coverage(['/World/a'],['/World/a'],reference_audit_complete=True,unresolved_references=['missing.usd'])['coverage_pass'])
    def test_truthy_audit_claim_is_rejected(self):
        for value in (1, 'yes', None):
            with self.assertRaises(ValueError):
                audit_scene_coverage(['/World/a'],['/World/a'],reference_audit_complete=value)

    def test_usd_inspection_includes_hidden_instanced_collision(self):
        import tempfile
        from pathlib import Path
        try:
            from pxr import Usd, UsdGeom, UsdPhysics
        except ImportError:
            self.skipTest('USD runtime unavailable')
        from arm_practice.scene_coverage import inspect_usd_inventory
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'scene.usda';stage=Usd.Stage.CreateNew(str(path))
            UsdGeom.Xform.Define(stage,'/Library')
            cube=UsdGeom.Cube.Define(stage,'/Library/collision')
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            inst=UsdGeom.Xform.Define(stage,'/Instance').GetPrim()
            inst.GetReferences().AddInternalReference('/Library');inst.SetInstanceable(True)
            UsdGeom.Imageable(inst).MakeInvisible()
            stage.GetRootLayer().Save()
            r=inspect_usd_inventory(path)
            self.assertIn('/Instance/collision',r['native_paths'])
            self.assertTrue(r['references_pass']);self.assertFalse(r['motion_authorized'])
