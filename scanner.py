import numpy as np
import open3d as o3d

class Scanner:
    def __init__(self, obj_min_depth, obj_max_depth, w, h, fx, fy=None, cx=None, cy=None, extrinsics=np.array([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])):
        if fy is None:
            fy = fx
        if cx is None:
            cx = w / 2
        if cy is None:
            cy = h / 2
        self.scan = o3d.geometry.PointCloud()
        self.intrinsics = o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, cx, cy)
        self.extrinsics = extrinsics
        # self.crop_bounds = o3d.geometry.AxisAlignedBoundingBox(
        #     min_bound=(-cx * -obj_max_depth / fx, -cy * -obj_max_depth / fy, obj_min_depth),
        #     max_bound=(w - cx * obj_max_depth / fx, h - cy * obj_max_depth / fy, obj_max_depth))
        self.crop_bounds = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=(-10, -4, obj_min_depth),
            max_bound=(10, 17, obj_max_depth))
        self.rotation_center = np.asarray([0, 0, obj_min_depth + (obj_max_depth - obj_max_depth) / 2])

    def add_image(self, im, rotation):
        od = o3d.geometry.Image((im).astype(np.uint16))
        pcd = o3d.geometry.PointCloud.create_from_depth_image(od, self.intrinsics, self.extrinsics, depth_scale=1, stride=4)
        # print(np.asarray(o3d.geometry.AxisAlignedBoundingBox.create_from_points(pcd.points).get_box_points()))
        pcd = pcd.crop(self.crop_bounds)
        pcd.estimate_normals()
        pcd.orient_normals_towards_camera_location()
        pcd = pcd.rotate(o3d.geometry.get_rotation_matrix_from_axis_angle((0, -rotation, 0))) #, self.rotation_center)
        # o3d.visualization.draw_geometries([pcd])
        self.scan.points.extend(pcd.points)
        self.scan.normals.extend(pcd.normals)

    def save_mesh(self, file_name):
        # self.scan.remove_statistical_outlier(100, 0.5)
        # self.scan.estimate_normals()
        o3d.visualization.draw_geometries([self.scan])

        o3d.io.write_point_cloud(f"{file_name}.ply", self.scan)

        # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd=self.scan, radii=o3d.utility.DoubleVector(np.array([1.0, 5.0])))
        # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd=self.scan, radii=5)
        mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd=self.scan)
        # mesh, _ = self.scan.compute_convex_hull(joggle_inputs=True)
        mesh.compute_triangle_normals()
        o3d.io.write_triangle_mesh(f"{file_name}.stl", mesh)