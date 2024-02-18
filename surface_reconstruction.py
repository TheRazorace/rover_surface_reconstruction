import open3d as o3d 
import numpy as np
import matplotlib.pyplot as plt
import pickle

def process_pc(pcd):

    # Downsample the point cloud for better performance and visualization
    pcd = pcd.voxel_down_sample(voxel_size=0.02)

    #Statistical outlier removal 
    pcd, index = pcd.remove_statistical_outlier(nb_neighbors=50, std_ratio=1.0)
    #inlier_cloud = pcd.select_by_index(index).paint_uniform_color([0.8,0.8,0.8])
    #outlier_cloud = pcd.select_by_index(index, invert = True).paint_uniform_color([1,0,0])

    #Radius outlier removal
    #pcd, index = pcd.remove_radius_outlier(nb_points = 16, radius = 0.07)

    # Normalize the point cloud for consistent scale
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    #pcd.orient_normals_consistent_tangent_plane(100)

    return pcd

def visualize(vis_objects):
    # Access the rendering options and set the point size
    point_size = 0.01

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    for obj in vis_objects:
        vis.add_geometry(obj)
    render_option = vis.get_render_option()
    render_option.point_size = point_size

    # # Visualize the refined point cloud
    vis.run()
    vis.destroy_window()

    return

def surface_reconstructor_alpha(pcd):

    #Alpha shape reconstruction
    poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, 0.1)
    bbox = pcd.get_axis_aligned_bounding_box()
    p_mesh_crop = poisson_mesh.crop(bbox)

    o3d.visualization.draw_geometries([p_mesh_crop])

    return

def surface_reconstructor_bpa(pcd):

    #radius determination
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 3 * avg_dist

    #computing the mehs
    bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd,o3d.utility.DoubleVector([radius, radius * 2]))

    #decimating the mesh
    #dec_mesh = bpa_mesh.simplify_quadric_decimation(100000)

    o3d.visualization.draw_geometries([bpa_mesh], mesh_show_back_face = True)

    return

def surface_reconstructor_poison(pcd):

    #computing the mesh
    poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9, width=0, scale=1.1, linear_fit=False)[0]
    #cropping
    bbox = pcd.get_axis_aligned_bounding_box()
    p_mesh_crop = poisson_mesh.crop(bbox)
    # p_mesh_crop.compute_triangle_normals()
    o3d.visualization.draw_geometries([p_mesh_crop], mesh_show_back_face = True)

    return

if __name__ == "__main__":
    
    print("Loading point cloud...")
    ply_path = "/Users/ioannisdasoulas/Desktop/Various-Projects/ROS/rtab_refinement/integrated.ply"
    #poses_path = "/Users/ioannisdasoulas/Desktop/Various-Projects/ROS/rtab_refinement/poses.txt"
    
    pcd = o3d.io.read_triangle_mesh(ply_path)
    #poses = np.loadtxt("poses.txt")
    o3d.visualization.draw_geometries([pcd], mesh_show_back_face = True)

    # print("Performing point cloud processing...")
    # pcd = process_pc(pcd)

    # print("Performing surface reconstruction...")
    # # #surface_reconstructor_alpha(pcd)
    # # #surface_reconstructor_bpa(pcd)
    #surface_reconstructor_poison(pcd)
    

