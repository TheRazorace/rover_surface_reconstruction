import open3d as o3d 
import numpy as np
from surface_reconstruction import process_pc
import matplotlib.pyplot as plt

def visualize(pcds):

    front = [ -0.32370412536050946, -0.75850178381119415, 0.56558879336477663 ]
    lookat = [ 1.1203622817993164, -3.5053596496582031, 1.1029493808746338 ]
    up = [ 0.013827088441276031, 0.59391774118629204, 0.80440694199479024 ]
    zoom = 0.13957153320312479

    o3d.visualization.draw_geometries(pcds, front=front, lookat=lookat, up=up, zoom=zoom)

    return

def ransac_planar_segmentation(pcd):

    pt_to_plane_dist = 0.1

    plane_model, inliers = pcd.segment_plane(distance_threshold=pt_to_plane_dist, 
                                             ransac_n=3, 
                                             num_iterations=1000)
    
    [a, b, c, d] = plane_model

    inlier_cloud = pcd.select_by_index(inliers).paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd.select_by_index(inliers, invert=True).paint_uniform_color([0.6, 0.6, 0.6])
    # visualize([inlier_cloud, outlier_cloud])

    return inlier_cloud, outlier_cloud

def multi_order_ransac(pcd):

    max_plane_idx = 5 #Five planes (floor + 4 walls)
    pt_to_plane_dist = 0.1 

    segment_models = {}
    segments = {}
    rest = pcd

    for i in range(max_plane_idx):
        colors = plt.get_cmap("tab20")(i)
        segment_models[i], inliers = rest.segment_plane(distance_threshold=pt_to_plane_dist,
                                                        ransac_n=3,
                                                        num_iterations=1000)
        segments[i] = rest.select_by_index(inliers).paint_uniform_color(list(colors[:3]))
        rest = rest.select_by_index(inliers, invert=True)

    visualize([segments[i] for i in range(max_plane_idx)] + [rest])

    return segments, rest

def dbscan(pcd):

    labels = np.array(pcd.cluster_dbscan(eps=0.05, min_points=20))
    max_label = labels.max()
    print("Point cloud has", max_label + 1, " clusters")

    colors = plt.get_cmap("tab10")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    visualize([pcd])

if __name__ == "__main__":
    
    print("Loading point cloud...")
    ply_path = "/Users/ioannisdasoulas/Desktop/Various-Projects/ROS/rover_surface_reconstruction/point_clouds/cloud.ply"
    pcd = o3d.io.read_point_cloud(ply_path)

    pcd_center = pcd.get_center()

    pcd, outlier_cloud = process_pc(pcd)
    #visualize(pcd)
    inliers, pcd = ransac_planar_segmentation(pcd)
    #segments, rest multi_order_ransac(pcd)
    dbscan(pcd)


	