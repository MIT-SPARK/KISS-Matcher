import struct
import sys

import numpy as np
import open3d as o3d


def bin_to_pcd(bin_file_name):
    size_float = 4
    list_pcd = []
    with open(bin_file_name, "rb") as file:
        byte = file.read(size_float * 4)
        while byte:
            x, y, z, _ = struct.unpack("ffff", byte)
            list_pcd.append([x, y, z])
            byte = file.read(size_float * 4)
    np_pcd = np.asarray(list_pcd)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_pcd)
    return pcd


def main(binFileName, pcd_file_name):
    pcd = bin_to_pcd(binFileName)
    o3d.io.write_point_cloud(pcd_file_name, pcd)


if __name__ == "__main__":
    a = sys.argv[1]
    b = sys.argv[2]
    main(a, b)
