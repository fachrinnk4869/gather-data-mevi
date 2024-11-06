import unittest
import pypcd4 as pypcd
import os
import numpy as np


class TestSensor(unittest.TestCase):
    def setUp(self):

        # Path to an existing .pcd file to test (ensure this path is valid)
        self.lidar_file = "lidar.pcd"

        # Replace this with the directory and file name for your test
        self.file_path = "depth_camera.npy"

        # Load the point cloud data
        self.pt_cloudx = np.load(self.file_path)

    def test_lidar_data(self):
        # Check if the test PCD file exists
        self.assertTrue(os.path.exists(self.lidar_file),
                        "PCD file does not exist for testing.")

        # Call the method and capture the output using in-memory redirection
        try:
            lid_pc = pypcd.PointCloud.from_path(self.lidar_file)
            # lid_x = lid_pc.pc_data['x']
            # lid_y = lid_pc.pc_data['y']
            # lid_z = lid_pc.pc_data['z']
            # lid_intensity = lid_pc.pc_data['intensity']
            # Check if 'x', 'y', 'z', and 'intensity' fields exist
            self.assertIn('x', lid_pc.pc_data.dtype.names,
                          "Field 'x' is missing.")
            self.assertIn('y', lid_pc.pc_data.dtype.names,
                          "Field 'y' is missing.")
            self.assertIn('z', lid_pc.pc_data.dtype.names,
                          "Field 'z' is missing.")
            self.assertIn('intensity', lid_pc.pc_data.dtype.names,
                          "Field 'intensity' is missing.")

            print("Test passed: x, y, z, and intensity fields are present.")

        except Exception as e:
            self.fail(f"An error occurred during the test: {str(e)}")

    def test_point_cloud_shape(self):
        # Print the original shape of the point cloud
        print("Original shape:", self.pt_cloudx.shape)

        # Check if the point cloud has 3 dimensions (channels, height, width)
        self.assertEqual(len(self.pt_cloudx.shape),
                         3, f"Point cloud is not 3D. But {len(self.pt_cloudx.shape)}D")

        # Extract channels, height, and width from the shape
        c, h, w = self.pt_cloudx.shape
        print(f"Channels: {c}, Height: {h}, Width: {w}")

        # Resize the point cloud to (c, 128, 256) for further processing
        resized_pt_cloudx = np.resize(self.pt_cloudx, (c, 128, 256))
        print("Resized shape:", resized_pt_cloudx.shape)

        # Verify the resized shape matches the expected size
        self.assertEqual(resized_pt_cloudx.shape, (c, 128, 256),
                         "Resizing did not produce the expected shape.")


if __name__ == "__main__":
    unittest.main()
