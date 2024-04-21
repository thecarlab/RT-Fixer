import open3d as o3d
import os
import tempfile
import shutil
import time 

def merge_and_voxelize_pcd_files(folder_path, voxel_size, batch_size=50):
    # Create a temporary directory to store intermediate files
    temp_dir = "/mnt/Data/temp/temp/temp"
    # tempfile.mkdtemp()

    pcd_files = [f for f in os.listdir(folder_path) if f.endswith('.pcd')]
    total_files = len(pcd_files)
    intermediate_files = []

    print(f'Total files to process: {total_files}')

    # Process the PCD files in batches
    for i in range(0, total_files, batch_size):
        batch_files = pcd_files[i:i + batch_size]
        batch_cloud = None
        j = 0

        # Merge PCD files in the current batch
        for filename in batch_files:
            start_time = time.time()  # Start time measurement

            # Process each PCD file
            cloud = o3d.io.read_point_cloud(os.path.join(folder_path, filename))
            end_time = time.time()  # End time measurement
            time_taken = end_time - start_time  # Calculate time taken
            print(f'[{j}] Load {filename} in {time_taken:.4f} seconds.')

            start_time = time.time()  # Start time measurement
            batch_cloud = cloud if batch_cloud is None else batch_cloud + cloud

            end_time = time.time()  # End time measurement
            time_taken = end_time - start_time  # Calculate time taken
            print(f'[{j}] Processed {filename} in {time_taken:.4f} seconds.')
            j = j + 1

        # Apply voxel grid filter to the merged batch and save to temp directory
        if batch_cloud is not None:
            voxel_down_pcd = batch_cloud.voxel_down_sample(voxel_size=voxel_size)
            intermediate_file_path = os.path.join(temp_dir, f'intermediate_{i//batch_size}.pcd')
            o3d.io.write_point_cloud(intermediate_file_path, voxel_down_pcd)
            # intermediate_files.append(intermediate_file_path)

        remaining_files = total_files - (i + batch_size)
        if remaining_files < 0:
            remaining_files = 0
        print(f'{remaining_files} files left to process...')

    # # Merge all intermediate results
    # final_merged_cloud = None
    # for file_path in intermediate_files:
    #     cloud = o3d.io.read_point_cloud(file_path)
    #     final_merged_cloud = cloud if final_merged_cloud is None else final_merged_cloud + cloud
    #     # Delete the intermediate file to save disk space
    #     os.remove(file_path)

    # # Save the final merged and voxelized cloud
    # output_file = os.path.join(folder_path, 'final_merged_and_filtered.pcd')
    # o3d.io.write_point_cloud(output_file, final_merged_cloud)

    # print(f'Final merged and voxelized point cloud saved to {output_file}')

    # # Delete the temporary directory
    # shutil.rmtree(temp_dir)

# Usage
folder_path = '/mnt/Data/temp/temp'  # Replace with the path to your folder of PCD files
voxel_size = 0.01  # Adjust the voxel size as needed
merge_and_voxelize_pcd_files(folder_path, voxel_size)