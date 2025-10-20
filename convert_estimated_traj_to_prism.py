#!/usr/bin/env python3
import argparse
import os
import sys
import glob
import numpy as np
from scipy.spatial.transform import Rotation as R

# Predefined IMU configurations with their corresponding transformations
IMU_CONFIGS = {
    "stim320": {
        "trans": [0.0238152864, -0.0273611894, 0.3934754823],
        "quat": [0.9998814853207453, 0.009857146309366818, -0.0023588198116095025, 0.011588267709731866],  
        "description": "STIM320 IMU "
    },
    "alphasense": {
        "trans": [0.0223316807, 0.0152511962, -0.2951527335],
        "quat": [-0.012758161951109586, -0.03811358689245267, -0.012858765323913758, 0.9991092212326765],  
        "description": "Alphasense IMU"
    },
    "adis": {
        "trans": [0.0242841553, -0.1163165298, 0.3132783618],
        "quat": [0.7079809342902135, -0.7060872322742765, -0.011637884902115788, 0.008269022663757192],  
        "description": "ADIS IMU"
    },
    "anymal": {
        "trans": [-0.6408916016, -0.0003181948, -0.4385000481],
        "quat": [0.0013881329695698875, -0.005764413801578167, 0.9999711754540795, 0.004742666061404675], 
        "description": "Anymal IMU"
    }
    ,
    "cpt7": {
        "trans": [0.3088378016, -0.0383129972, -0.234944116],
        "quat": [-0.005724388155828644, -0.0013879431347362404, -0.004742721620119043, 0.9999714053840391],  
        "description": "CPT7 IMU"
    }
    # Add more IMU configurations as needed
}

def parse_vec(s, n, name):
    # Accept "a b c", "a,b,c", or mixed; collapse to floats
    parts = [p for p in s.replace(",", " ").split() if p]
    if len(parts) != n:
        raise ValueError(f"{name}: expected {n} values, got {len(parts)} ('{s}')")
    return list(map(float, parts))

def pose_to_matrix(tx, ty, tz, qx, qy, qz, qw):
    T = np.eye(4, dtype=float)
    T[:3, :3] = R.from_quat([qx, qy, qz, qw]).as_matrix()
    T[:3, 3] = np.array([tx, ty, tz], dtype=float)
    return T

def matrix_to_pose(T):
    tx, ty, tz = T[:3, 3]
    qx, qy, qz, qw = R.from_matrix(T[:3, :3]).as_quat()
    return tx, ty, tz, qx, qy, qz, qw

def build_transform(trans_vec, quat_vec):
    q = np.array(quat_vec, dtype=float)
    n = np.linalg.norm(q)
    if n == 0.0:
        raise ValueError("Transform quaternion has zero norm.")
    q /= n
    tx, ty, tz = trans_vec
    qx, qy, qz, qw = q
    return pose_to_matrix(tx, ty, tz, qx, qy, qz, qw)

def transform_file(input_path, trans_vec, quat_vec, out_suffix, copy_covariance=False):
    if not os.path.isfile(input_path):
        raise FileNotFoundError(f"Input file not found: {input_path}")

    T_delta = build_transform(trans_vec, quat_vec)

    folder, base = os.path.split(input_path)
    name, ext = os.path.splitext(base)
    output_path = os.path.join(folder, f"{name}{out_suffix}{ext if ext else '.txt'}")

    n_in, n_out = 0, 0
    has_additional_columns = False
    additional_column_count = 0
    
    with open(input_path, "r") as fin, open(output_path, "w") as fout:
        for line in fin:
            s = line.strip()
            if not s or s.startswith("#"):
                fout.write(line)
                continue

            parts = s.split()
            if len(parts) < 8:
                fout.write(line)
                continue

            try:
                n_in += 1
                t = float(parts[0])
                tx, ty, tz = map(float, parts[1:4])
                qx, qy, qz, qw = map(float, parts[4:8])

                T_pose = pose_to_matrix(tx, ty, tz, qx, qy, qz, qw)
                T_out = T_pose @ T_delta
                tx2, ty2, tz2, qx2, qy2, qz2, qw2 = matrix_to_pose(T_out)

                # Write transformed pose
                output_line = f"{t:.17f} {tx2:.17f} {ty2:.17f} {tz2:.17f} {qx2:.17f} {qy2:.17f} {qz2:.17f} {qw2:.17f}"
                
                # Copy any additional columns (like covariance data) without modification if flag is set
                if copy_covariance and len(parts) > 8:
                    if not has_additional_columns:
                        has_additional_columns = True
                        additional_column_count = len(parts) - 8
                    additional_data = " ".join(parts[8:])
                    output_line += f" {additional_data}"
                
                fout.write(f"{output_line}\n")
                n_out += 1
            except Exception:
                fout.write(line)

    # Print information about additional columns if found
    if copy_covariance and has_additional_columns:
        print(f"  Note: Preserved {additional_column_count} additional columns (e.g., covariance data)")
    elif not copy_covariance and has_additional_columns:
        print(f"  Note: Skipped {additional_column_count} additional columns (use --copy_covariance_also to preserve them)")
    
    return output_path, n_in, n_out

def find_trajectory_files(folder_path):
    """
    Recursively find all potential trajectory files in the given folder and subfolders.
    Returns files with common trajectory extensions (.txt, .traj, .log, .dat)
    """
    if not os.path.isdir(folder_path):
        raise NotADirectoryError(f"Not a directory: {folder_path}")
    
    # Common extensions for trajectory files
    extensions = ['*.txt', '*.traj', '*.log', '*.dat']
    trajectory_files = []
    
    for ext in extensions:
        # Use recursive glob to find files in all subdirectories
        pattern = os.path.join(folder_path, '**', ext)
        trajectory_files.extend(glob.glob(pattern, recursive=True))
    
    # Remove duplicates and sort
    trajectory_files = sorted(list(set(trajectory_files)))
    
    # Filter out files that are likely not trajectory files based on name patterns
    filtered_files = []
    exclude_patterns = ['_transformed', '_converted', 'backup', 'temp', 'tmp']
    
    for file_path in trajectory_files:
        filename = os.path.basename(file_path).lower()
        if not any(pattern in filename for pattern in exclude_patterns):
            # Basic check: file should have at least some content
            try:
                if os.path.getsize(file_path) > 0:
                    filtered_files.append(file_path)
            except OSError:
                continue
    
    return filtered_files

def process_folder(folder_path, trans_vec, quat_vec, out_suffix, copy_covariance=False):
    """
    Process all trajectory files in a folder and its subfolders.
    Returns a list of (input_file, output_file, processed_lines, total_lines) tuples
    """
    trajectory_files = find_trajectory_files(folder_path)
    
    if not trajectory_files:
        print(f"No trajectory files found in: {folder_path}")
        return []
    
    results = []
    print(f"Found {len(trajectory_files)} trajectory files to process:")
    
    for i, file_path in enumerate(trajectory_files, 1):
        relative_path = os.path.relpath(file_path, folder_path)
        print(f"  [{i}/{len(trajectory_files)}] {relative_path}")
        
        try:
            output_path, n_in, n_out = transform_file(file_path, trans_vec, quat_vec, out_suffix, copy_covariance)
            results.append((file_path, output_path, n_out, n_in))
            print(f"    -> Processed {n_out}/{n_in} pose lines")
        except Exception as e:
            print(f"    -> ERROR: {e}")
            results.append((file_path, None, 0, 0))
    
    return results

def list_available_imus():
    """Display available IMU configurations"""
    print("\nAvailable IMU configurations:")
    print("-" * 60)
    print(f"{'IMU Name':<15} | {'Translation':<20} | {'Quaternion':<28} | Description")
    print("-" * 60)
    for name, config in IMU_CONFIGS.items():
        trans_str = f"[{', '.join(f'{v:.3f}' for v in config['trans'])}]"
        quat_str = f"[{', '.join(f'{v:.3f}' for v in config['quat'])}]"
        print(f"{name:<15} | {trans_str:<20} | {quat_str:<28} | {config['description']}")
    print("-" * 60)

def main():
    p = argparse.ArgumentParser(description="Apply a transformation to every pose in trajectory file(s) based on IMU type. "
                               "Can process single files or recursively process all trajectory files in a folder.")
    p.add_argument("input", help="Path to input trajectory file or folder containing trajectory files "
                                 "(trajectory format: timestamp tx ty tz qx qy qz qw)")
    p.add_argument("--imu", required=True, help="IMU name to use for transformation")
    p.add_argument("--suffix", default="_transformed", help="Suffix for the output filename")
    p.add_argument("--copy_covariance_also", action="store_true", help="Copy additional columns (e.g., covariance data) without modification")
    p.add_argument("--list", action="store_true", help="List all available IMU configurations")
    args = p.parse_args()

    if args.list:
        list_available_imus()
        return

    try:
        # Check if the specified IMU exists in our configurations
        if args.imu not in IMU_CONFIGS:
            print(f"ERROR: Unknown IMU '{args.imu}'", file=sys.stderr)
            list_available_imus()
            sys.exit(1)
            
        # Get the IMU configuration
        imu_config = IMU_CONFIGS[args.imu]
        trans_vec = imu_config["trans"]
        quat_vec = imu_config["quat"]
        
        print(f"Using IMU: {args.imu} ({imu_config['description']})")
        print(f"Translation: [{', '.join(f'{v:.6f}' for v in trans_vec)}]")
        print(f"Quaternion: [{', '.join(f'{v:.6f}' for v in quat_vec)}]")
        print()
        
        # Check if input is a file or folder
        if os.path.isfile(args.input):
            # Process single file (original behavior)
            print(f"Processing single file: {args.input}")
            if args.copy_covariance_also:
                print("Covariance copying: ENABLED")
            else:
                print("Covariance copying: DISABLED (use --copy_covariance_also to enable)")
            out_path, n_in, n_out = transform_file(args.input, trans_vec, quat_vec, args.suffix, args.copy_covariance_also)
            print(f"Wrote: {out_path}")
            print(f"Processed {n_out}/{n_in} pose lines.")
            
        elif os.path.isdir(args.input):
            # Process folder recursively
            print(f"Processing folder: {args.input}")
            if args.copy_covariance_also:
                print("Covariance copying: ENABLED")
            else:
                print("Covariance copying: DISABLED (use --copy_covariance_also to enable)")
            results = process_folder(args.input, trans_vec, quat_vec, args.suffix, args.copy_covariance_also)
            
            # Summary statistics
            total_files = len(results)
            successful_files = sum(1 for _, output_path, _, _ in results if output_path is not None)
            total_poses_in = sum(n_in for _, _, _, n_in in results)
            total_poses_out = sum(n_out for _, output_path, n_out, _ in results if output_path is not None)
            
            print(f"\nSummary:")
            print(f"  Total files found: {total_files}")
            print(f"  Successfully processed: {successful_files}")
            print(f"  Failed: {total_files - successful_files}")
            print(f"  Total poses processed: {total_poses_out}/{total_poses_in}")
            
            if successful_files == 0:
                print("WARNING: No files were successfully processed!")
                sys.exit(1)
        else:
            raise FileNotFoundError(f"Input path does not exist or is neither a file nor a directory: {args.input}")
            
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()


