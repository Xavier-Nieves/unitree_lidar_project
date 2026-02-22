"""Master post-processing orchestrator.

Runs the full mapping pipeline on a completed flight session:
  1. Load raw point cloud
  2. Clean (outlier removal + downsample)
  3. Generate mesh (normals + Poisson)
  4. Extract SLAM trajectory
  5. Build camera poses from extrinsic calibration
  6. Project textures from equirectangular images
  7. Refine mesh (seam smoothing, color correction)
  8. Save final textured model
"""

import os
import yaml
from pathlib import Path
from typing import Optional

from utils.logger import setup_logger
from utils import file_io

logger = setup_logger(__name__)


def run_pipeline(session_dir: str, config: dict) -> bool:
    """Run the full post-processing pipeline on a flight session.

    Args:
        session_dir: Path to the flight session directory
                     (e.g., data/flights/20260222_143000/).
        config: Full config dict (from config.yaml).

    Returns:
        True if pipeline completed successfully.
    """
    import open3d as o3d

    from mapping.cloud_cleaner import clean_point_cloud
    from mapping.mesh_generator import generate_mesh
    from mapping.trajectory_extractor import load_trajectory_json, subsample_trajectory
    from mapping.camera_pose_builder import load_extrinsic, build_camera_poses
    from mapping.spherical_projector import project_texture
    from mapping.mesh_refiner import refine_mesh

    mapping_config = config.get("mapping", {})
    processed_dir = os.path.join(session_dir, "processed")
    raw_dir = os.path.join(session_dir, "raw")
    Path(processed_dir).mkdir(parents=True, exist_ok=True)

    # --- Step 1: Load raw point cloud ---
    logger.info("=" * 60)
    logger.info("STEP 1/7: Loading raw point cloud")
    logger.info("=" * 60)

    cloud_path = os.path.join(processed_dir, "cloud_raw.pcd")
    if not os.path.exists(cloud_path):
        # Try to find it in raw/bags and convert
        logger.warning(f"cloud_raw.pcd not found at {cloud_path}")
        logger.info("Looking for bag files to extract cloud...")
        bag_dir = os.path.join(raw_dir, "bags")
        bags = list(Path(bag_dir).glob("*")) if os.path.exists(bag_dir) else []
        if bags:
            from utils.bag_tools import read_pointclouds_from_bag, merge_clouds
            clouds, _ = read_pointclouds_from_bag(str(bags[0]))
            merged = merge_clouds(clouds)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(merged[:, :3])
            o3d.io.write_point_cloud(cloud_path, pcd)
        else:
            logger.error("No raw cloud or bags found. Cannot proceed.")
            return False

    pcd = file_io.load_point_cloud(cloud_path)

    # --- Step 2: Clean point cloud ---
    logger.info("=" * 60)
    logger.info("STEP 2/7: Cleaning point cloud")
    logger.info("=" * 60)

    cleaning_config = mapping_config.get("cloud_cleaning", {})
    pcd_clean = clean_point_cloud(pcd, cleaning_config)

    clean_path = os.path.join(processed_dir, "cloud_clean.pcd")
    file_io.save_point_cloud(pcd_clean, clean_path)

    # --- Step 3: Generate mesh ---
    logger.info("=" * 60)
    logger.info("STEP 3/7: Generating mesh")
    logger.info("=" * 60)

    meshing_config = mapping_config.get("meshing", {})
    mesh = generate_mesh(pcd_clean, meshing_config)

    mesh_path = os.path.join(processed_dir, "mesh.obj")
    file_io.save_mesh(mesh, mesh_path)

    # --- Step 4: Extract trajectory ---
    logger.info("=" * 60)
    logger.info("STEP 4/7: Extracting trajectory")
    logger.info("=" * 60)

    traj_path = os.path.join(raw_dir, "metadata", "trajectory.json")
    if os.path.exists(traj_path):
        poses = load_trajectory_json(traj_path)
        poses = subsample_trajectory(poses, min_distance=1.0)
    else:
        logger.warning("No trajectory file found — skipping texture projection")
        poses = []

    # --- Step 5: Build camera poses ---
    logger.info("=" * 60)
    logger.info("STEP 5/7: Building camera poses")
    logger.info("=" * 60)

    camera_poses = []
    if poses:
        cal_file = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "config", "camera_calibration.yaml"
        )
        if os.path.exists(cal_file):
            extrinsic = load_extrinsic(cal_file)
            camera_poses = build_camera_poses(poses, extrinsic)
            cam_poses_path = os.path.join(processed_dir, "camera_poses.json")
            file_io.save_poses_json(camera_poses, cam_poses_path)
        else:
            logger.warning(f"Calibration file not found: {cal_file}")

    # --- Step 6: Texture projection ---
    logger.info("=" * 60)
    logger.info("STEP 6/7: Projecting textures")
    logger.info("=" * 60)

    images_dir = os.path.join(raw_dir, "images")
    if os.path.exists(images_dir) and camera_poses:
        image_files = sorted(
            str(p) for p in Path(images_dir).glob("*.jpg")
        ) + sorted(
            str(p) for p in Path(images_dir).glob("*.png")
        )

        if image_files:
            # Match images to nearest camera poses by count
            step = max(1, len(camera_poses) // len(image_files))
            matched_poses = camera_poses[::step][:len(image_files)]

            project_texture(mesh, image_files, matched_poses)
        else:
            logger.info("No images found for texture projection")
    else:
        logger.info("Skipping texture projection (no images or poses)")

    # --- Step 7: Refine mesh ---
    logger.info("=" * 60)
    logger.info("STEP 7/7: Refining mesh")
    logger.info("=" * 60)

    mesh = refine_mesh(mesh)

    # Save final model
    output_dir = os.path.join(processed_dir, "model_textured")
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    final_obj = os.path.join(output_dir, "model.obj")
    file_io.save_mesh(mesh, final_obj)

    logger.info("=" * 60)
    logger.info("PIPELINE COMPLETE")
    logger.info(f"Output: {output_dir}")
    logger.info("=" * 60)

    return True
