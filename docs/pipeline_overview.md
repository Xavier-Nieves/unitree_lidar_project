# Mapping Pipeline Overview

## Pipeline Stages

```
Raw Cloud → Clean → Mesh → Trajectory → Camera Poses → Texture → Refine → Output
```

### 1. Cloud Cleaning (`mapping/cloud_cleaner.py`)
- Statistical outlier removal (configurable neighbors + std ratio)
- Voxel downsampling (configurable voxel size)
- Input: cloud_raw.pcd → Output: cloud_clean.pcd

### 2. Mesh Generation (`mapping/mesh_generator.py`)
- Normal estimation (KD-tree hybrid search)
- Consistent normal orientation
- Poisson surface reconstruction (configurable depth)
- Low-density vertex removal
- Connected component filtering
- Input: cloud_clean.pcd → Output: mesh.obj

### 3. Trajectory Extraction (`mapping/trajectory_extractor.py`)
- Load poses from TrajectoryLogger JSON output
- Or extract from ROS 2 bag file (/Odometry topic)
- Subsample to reduce redundancy
- Input: trajectory.json → Output: pose list

### 4. Camera Pose Building (`mapping/camera_pose_builder.py`)
- Load LiDAR-to-camera extrinsic calibration
- Transform SLAM poses (LiDAR frame) to camera frame
- Input: SLAM poses + calibration → Output: camera_poses.json

### 5. Texture Projection (`mapping/spherical_projector.py`)
- Direct equirectangular projection (no virtual cameras)
- For each mesh vertex: find direction from nearest camera, sample color
- Distance-weighted blending for overlapping views
- Input: mesh + images + camera_poses → Output: colored mesh

### 6. Mesh Refinement (`mapping/mesh_refiner.py`)
- Laplacian color smoothing to reduce texture seams
- Brightness normalization
- Input: colored mesh → Output: refined mesh

## Configuration

All parameters in `config.yaml`:

```yaml
mapping:
  cloud_cleaning:
    outlier_neighbors: 20
    outlier_std_ratio: 2.0
    voxel_size: 0.02
  meshing:
    method: poisson
    poisson_depth: 9
  texturing:
    projection: spherical
    blend_overlapping: true
```

## Running Individual Stages

The pipeline runs all stages automatically via:
```bash
python3 main.py process --session YYYYMMDD_HHMMSS
```

Individual stages can be imported and run standalone:
```python
from mapping.cloud_cleaner import clean_point_cloud
from mapping.mesh_generator import generate_mesh
```
