# Post-Processing Pipeline

**Document:** 4 of 9
**Repo path:** `docs/post_processing.md`
**Last updated:** April 2026
**Project:** DronePi — Autonomous LiDAR Mapping Drone — UPRM Capstone

---

## Table of Contents

1. [Overview](#1-overview)
2. [Trigger Chain](#2-trigger-chain)
3. [Mesh Pipeline — Stage Reference](#3-mesh-pipeline--stage-reference)
4. [Texture Pipeline — Stage Reference](#4-texture-pipeline--stage-reference)
5. [Ortho Pipeline — Stage Reference](#5-ortho-pipeline--stage-reference)
6. [LED and Buzzer Feedback](#6-led-and-buzzer-feedback)
7. [CLI Reference](#7-cli-reference)
8. [Output File Reference](#8-output-file-reference)
9. [Performance Benchmarks](#9-performance-benchmarks)
10. [Algorithm Decisions](#10-algorithm-decisions)
11. [Environment Requirements](#11-environment-requirements)
12. [Debugging a Failed Run](#12-debugging-a-failed-run)
13. [Extending the Pipeline](#13-extending-the-pipeline)

---

## 1. Overview

After every flight session the post-processing system automatically converts
the raw rosbag recording and flight frame images into three deliverables
viewable in the browser:

- A textured 3D mesh (`textured_mesh.ply` / `mesh_final.ply`) displayed in the
  3D tab of `meshview.html` via Three.js.
- An XYZ slippy-map tile pyramid displayed in the Ortho tab of `meshview.html`
  via Leaflet.
- A point cloud (`combined_cloud.ply`) available alongside the mesh.

The entire system runs on the Raspberry Pi 5 with no cloud dependency. It is
triggered automatically by `drone_watchdog.py` on disarm via
`PostflightMonitor.trigger()` and takes approximately 30–90 seconds total
depending on scan duration and algorithm choices.

The system comprises three orchestrators, each calling independent classes:

| Orchestrator | Classes | Output |
|---|---|---|
| `postprocess_mesh.py` | `mesh_tools/` + `texture_tools/` | PLY mesh + cloud + `metadata.json` |
| `postprocess_ortho.py` | `ortho_tools/` | XYZ tile pyramid + `ortho_metadata.json` |
| `run_postflight.py` | Calls both above in sequence | Combined post-flight trigger |

All pipeline stages follow the same modular class pattern: one public method,
typed inputs, typed outputs, no side effects beyond their own output. Any stage
can be replaced, skipped, or tested in isolation without modifying any other
component.

---

## 2. Trigger Chain

```
Drone disarms
    |
    v
drone_watchdog.py detects disarm
    | stops subprocesses in order:
    |   bag recorder → Hailo node → SLAM bridge → Point-LIO
    |
    | calls PostflightMonitor.trigger()
    v
run_postflight.py --auto --skip-wait
    |
    | [1/2] waits up to 8s for bag recorder to write metadata.yaml
    |       (watchdog's GRACEFUL_KILL_S = 5s, so 8s gives comfortable margin)
    |
    +──────────────────────────────────────────────────────────────────────────┐
    |  MESH + TEXTURE PIPELINE                                                 |
    |                                                                          |
    | postprocess_mesh.py --bag <path> --auto                                  |
    |   [1/7] BagReader          extract /cloud_registered → Nx3 float32      |
    |   [2/7] MLSSmoother        denoise (Open3D MLS)                         |
    |   [3/7] GroundClassifier   SMRF split → ground / nonground              |
    |   [4/7] DTMBuilder         Delaunay 2.5D terrain mesh                   |
    |   [5/7] DSMBuilder         Ball Pivoting surface mesh                   |
    |   [6/7] MeshMerger +       combine DTM + DSM                            |
    |          TextureProjectionStage  project IMX477 frames (if available)   |
    |   [7/7] Publisher          PLY + JSON → /mnt/ssd/maps/<session>/        |
    |                            latest.json updated → browser auto-loads 3D  |
    |                                                                          |
    └──────────────────────────────────────────────────────────────────────────┘
    |
    +──────────────────────────────────────────────────────────────────────────┐
    |  ORTHO PIPELINE  [non-fatal — failure never changes mesh exit code]      |
    |                                                                          |
    | postprocess_ortho.py --session <path> --maps-dir /mnt/ssd/maps          |
    |  (only runs if flight_frames/<session>/ exists and is non-empty)         |
    |                                                                          |
    |   [1/5] FrameIngestor      read flight_frames/ + sidecar JSON           |
    |   [2/5] QualityFilter      reject blurry frames (Laplacian variance)    |
    |   [3/5] MosaicBuilder      cv2.Stitcher panoramic stitch + GPS affine   |
    |   [4/5] TileCutter         gdal2tiles XYZ pyramid (Pillow fallback)     |
    |   [5/5] OrthoPublisher     write ortho_metadata.json                    |
    |                            browser Ortho tab auto-loads on next poll     |
    |                                                                          |
    └──────────────────────────────────────────────────────────────────────────┘
    |
    v
flight_logger.py → logs/flight_history.log  (one line per session)
```

All pipeline output from both orchestrators is piped into `journalctl` via
`PostflightMonitor` and prefixed `[POSTFLIGHT]`. Monitor in real time with:

```bash
sudo journalctl -u drone-watchdog -f | grep POSTFLIGHT
```

---

## 3. Mesh Pipeline — Stage Reference

The mesh pipeline is implemented in `utils/postprocess_mesh.py` (orchestrator)
and `utils/mesh_tools/` (stages). All data between stages passes in memory as
NumPy arrays or mesh objects. Only `Publisher` (Stage 7) writes to disk.

### Stage 1 — BagReader (`mesh_tools/bag_reader.py`)

**Input:** MCAP bag directory path
**Output:** NumPy Nx3 float32 point array, session metadata dict

Reads `/cloud_registered` messages from the MCAP file using the `rosbags`
library. Does not require a live ROS 2 installation. Parses `metadata.yaml`
for session duration, frame count, and topic timestamps.

A minimum point threshold (default 50,000; configurable via `--min-points` or
`mesh.min_points_to_process` in `config.yaml`) is checked immediately after
extraction. If the count is below threshold the pipeline aborts cleanly and
writes no output rather than producing a degenerate mesh.

A point cap is applied to keep downstream processing tractable. Default caps:
500,000 points (standard mode) and 150,000 (fast mode). Both are overridable
via `config.yaml` `[mesh]` section or CLI flags.

```python
class BagReader:
    def extract(self) -> np.ndarray:  # Nx3 float32, Point-LIO map frame
```

### Stage 2 — MLSSmoother (`mesh_tools/mls_smoother.py`)

**Input:** Nx3 float32
**Output:** Nx3 float32 (denoised)

Applies Moving Least Squares smoothing via Open3D to reduce IMU vibration
noise accumulated in the point cloud during flight. Falls back to Statistical
Outlier Removal (SOR) if MLS is unavailable.

Skip with `--no-mls` for speed-critical reprocessing, or when the raw cloud
is already clean. In `--fast` mode MLS is skipped automatically.

```python
class MLSSmoother:
    def smooth(self, points: np.ndarray) -> np.ndarray:
```

### Stage 3 — GroundClassifier (`mesh_tools/ground_classifier.py`)

**Input:** Nx3 float32
**Output:** ground Nx3 float32, nonground Nx3 float32

Separates ground returns from structure and vegetation returns. This split
allows the terrain mesh (DTM) and surface mesh (DSM) to be built with
algorithms suited to each type, producing better output than meshing
everything together.

**Primary method:** SMRF (Simple Morphological Filter) via PDAL 3.5.3. SMRF
is a professional ground classification algorithm that handles slopes and
irregular terrain. Parameters are tunable via `--ground-threshold` and
`--ground-cell-size`.

**Fallback method:** Z-percentile split. If PDAL is unavailable or SMRF fails,
points below a configurable Z threshold (`--ground-height`, default 0.3 m above
minimum Z) are classified as ground. Reliable for flat open terrain.

When SMRF falls back, a warning is logged and visible in `journalctl`. Results
are acceptable for survey terrain but may misclassify structures close to grade.

```python
class GroundClassifier:
    def classify(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        # Returns (ground_points, nonground_points)
```

### Stage 4 — DTMBuilder (`mesh_tools/dtm_builder.py`)

**Input:** ground Nx3 float32
**Output:** trimesh.Trimesh

Builds a Digital Terrain Model from ground points using Delaunay 2.5D
triangulation. Treats the point cloud as a height field projected onto the XY
plane and triangulates without requiring surface normals.

Process:

1. Project ground points onto the XY plane
2. Grid binning — median Z per cell at the configured resolution (`--grid-res`, default 0.10 m; doubled in fast mode)
3. Delaunay triangulation of the 2D grid
4. Long-edge gap filtering — removes stretched triangles across data voids

DTM is the fastest meshing stage and consistently produces watertight geometry.

```python
class DTMBuilder:
    def build(self, ground_points: np.ndarray) -> trimesh.Trimesh:
```

### Stage 5 — DSMBuilder (`mesh_tools/dsm_builder.py`)

**Input:** nonground Nx3 float32
**Output:** open3d.geometry.TriangleMesh

Builds a Digital Surface Model from non-ground points using the Ball Pivoting
Algorithm (BPA). BPA rolls a virtual sphere across the point cloud surface and
creates triangles wherever the sphere simultaneously contacts three points.

BPA radius is estimated automatically from point density when not specified
(`--bpa-radius`). Input point count is capped at `--max-bpa-pts` (default
80,000 standard, 50,000 fast) to bound runtime.

SLAM drift inflates the effective point spacing on longer scans, causing BPA
to select a larger radius and produce a coarser mesh. This is expected. Pass
`--bpa-radius 0.05` or `--use-poisson` to override when this occurs.

```python
class DSMBuilder:
    def build(self, nonground_points: np.ndarray) -> o3d.geometry.TriangleMesh:
```

### Stage 6 — MeshMerger (`mesh_tools/mesh_merger.py`)

**Input:** DTM trimesh, DSM Open3D mesh
**Output:** combined trimesh.Trimesh

Merges the terrain mesh and surface mesh into a single unified output. The
merged mesh is `mesh_final.ply`. Both constituent meshes are also written
separately as `mesh_dtm.ply` and `mesh_dsm.ply`.

```python
class MeshMerger:
    def merge(self, dtm: trimesh.Trimesh,
              dsm: o3d.geometry.TriangleMesh) -> trimesh.Trimesh:
```

### Stage 7 — Publisher (`mesh_tools/publisher.py`)

**Input:** mesh, cloud points, session metadata, DTM, DSM
**Output:** PLY files + JSON manifests on disk

Writes all outputs to `/mnt/ssd/maps/<session_id>/`. Updates
`/mnt/ssd/maps/latest.json` to point to the new session, triggering the
browser viewer to auto-load within 10 seconds.

```python
class Publisher:
    def publish(self, mesh, cloud_pts, session_id, bag_path,
                bag_meta, elapsed_s, dtm_mesh, dsm_mesh) -> str:
        # Returns output directory path
```

---

## 4. Texture Pipeline — Stage Reference

The texture pipeline is implemented in `utils/texture_tools/`. It runs as
Stage 6 inside `postprocess_mesh.py` — between `MeshMerger` and `Publisher`.
Failure is non-fatal: if texture projection fails for any reason, the grey
(uncoloured) `mesh_final.ply` is still published.

### Architecture

Texture projection uses two sequential passes over the rosbag:

**Pass 1 — Pose loading.** `PoseInterpolator` reads all messages on the
`/aft_mapped_to_init` SLAM odometry topic from the bag and builds a sorted
timestamp array. This requires all poses to be available before processing
begins, because interpolation requires a pose bracket on both sides of each
camera timestamp.

**Pass 2 — Frame projection.** For each `/arducam/image_raw` message in the
bag, the projector:

1. Queries `PoseInterpolator.get_pose_at(timestamp)` using SLERP interpolation
   to find the exact drone pose when the shutter fired.
2. Computes the camera world pose by applying the extrinsic transform:
   `T_cam_world = T_drone_world × T_cam_body`
3. Transforms each mesh vertex from world frame to camera frame.
4. Projects each vertex to a pixel coordinate using the pinhole model:
   `u = fx * (X_c / Z_c) + cx`, `v = fy * (Y_c / Z_c) + cy`
5. Applies Z-buffer occlusion culling via Open3D ray casting — vertices
   occluded by other geometry are excluded.
6. Accumulates the sampled pixel colour into a per-vertex weighted buffer:
   `color_accum[v] += image[int(v_px), int(u_px)]`

After all frames, each vertex colour is finalised by dividing accumulated
colour by accumulated weight: `final_color = color_accum / max(weight, 1e-6)`.

### Texture skip conditions

Stage 6 is skipped automatically when any of the following are true. The reason
is logged and the grey mesh is published as a fallback.

| Condition | Flag / Config |
|---|---|
| `--no-texture` passed | CLI flag |
| `--no-mesh` passed (nothing to project onto) | CLI flag |
| `camera.enabled = false` in `config.yaml` | config |
| `/arducam/image_raw` topic absent from bag | auto-detected |
| `config/camera_calibration.yaml` not found | auto-detected |

### Core classes

**`CameraModel` (`texture_tools/camera_model.py`)**

Loads intrinsics and extrinsics from `config/camera_calibration.yaml`. Provides
the two methods consumed by `TextureProjector`.

```python
class CameraModel:
    def get_camera_world_pose(self, drone_world_pose: np.ndarray) -> np.ndarray:
        # drone_world_pose: 4x4 SLAM pose in world frame
        # Returns: 4x4 camera pose in world frame
        return drone_world_pose @ self.T_cam_body

    def project_point(self, point_cam: np.ndarray) -> tuple | None:
        # point_cam: (3,) in camera frame
        # Returns: (u, v) pixel ints, or None if behind camera or outside image
        if point_cam[2] <= 0:
            return None
        u = self.fx * point_cam[0] / point_cam[2] + self.cx
        v = self.fy * point_cam[1] / point_cam[2] + self.cy
        if 0 <= u < self.width and 0 <= v < self.height:
            return int(u), int(v)
        return None
```

The extrinsic body-to-camera transform `T_cam_body` is built from
`config/camera_calibration.yaml`:

```python
# Current measured extrinsic (physical measurement, code unverified)
translation: [-0.070, 0.000, 0.030]  # metres, LiDAR frame to camera frame
rotation_rpy: [0.0, 0.0, 0.0]        # radians, ZYX convention
```

**`PoseInterpolator` (`texture_tools/pose_interpolator.py`)**

Reads SLAM pose history from the bag and provides SLERP interpolation at
arbitrary timestamps. Shared between the texture pipeline and the ortho
pipeline `FrameIngestor`.

```python
class PoseInterpolator:
    def load(self) -> int:          # reads bag, returns pose count
    def get_pose_at(self, ts: float) -> np.ndarray:  # 4x4, SLERP interpolated
    def __len__(self) -> int:
    @property
    def time_range_s(self) -> float:
```

**`TextureProjector` (`texture_tools/texture_projector.py`)**

Manages per-vertex colour accumulation buffers. Accepts one camera frame at a
time via `project_frame()`. Finalises vertex colours via `finalize()`.

```python
class TextureProjector:
    def __init__(self, mesh: o3d.geometry.TriangleMesh,
                 camera: CameraModel,
                 use_distortion: bool = False,
                 angle_weighting: bool = False): ...

    def project_frame(self, image: np.ndarray,
                      camera_world_pose: np.ndarray) -> int:
        # Returns count of vertices updated from this frame

    def finalize(self) -> o3d.geometry.TriangleMesh:
        # Returns mesh with vertex_colors populated
```

Occlusion culling implementation (from `_get_visible_vertices()`):

```python
# Cast ray from camera origin toward each vertex candidate
ray = normalize(P_world - camera_origin)
hit = raycasting_scene.cast_rays(ray)
vertex_distance = norm(P_world - camera_origin)
tolerance = 0.05  # metres
if hit['t_hit'] >= vertex_distance - tolerance:
    # vertex is visible from this camera pose
```

**`TextureProjectionStage` (`texture_tools/texture_stage.py`)**

The Stage 6 interface called by `postprocess_mesh.py`. Wraps the three classes
above and handles the two-pass bag replay.

```python
class TextureProjectionStage:
    def __init__(self, calibration_path=None, use_distortion=False,
                 angle_weighting=False, max_frames=None, save_output=True): ...

    def project(self, mesh, bag_path, pose_topic="/aft_mapped_to_init",
                image_topic="/arducam/image_raw") -> o3d.geometry.TriangleMesh:
```

Parameters passed from `postprocess_mesh.py`:

```python
stage = TextureProjectionStage(
    calibration_path = "unitree_drone_mapper/config/camera_calibration.yaml",
    use_distortion   = False,   # negligible at survey altitudes
    angle_weighting  = False,   # uniform weight — faster on Pi 5
    max_frames       = args.texture_frames,  # None = all; fast mode caps to 30
    save_output      = True,    # writes textured_mesh.ply to bag dir
)
```

Output is `textured_mesh.ply` alongside `mesh_final.ply`. The browser viewer
prioritises `textured_mesh.ply` when present and falls back to `mesh_final.ply`
automatically.

---

## 5. Ortho Pipeline — Stage Reference

The ortho pipeline is implemented in `utils/postprocess_ortho.py` (orchestrator)
and `utils/ortho_tools/` (stages). It runs after the mesh pipeline via
`run_postflight.py`. Its failure never changes the mesh pipeline's exit code.

`run_postflight.py` skips the ortho pipeline entirely if
`/mnt/ssd/maps/<session_id>/flight_frames/` does not exist or contains no JPEG
files. This is the normal condition for flights without the IMX477 camera active.

### Stage 1 — FrameIngestor (`ortho_tools/frame_ingestor.py`)

**Input:** Session directory path, optional bag path for SLAM pose matching
**Output:** list of `FrameRecord`

Reads JPEG images written by `camera_capture.py` during flight from
`flight_frames/<session_id>/`. Each JPEG has an accompanying sidecar JSON
containing:

```json
{
  "ros_timestamp": 1714000000.123,
  "gps": {"lat": 18.2100, "lon": -67.1400, "alt": 45.2},
  "enu": {"x": 12.4, "y": 3.1, "z": 8.7}
}
```

`FrameIngestor` calls `PoseInterpolator` (shared with `texture_tools/`) to
match SLAM poses to each frame timestamp. Pose matching is non-fatal — frames
without a matched pose are still included in the mosaic but `pose_4x4` is
`None`. The full-path Orthorectifier (planned) requires a matched pose for
measurement-grade output.

```python
class FrameIngestor:
    def load(self) -> list[FrameRecord]:

@dataclass
class FrameRecord:
    image:        np.ndarray          # BGR image
    pose_4x4:     np.ndarray | None   # SLAM pose at frame time, or None
    gps:          tuple | None        # (lat, lon, alt)
    ros_ts:       float | None        # ROS timestamp seconds
    waypoint_idx: int
    path:         Path
    enu_z:        float | None        # SLAM altitude for GSD calculation
```

### Stage 2 — QualityFilter (`ortho_tools/quality_filter.py`)

**Input:** list of `FrameRecord`
**Output:** filtered list of `FrameRecord`

Rejects blurry frames using the Laplacian variance method. A frame is rejected
if `cv2.Laplacian(grey, cv2.CV_64F).var() < blur_threshold`. Default threshold
is 50.0. Configurable via `--blur-threshold`. If fewer than `min_frames` (5)
pass the filter, all frames are kept to prevent an empty mosaic.

```python
class QualityFilter:
    def filter(self, records: list[FrameRecord]) -> list[FrameRecord]:
```

### Stage 3 — MosaicBuilder (`ortho_tools/mosaic_builder.py`) — fast path

**Input:** list of `FrameRecord`
**Output:** numpy canvas (H, W, 3), geo bounds dict

Wraps `cv2.Stitcher_create()` in panoramic mode. Feature-based stitching
combines frames into a single canvas. GPS affine positioning is applied after
stitching via a world file (`.wld`) when GPS coordinates are available.

If `cv2.Stitcher` fails (too few features, insufficient overlap), the
`MosaicBuilder` falls back to a grid collage. The fallback is logged as a
warning and `collage_fallback: true` is written to `ortho_metadata.json`.

The fast path output is not measurement-grade — it is GPS-anchored but not
orthorectified. It is suitable for visual reference and approximate
georeferencing.

```python
class MosaicBuilder:
    def build(self, records: list[FrameRecord]) -> tuple[np.ndarray, dict]:
        # Returns (canvas, geo_bounds)
        # geo_bounds: {lat_min, lat_max, lon_min, lon_max, gps_count}
```

**Full path — Orthorectifier (planned, `--full` flag):** Replaces
`MosaicBuilder` with `Orthorectifier`, which ray-casts each frame pixel through
the LiDAR DTM from `dtm_builder.py`. This produces measurement-grade output
co-registered natively with `mesh_final.ply` in the same SLAM coordinate frame.
The `Orthorectifier` module is planned and not yet implemented.

### Stage 4 — TileCutter (`ortho_tools/tile_cutter.py`)

**Input:** stitched raster + geo bounds, output directory
**Output:** XYZ PNG tile pyramid at `ortho/tiles/{z}/{x}/{y}.png`

Wraps `gdal2tiles` (system GDAL) to produce an XYZ (Google Maps convention,
not TMS) tile pyramid. The XYZ convention matches Leaflet's default coordinate
scheme without requiring a `tms: true` option.

Falls back to a pure-Python Pillow grid splitter when `gdal2tiles` is
unavailable. Pillow fallback tiles are valid PNG but lack geographic reference.

Default zoom range: 14–19. Zoom 19 produces approximately 0.3 m/pixel tiles,
matching IMX477 GSD at 30 m AGL.

```python
class TileCutter:
    def cut(self, raster_path: Path, output_dir: Path,
            geo: dict | None) -> tuple[Path, int, int]:
        # Returns (tile_dir, zoom_min, zoom_max)
```

```bash
# gdal2tiles invocation (XYZ convention)
gdal2tiles --zoom=14-19 --xyz --processes=4 \
  --resampling=average mosaic.jpg ortho/tiles/
```

### Stage 5 — OrthoPublisher (`ortho_tools/publisher.py`)

**Input:** session metadata, tile directory, geo bounds
**Output:** `ortho_metadata.json` at `/mnt/ssd/maps/`

Writes the JSON file consumed by `meshview.html` to configure the Leaflet
slippy-map tab. The browser Ortho tab auto-loads on the next 10-second poll.

```json
{
  "session_id":       "scan_20260424_221106",
  "mode":             "fast",
  "timestamp":        "2026-04-24T22:14:32Z",
  "tile_url":         "rosbags/scan_20260424_221106/ortho/tiles/{z}/{x}/{y}.png",
  "bounds": {
    "lat_min": 18.2098, "lat_max": 18.2112,
    "lon_min": -67.1408, "lon_max": -67.1392
  },
  "zoom_min":         14,
  "zoom_max":         19,
  "frame_count":      87,
  "gps_count":        87,
  "collage_fallback": false,
  "processing_s":     41.3
}
```

The `tile_url` is a path relative to `serve.py`'s document root. `serve.py`
exposes `/rosbags/<session>/` so tiles resolve correctly from `meshview.html`
regardless of network topology.

```python
class OrthoPublisher:
    def publish(self, session_id, session_dir, tile_dir,
                zoom_min, zoom_max, mode, geo,
                frame_count, collage_fallback, processing_s) -> Path:
```

---

## 6. LED and Buzzer Feedback

The mesh pipeline writes `/tmp/postflight_status.json` via `PostflightIPC` at
each stage transition. `led_service.py` and the `PostflightBuzzerDriver` in
`drone_watchdog.py` read this file on each poll cycle and produce matching
LED and audio feedback.

### LED states

| Condition | LED State | Pattern |
|---|---|---|
| Pipeline running (any stage) | `POSTFLIGHT_RUNNING` | Yellow 1 Hz blink |
| Pipeline completed | `POSTFLIGHT_DONE` | Green + yellow solid, held 5 s |
| Pipeline crashed | `POSTFLIGHT_FAILED` | Red 3 Hz blink |

### Buzzer stage tones

The `PostflightBuzzerDriver` fires one-shot QBASIC tones when the pipeline
advances to a new named stage, on top of the background `TUNE_POSTPROCESS_ACTIVE`
heartbeat tick.

| Pipeline stage | Tone | Trigger |
|---|---|---|
| `bag_extract` | `TUNE_PF_EXTRACT` — short chirp | BagReader complete |
| `ground_classify` | `TUNE_PF_CLASSIFY` — ascending two-note | GroundClassifier complete |
| `dtm` or `dsm` | `TUNE_PF_MESH_STAGE` — medium confirm | Each mesh build complete |
| Done (exit 0) | `TUNE_POSTPROCESS_DONE` — happy chord | Pipeline finished |
| Failed (exit ≠ 0) | `TUNE_ERROR` — repeating low tone | Pipeline crashed |

The done/fail tones are latched per session — they fire once and do not repeat
if the watchdog polls the status file again after the session is complete.

### IPC file schema

```json
{
  "ts":     1714000000.123,
  "stage":  "mls",
  "failed": false,
  "done":   false
}
```

The file is written atomically (write to `.tmp` then `os.replace`) to prevent
partial reads.

---

## 7. CLI Reference

### `run_postflight.py` (top-level trigger)

```bash
# Automatic — called by watchdog on disarm
python3 run_postflight.py --auto

# Manual — process latest bag
python3 run_postflight.py

# Manual — process specific bag
python3 run_postflight.py --bag /mnt/ssd/rosbags/scan_20260424_221106

# Skip ortho pipeline
python3 run_postflight.py --no-ortho

# Fast ortho (fewer tiles, smaller stitcher input)
python3 run_postflight.py --fast-ortho

# Skip bag-close wait (bag already closed)
python3 run_postflight.py --skip-wait
```

**Exit codes:** `0` = success, `1` = no bag found, `2` = mesh pipeline failed.
Ortho pipeline failure never changes the exit code.

---

### `postprocess_mesh.py` flags

| Flag | Default | Effect |
|---|---|---|
| `--bag PATH` | required | Path to rosbag directory |
| `--auto` | off | Non-interactive mode |
| `--fast` | off | Skip MLS, aggressive downsampling, cap texture to 30 frames |
| `--debug` | off | Save intermediate PLY files to `debug/` subfolder |
| `--no-mls` | off | Skip MLS smoothing stage |
| `--no-mesh` | off | Cloud only — skip all meshing stages |
| `--no-texture` | off | Skip Stage 6 texture projection |
| `--texture-frames N` | all | Cap camera frames for texture projection |
| `--use-poisson` | off | Legacy Poisson reconstruction instead of DTM + DSM |
| `--poisson-depth N` | 8 | Octree depth for Poisson reconstruction |
| `--grid-res N` | 0.10 | DTM grid cell size in metres |
| `--mls-radius N` | 0.05 | MLS search radius in metres |
| `--bpa-radius N` | auto | Fixed BPA radius in metres |
| `--max-bpa-pts N` | 80000 | Hard cap on BPA input point count |
| `--ground-height N` | 0.3 | Z threshold for fallback ground classifier |
| `--ground-threshold N` | 0.5 | SMRF threshold parameter |
| `--ground-cell-size N` | 1.0 | SMRF cell size in metres |
| `--max-frames N` | all | Cap LiDAR frames read from bag |
| `--min-points N` | 50000 | Abort threshold — minimum points to proceed |
| `--maps-dir PATH` | /mnt/ssd/maps | Output directory |

**Common usage patterns:**

```bash
# Standard post-flight run (watchdog automatic)
python3 postprocess_mesh.py --bag /mnt/ssd/rosbags/scan_20260424_221106/ --auto

# Debug run — saves intermediate PLY at every stage
python3 postprocess_mesh.py --bag /mnt/ssd/rosbags/scan_20260424_221106/ --debug

# Fast preview — aggressive downsampling, skip MLS, cap texture to 30 frames
python3 postprocess_mesh.py --bag /mnt/ssd/rosbags/scan_20260424_221106/ --fast

# Cloud only — no mesh
python3 postprocess_mesh.py --bag /mnt/ssd/rosbags/scan_20260424_221106/ --no-mesh

# Skip texture (e.g. no camera data in bag)
python3 postprocess_mesh.py --bag /mnt/ssd/rosbags/scan_20260424_221106/ --no-texture

# Limit texture frames for faster reprocessing
python3 postprocess_mesh.py --bag /mnt/ssd/rosbags/scan_20260424_221106/ --texture-frames 50

# Fix coarse BPA mesh from SLAM drift
python3 postprocess_mesh.py --bag /mnt/ssd/rosbags/scan_20260424_221106/ --bpa-radius 0.05

# Poisson reconstruction (less sensitive to point spacing)
python3 postprocess_mesh.py --bag /mnt/ssd/rosbags/scan_20260424_221106/ --use-poisson

# Tune ground classifier for complex terrain
python3 postprocess_mesh.py --bag /mnt/ssd/rosbags/scan_20260424_221106/ \
  --ground-threshold 0.3 --ground-cell-size 0.5
```

---

### `postprocess_ortho.py` flags

| Flag | Default | Effect |
|---|---|---|
| `--session PATH` | latest bag | Session bag directory |
| `--maps-dir PATH` | /mnt/ssd/maps | Output directory |
| `--blur-threshold N` | 50.0 | Laplacian variance threshold for blur rejection |
| `--fast` | off | Smaller stitcher input, fewer tiles |
| `--full` | off | Full path (Orthorectifier) — planned, not yet implemented |
| `--frames-dir PATH` | — | Custom frames directory (ODM test path) |
| `--session-id STR` | — | Override session ID (used with --frames-dir) |
| `--zoom-min N` | 14 | Minimum tile zoom level |
| `--zoom-max N` | 19 | Maximum tile zoom level |
| `--no-ortho` | off | Skip ortho (passed via run_postflight.py) |
| `--fast-ortho` | off | Pass --fast to ortho (passed via run_postflight.py) |

```bash
# Reprocess ortho for a specific session
python3 postprocess_ortho.py --session /mnt/ssd/rosbags/scan_20260424_221106

# Reprocess with tighter blur rejection
python3 postprocess_ortho.py --session /mnt/ssd/rosbags/scan_20260424_221106 \
  --blur-threshold 80.0

# Test with a custom image directory (no SLAM bag needed)
python3 postprocess_ortho.py --frames-dir /path/to/images \
  --session-id scan_test --maps-dir /tmp/maps
```

---

## 8. Output File Reference

### Mesh pipeline outputs — `/mnt/ssd/maps/<session_id>/`

| File | Description | Viewer |
|---|---|---|
| `combined_cloud.ply` | Full cleaned point cloud | Cloud button in meshview.html |
| `mesh_final.ply` | Combined DTM + DSM | Mesh button in meshview.html |
| `textured_mesh.ply` | Mesh with per-vertex RGB (when Stage 6 succeeds) | Loaded preferentially by viewer |
| `mesh_dtm.ply` | Terrain mesh only | File selector → debug files |
| `mesh_dsm.ply` | Surface mesh only | File selector → debug files |
| `metadata.json` | Processing metadata for this session | `/api/flights` endpoint |

`latest.json` at `/mnt/ssd/maps/` is updated on every successful run and points
to the most recent session. The viewer polls this every 10 seconds.

### metadata.json structure

```json
{
  "session": "scan_20260424_221106",
  "timestamp": "2026-04-24T22:11:06",
  "duration_s": 273,
  "frame_count": 2663,
  "point_count": 3320692,
  "mesh_vertices": 25168,
  "mesh_faces": 50456,
  "watertight": true,
  "textured": true,
  "texture_frames": 87,
  "processing_time_s": 58.2,
  "algorithms": {
    "smoother":   "MLS",
    "classifier": "SMRF",
    "dtm":        "Delaunay2.5D",
    "dsm":        "BPA"
  }
}
```

### Debug outputs (`--debug` flag)

Intermediate PLY files are saved to `<bag_dir>/debug/` when `--debug` is
passed. These are not copied to `maps/`.

| File | After stage | Contents |
|---|---|---|
| `debug_1_raw.ply` | BagReader | Raw unprocessed point cloud |
| `debug_2_capped.ply` | Point cap | Cloud after max point limit |
| `debug_3_mls.ply` | MLSSmoother | Denoised cloud |
| `debug_4_ground.ply` | GroundClassifier | Ground points |
| `debug_4_nonground.ply` | GroundClassifier | Non-ground points |
| `debug_5_dtm.ply` | DTMBuilder | Terrain mesh |
| `debug_6_dsm.ply` | DSMBuilder | Surface mesh |

### Ortho pipeline outputs

| File | Location | Description |
|---|---|---|
| `ortho/tiles/{z}/{x}/{y}.png` | `<bag_dir>/ortho/` | XYZ PNG tile pyramid |
| `ortho/mosaic.jpg` | `<bag_dir>/ortho/` | Full stitched mosaic JPEG |
| `ortho_metadata.json` | `/mnt/ssd/maps/` | Leaflet config for Ortho tab |

---

## 9. Performance Benchmarks

Reference scan: `scan_20260319_230358` — handheld indoor room scan,
4 minutes 33 seconds, 3,320,692 raw points, 2,663 frames.
Platform: Raspberry Pi 5, 16 GB RAM, 3.0 GHz.

### Mesh pipeline

| Stage | Time | Notes |
|---|---|---|
| BagReader | 6.3 s | MCAP parsing |
| MLSSmoother | TBD | Benchmarked after Open3D 0.19.0 install |
| GroundClassifier (Z-pct fallback) | 5.5 s | SMRF time TBD |
| DTMBuilder | 1.4 s | 75,000 triangles |
| DSMBuilder (BPA) | ~12 s | Slower when SLAM drift inflates radius |
| MeshMerger | ~0.5 s | |
| Publisher | 0.8 s | File writes |
| **Total (without MLS, without texture)** | **~25 s** | |

### Texture pipeline

| Condition | Time | Notes |
|---|---|---|
| 30 frames (fast mode) | ~25–35 s | Two-pass bag replay, Z-buffer culling |
| All frames (full mission) | ~90–180 s | Scales linearly with frame count |

Texture stage runtime is dominated by the Open3D ray casting scene creation
(once per run) and the projection loop (linear in frame count). On Pi 5,
all-frames processing exceeds the 60-second post-flight window for longer
flights. Use `--texture-frames 50` for a quality/speed balance.

### Ortho pipeline

| Condition | Time | Notes |
|---|---|---|
| Typical 60–100 frames (fast mode) | ~30–50 s | cv2.Stitcher + gdal2tiles |
| Stitcher failure → collage fallback | ~5–10 s | Much faster |
| Full-path orthorectifier (planned) | TBD | Requires DTM first |

Mesh verified watertight: 25,168 vertices, 50,456 faces on reference scan.

---

## 10. Algorithm Decisions

### Why Delaunay 2.5D for terrain (DTM)

Delaunay 2.5D projects the point cloud onto the XY plane and triangulates it
as a height field. It is fast, always produces a closed manifold, and handles
ground points well because terrain is approximately planar over short distances.
It cannot represent overhanging surfaces, which is acceptable for terrain.

### Why Ball Pivoting for surfaces (DSM)

BPA adapts to local point density and preserves sharp edges on structures,
outperforming Poisson for architectural geometry where edge preservation matters.
Its main weakness is sensitivity to point spacing — SLAM drift spreads points
unevenly and inflates the selected radius, producing coarser meshes on longer
flights. Poisson reconstruction is available as an alternative via
`--use-poisson` when this occurs.

### Why Open3D MLS over pymeshlab

The original build used `pymeshlab` for ARM64 Python 3.12 compatibility because
Open3D had no ARM64 wheel available at that time. Open3D 0.19.0 is now available
via conda-forge and is the active smoother. `pymeshlab` remains installed as a
fallback.

### Why SMRF via PDAL

SMRF (Simple Morphological Filter) is a well-validated professional algorithm
used in production LiDAR processing tools including LAStools and PDAL itself.
It outperforms Z-threshold classification on sloped or irregular terrain by
modelling the progressive morphological difference between ground and object
returns. PDAL 3.5.3 is installed via conda-forge in the `dronepi` environment.

### Why two-pass texture projection

Texture projection uses two passes over the bag rather than one interleaved
pass. `PoseInterpolator` requires pose brackets on both sides of each camera
timestamp to perform SLERP interpolation. A single forward pass would miss the
right bracket for any camera frame that arrives before the next SLAM pose
message, producing incorrect interpolated poses. Two passes guarantee correct
interpolation for all frames.

### Why fast-path ortho over full-path ortho

The fast path (`cv2.Stitcher`) produces a stitched mosaic in approximately
30–50 seconds — within the post-flight window that completes before the next
flight is ready. The full-path Orthorectifier requires the DTM from the mesh
pipeline and ray-casts every pixel, which takes significantly longer and also
requires the mesh pipeline to have completed first. The fast path is suitable
for the visual reference use case. The full path is planned for when
measurement-grade ortho output is needed.

---

## 11. Environment Requirements

Both pipelines must run inside the `dronepi` Conda environment.

```bash
conda activate dronepi
```

### Mesh + texture pipeline requirements

| Package | Version | Role |
|---|---|---|
| open3d | 0.19.0 | MLS smoothing, BPA mesh, ray casting |
| pdal | 3.5.3 | SMRF ground classification |
| trimesh | latest | DTM mesh, merge, PLY export |
| rosbags | latest | MCAP bag reading |
| scipy | latest | Spatial operations in DTMBuilder |
| numpy | latest | All array operations |
| opencv-python | latest | Image reading in TextureProjector |
| pymeshlab | latest | Fallback mesh processing |
| pyyaml | latest | Calibration YAML loading |

### Ortho pipeline requirements

| Package | Role |
|---|---|
| opencv-python | `cv2.Stitcher`, blur detection (Laplacian) |
| piexif | GPS EXIF extraction from JPEG frames |
| Pillow | Tile fallback when gdal2tiles unavailable |
| gdal-bin (system) | `gdal2tiles` tile pyramid generation |

Install on Pi:
```bash
sudo apt install gdal-bin
pip install opencv-python piexif Pillow --break-system-packages
```

### Verify environment

```bash
conda activate dronepi

# Mesh pipeline
python3 -c "import open3d; print('Open3D:', open3d.__version__)"
python3 -c "import pdal; print('PDAL:', pdal.__version__)"
python3 -c "import trimesh; print('trimesh OK')"
python3 -c "from rosbags.rosbag2 import Reader; print('rosbags OK')"

# Texture pipeline
python3 -c "from texture_tools import CameraModel, PoseInterpolator; print('texture_tools OK')"

# Ortho pipeline
python3 -c "import cv2; print('OpenCV:', cv2.__version__)"
python3 -c "import piexif; print('piexif OK')"
gdal2tiles --version
```

---

## 12. Debugging a Failed Run

### Check the watchdog journal

All pipeline output is piped into `journalctl` via `PostflightMonitor`.

```bash
# Live — watch as pipeline runs
sudo journalctl -u drone-watchdog -f

# Filter to postflight output only
sudo journalctl -u drone-watchdog | grep POSTFLIGHT

# Check pipeline stage at time of crash
cat /tmp/postflight_status.json
```

### Bag not closed cleanly

**Symptom:** `rosbags` raises a parse error or `BagReader` returns zero points.
`metadata.yaml` is absent from the bag directory.

**Cause:** Bag recorder was killed with SIGKILL before the MCAP writer
finalised the file. `run_postflight.py` waits 8 seconds, but SIGKILL bypasses
the close sequence.

```bash
ls /mnt/ssd/rosbags/scan_YYYYMMDD_HHMMSS/
# metadata.yaml must be present — if missing, bag is corrupt

# Reprocess manually, skip the wait
python3 utils/run_postflight.py --bag /mnt/ssd/rosbags/<session> --skip-wait
```

### Stage fails with ImportError

The `dronepi` Conda environment was not active when the pipeline launched. The
systemd `ExecStart` must source the Conda environment explicitly.

```bash
# Confirm ExecStart in watchdog service:
cat /etc/systemd/system/drone-watchdog.service | grep ExecStart

# Should include:
ExecStart=/bin/bash -c "source ~/miniforge3/etc/profile.d/conda.sh && \
  conda activate dronepi && python3 drone_watchdog.py"
```

### DSMBuilder produces coarse mesh

**Cause:** SLAM drift inflated effective point spacing; BPA selected a large
radius.

```bash
# Fix with explicit smaller radius
python3 utils/postprocess_mesh.py --bag <path> --bpa-radius 0.05

# Alternative: Poisson (less sensitive to point spacing)
python3 utils/postprocess_mesh.py --bag <path> --use-poisson
```

### GroundClassifier falls back to Z-percentile

SMRF failed on the point cloud — very sparse or very dense clouds can cause
PDAL SMRF to error. The fallback is logged as a warning. Results are
acceptable for flat terrain.

```bash
# Inspect PDAL error in journal
sudo journalctl -u drone-watchdog | grep -i "smrf\|pdal\|classify"

# Manually tune SMRF parameters
python3 utils/postprocess_mesh.py --bag <path> \
  --ground-threshold 0.3 --ground-cell-size 0.5
```

### Texture stage skipped unexpectedly

Check the skip reason printed to the pipeline log:

```bash
sudo journalctl -u drone-watchdog | grep "Texture.*skip\|SKIP\|no-texture"
```

Common causes and fixes:

| Message | Fix |
|---|---|
| `topic '/arducam/image_raw' not found in bag` | Camera was not capturing during flight — check `camera_capture.py` was running |
| `calibration file not found` | Verify `config/camera_calibration.yaml` exists |
| `camera.enabled=false in config.yaml` | Set `camera.enabled: true` in `unitree_drone_mapper/config.yaml` |

### Texture stage fails at runtime

```bash
# Reprocess with limited frames to reduce memory pressure
python3 utils/postprocess_mesh.py --bag <path> --texture-frames 30

# Reprocess without texture to get the grey mesh quickly
python3 utils/postprocess_mesh.py --bag <path> --no-texture
```

### Ortho stitcher fails / collage fallback

**Symptom:** `ortho_metadata.json` shows `"collage_fallback": true`. Mosaic
looks like a grid of separate frames, not a continuous image.

**Cause:** `cv2.Stitcher` could not match features between frames — typically
from insufficient overlap, extreme nadir angle without texture variation, or
too few frames passing the blur filter.

```bash
# Relax blur filter to pass more frames
python3 utils/postprocess_ortho.py --session <path> --blur-threshold 20.0

# Reduce tile zoom to speed up tile generation
python3 utils/postprocess_ortho.py --session <path> --zoom-max 17
```

---

## 13. Extending the Pipeline

### Adding a new mesh pipeline stage

1. Create `utils/mesh_tools/your_module.py` with a single class and one public method.
2. Add an `.is_available()` classmethod if the stage depends on optional hardware or packages.
3. Import and call the class from `postprocess_mesh.py` at the appropriate position.
4. Add a `--no-your-stage` CLI flag to `postprocess_mesh.py`'s argument parser.
5. Add a `PipelineStage` enum entry to `debugger_tools/status.py`.
6. Add a `--debug` save call in `DebugSaver` if the stage produces a point cloud or mesh.
7. If the stage produces a new output file type, add a matching write path to `Publisher`.

Stage skeleton:

```python
# utils/mesh_tools/your_module.py
import numpy as np

class YourModule:

    @classmethod
    def is_available(cls) -> bool:
        try:
            import your_dependency
            return True
        except ImportError:
            return False

    def process(self, points: np.ndarray) -> np.ndarray:
        if not self.is_available():
            print("[YourModule] dependency not available — skipping")
            return points
        print("[YourModule] processing...")
        # your logic here
        return processed_points
```

### Adding a new ortho pipeline stage

Follow the same pattern in `utils/ortho_tools/`. The orchestrator is
`postprocess_ortho.py`. All stages consume and return `list[FrameRecord]` or
raster data. Add a CLI flag to `postprocess_ortho.py` to allow skipping.

### Planned future stages

| Module | Pipeline | Purpose | Blocked by |
|---|---|---|---|
| `ortho_tools/orthorectifier.py` | Ortho (full path) | Ray-cast pixels through LiDAR DTM for measurement-grade output | Implementation — requires DTM from mesh pipeline |
| `hailo/hailo_ground.py` | Mesh | Neural ground classification via Hailo-8 | Camera + hailo/ submodule integration |
| `mesh_tools/multi_bag_merger.py` | Mesh | Combine sessions from multiple battery charges | Implementation |
| `mesh_tools/storage_monitor.py` | Preflight | SSD health and voltage check as preflight section | Implementation |
