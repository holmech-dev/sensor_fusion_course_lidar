# Lidar Obstacle Detection

Point cloud processing pipeline for detecting obstacles in streamed lidar data. Voxel grid downsampling and region cropping to cut the frame to the drivable area, RANSAC plane segmentation to separate road surface from obstacles, then Euclidean clustering to group the remaining returns into individual vehicles with bounding boxes.

RANSAC plane fitting and the clustering search structure are implemented directly rather than called from PCL, with the PCL equivalents retained for comparison.

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

## Pipeline

1. **Filter** — Voxel grid at 0.1 m, region of interest bounded to −15 to +30 m longitudinal, ±6.5 m lateral. Ego-vehicle roof returns cropped separately.
2. **Segment** — RANSAC plane fit, 100 iterations, 0.2 m distance threshold. Inliers are road, outliers are obstacles.
3. **Cluster** — Euclidean clustering at 0.5 m tolerance.
4. **Box** — Axis-aligned bounding box per cluster.

## Parameter choices

The RANSAC distance threshold is the sensitive one. Tighter and the road fragments into false obstacles, looser and the lower body of a nearby vehicle is absorbed into the road plane. Cluster tolerance trades the same way — above ~0.6 m adjacent vehicles merge into one box, below ~0.4 m a single car splits.

## Limitations

Axis-aligned boxes overestimate the footprint of any vehicle at an angle to the ego heading. Each frame is processed independently, so there is no tracking, velocity estimate or persistence through occlusion. Parameters are fixed and tuned for this scene — point density falls off with range, so a single cluster tolerance becomes less reliable towards the far edge of the cropped region at 30 m.

## Build

Requires PCL >= 1.11 and a C++14 compiler.

```bash
git clone https://github.com/holmech-dev/sensor_fusion_course_lidar.git
cd sensor_fusion_course_lidar
mkdir build && cd build
cmake ..
make
./environment
```

#### Build PCL from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_macosx.html#compiling-pcl-macosx)

---
Built on the starter framework from Udacity's Sensor Fusion Nanodegree.
