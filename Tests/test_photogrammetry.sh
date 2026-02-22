#!/bin/bash
# Quick test for photogrammetry (simple OpenCV version)

cd /home/dronepi/unitree_lidar_project/RPI5

echo "Starting photogrammetry test..."
echo "Will capture 5 images at 2 second intervals"

# Start in background with short interval
timeout 10s ./run_photogrammetry.sh 2.0

# Check results
echo ""
echo "Captured images:"
ls -lh photogrammetry_captures/ 2>/dev/null | tail -5
echo ""
echo "Total images: $(ls photogrammetry_captures/*.jpg 2>/dev/null | wc -l)"
