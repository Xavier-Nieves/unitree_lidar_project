#!/bin/bash
# Wrapper script to run Docker from project root

set -e

cd "$(dirname "$0")/docker"

case "$1" in
    build)
        echo "Building Docker image..."
        docker-compose build unitree-l1-ros2
        ;;
    up|start)
        echo "Starting Docker container..."
        xhost +local:docker 2>/dev/null || true
        docker-compose up -d unitree-l1-ros2
        echo "Container started. Access with: ./docker-run.sh shell"
        ;;
    shell|bash)
        echo "Connecting to container..."
        docker exec -it unitree_l1_ros2 bash
        ;;
    stop)
        echo "Stopping container..."
        docker-compose down
        ;;
    clean)
        echo "Removing container and image..."
        docker-compose down
        docker rmi unitree-l1-lidar:ros2-humble 2>/dev/null || true
        ;;
    *)
        echo "Usage: ./docker-run.sh {build|start|shell|stop|clean}"
        echo ""
        echo "Commands:"
        echo "  build  - Build the Docker image"
        echo "  start  - Start the container in background"
        echo "  shell  - Open bash shell in running container"
        echo "  stop   - Stop the container"
        echo "  clean  - Remove container and image"
        exit 1
        ;;
esac