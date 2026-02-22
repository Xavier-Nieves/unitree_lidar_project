#!/bin/bash
# System Verification Script
# Checks that all components are ready to use

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

print_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[✓]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[!]${NC} $1"; }
print_error() { echo -e "${RED}[✗]${NC} $1"; }

echo "================================================"
echo "  System Verification"
echo "================================================"
echo ""

PASS=0
FAIL=0

# Check ROS2
print_info "Checking ROS2 Jazzy..."
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    print_success "ROS2 Jazzy installed"
    ((PASS++))
else
    print_error "ROS2 Jazzy not found"
    ((FAIL++))
fi

# Check main executable
print_info "Checking main executable..."
if [ -x "./unitree_lidar_L1" ]; then
    print_success "unitree_lidar_L1 executable"
    ((PASS++))
else
    print_error "unitree_lidar_L1 not executable"
    ((FAIL++))
fi

# Check workspace
print_info "Checking ROS2 workspace..."
if [ -d "ros2_ws/install" ]; then
    print_success "Workspace built"
    ((PASS++))
else
    print_error "Workspace not built"
    ((FAIL++))
fi

# Check packages
print_info "Checking ROS2 packages..."
source /opt/ros/jazzy/setup.bash 2>/dev/null
source ros2_ws/install/setup.bash 2>/dev/null

if ros2 pkg list | grep -q "unitree_lidar_ros2"; then
    print_success "unitree_lidar_ros2 package"
    ((PASS++))
else
    print_error "unitree_lidar_ros2 not found"
    ((FAIL++))
fi

if ros2 pkg list | grep -q "point_lio"; then
    print_success "point_lio package"
    ((PASS++))
else
    print_error "point_lio not found"
    ((FAIL++))
fi

if ros2 pkg list | grep -q "insta360_photogrammetry"; then
    print_success "insta360_photogrammetry package"
    ((PASS++))
else
    print_error "insta360_photogrammetry not found"
    ((FAIL++))
fi

# Check LiDAR hardware
print_info "Checking LiDAR hardware..."
if ls /dev/ttyUSB* 2>/dev/null | grep -q .; then
    print_success "LiDAR detected at $(ls /dev/ttyUSB* | head -1)"
    ((PASS++))
else
    print_warning "LiDAR not detected (check connection)"
    ((FAIL++))
fi

# Check camera hardware
print_info "Checking camera hardware..."
if lsusb | grep -qi "insta"; then
    print_success "Insta360 camera detected"
    ((PASS++))
elif lsusb | grep -qi "A9 Platform"; then
    print_success "Insta360 camera detected (A9 Platform)"
    ((PASS++))
else
    print_warning "Insta360 camera not detected (check connection)"
    ((FAIL++))
fi

# Check scripts
print_info "Checking utility scripts..."
SCRIPTS=(
    "test_insta360.sh"
    "view_insta360_live.py"
    "run_photogrammetry.sh"
    "capture_insta360.sh"
)

for script in "${SCRIPTS[@]}"; do
    if [ -x "./$script" ]; then
        print_success "$script"
        ((PASS++))
    else
        print_error "$script not executable"
        ((FAIL++))
    fi
done

# Check documentation
print_info "Checking documentation..."
DOCS=(
    "README.md"
    "MASTER_INDEX.md"
    "SETUP_COMPLETE.md"
    "INSTA360_GUIDE.md"
    "PHOTOGRAMMETRY_GUIDE.md"
    "TROUBLESHOOTING.md"
)

for doc in "${DOCS[@]}"; do
    if [ -f "./$doc" ]; then
        print_success "$doc"
        ((PASS++))
    else
        print_error "$doc missing"
        ((FAIL++))
    fi
done

# Check output directories
print_info "Checking output directories..."
mkdir -p insta360_captures photogrammetry_captures
print_success "Output directories ready"
((PASS++))

echo ""
echo "================================================"
echo "  Verification Results"
echo "================================================"
echo ""
echo -e "  ${GREEN}PASSED:${NC} $PASS"
echo -e "  ${RED}FAILED:${NC} $FAIL"
echo ""

if [ $FAIL -eq 0 ]; then
    echo -e "${GREEN}✓ All checks passed! System is ready to use.${NC}"
    echo ""
    echo "Quick start:"
    echo "  ./unitree_lidar_L1 status         # Check system"
    echo "  ./unitree_lidar_L1 pointlio       # Run LiDAR + SLAM"
    echo "  ./run_photogrammetry.sh           # Run camera"
    exit 0
else
    echo -e "${YELLOW}! Some checks failed. Review above for details.${NC}"
    echo ""
    echo "Common fixes:"
    echo "  - Run: ./setup_workspace.sh"
    echo "  - Run: ./setup_photogrammetry_node.sh"
    echo "  - Connect hardware (LiDAR, Camera)"
    exit 1
fi
