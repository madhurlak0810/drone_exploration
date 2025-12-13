#!/bin/bash
# World Selector Script for Drone Exploration
# Usage: ./select_world.sh [world_name]
# Available worlds: exploration, maze, rooms, scattered

set -e

# Configuration
WORLD_DIR="/home/maddy/drone_ws/src/drone_rl/worlds"
LAUNCH_FILE="/home/maddy/drone_ws/launch_final.sh"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_info() {
    echo -e "${BLUE}[WORLD]${NC} $1"
}

# Available worlds
WORLDS=(
    "exploration:Original exploration environment with mixed obstacles"
    "maze:Maze-like layout with narrow corridors and dead ends"
    "rooms:Multi-room structure with distinct chambers"
    "scattered:Random obstacle distribution for varied exploration"
)

# Function to show available worlds
show_worlds() {
    print_info "Available Exploration Worlds:"
    echo ""
    for world in "${WORLDS[@]}"; do
        IFS=':' read -r name desc <<< "$world"
        echo -e "  ${BLUE}$name${NC}: $desc"
    done
    echo ""
}

# Function to validate world name
validate_world() {
    local world_name=$1
    local world_file="${WORLD_DIR}/${world_name}_world.world"
    
    if [[ ! -f "$world_file" ]]; then
        print_error "World file not found: $world_file"
        return 1
    fi
    
    return 0
}

# Function to update launch script with selected world
update_launch_script() {
    local world_name=$1
    local world_file="${world_name}_world.world"
    
    print_status "Updating launch script to use $world_name world..."
    
    # Create backup
    cp "$LAUNCH_FILE" "${LAUNCH_FILE}.backup"
    
    # Update the world file reference in launch script
    sed -i "s/[a-z_]*_world\.world/${world_file}/g" "$LAUNCH_FILE"
    
    print_status "Launch script updated successfully!"
    print_info "World: $world_name"
    print_info "File: $world_file"
}

# Function to show usage
show_usage() {
    echo "World Selector for Autonomous Drone Exploration"
    echo ""
    echo "Usage: $0 [WORLD_NAME]"
    echo ""
    show_worlds
    echo "Examples:"
    echo "  $0 maze          # Switch to maze world"
    echo "  $0 rooms         # Switch to rooms world"
    echo "  $0 scattered     # Switch to scattered world"
    echo "  $0 exploration   # Switch back to original world"
    echo ""
    echo "After selecting a world, run:"
    echo "  ./launch_final.sh              # Native launch"
    echo "  ./docker_run.sh build         # Rebuild Docker with new world"
    echo "  ./docker_run.sh gui            # Run Docker with new world"
    echo ""
}

# Main script logic
if [[ $# -eq 0 ]]; then
    show_usage
    exit 0
fi

WORLD_NAME=$1

# Validate world name
if ! validate_world "$WORLD_NAME"; then
    echo ""
    show_usage
    exit 1
fi

# Update launch script
update_launch_script "$WORLD_NAME"

echo ""
print_status "World selection complete!"
print_warning "Remember to rebuild Docker image if using containers:"
print_info "  ./docker_run.sh build"
print_info "  ./docker_run.sh gui"
echo ""
print_status "For native execution, simply run:"
print_info "  ./launch_final.sh"