#!/bin/bash
# Docker Build and Run Script for Drone Exploration System
# Usage: ./docker_run.sh [build|run|gui|dev|stop|clean|logs]

set -e

# Configuration
IMAGE_NAME="drone-exploration"
IMAGE_TAG="latest"
CONTAINER_NAME="drone-exploration-container"

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
    echo -e "${BLUE}[DOCKER]${NC} $1"
}

# Function to check if Docker is running
check_docker() {
    if ! docker info >/dev/null 2>&1; then
        print_error "Docker is not running. Please start Docker and try again."
        exit 1
    fi
}

# Function to pull the Docker image from Docker Hub
pull_image() {
    print_status "Pulling drone exploration Docker image from Docker Hub..."
    docker pull madhurlak/drone-exploration:latest
    docker tag madhurlak/drone-exploration:latest ${IMAGE_NAME}:${IMAGE_TAG}
    print_status "Docker image pulled and tagged successfully!"
}

# Function to build the Docker image
build_image() {
    print_status "Building drone exploration Docker image..."
    docker build -t ${IMAGE_NAME}:${IMAGE_TAG} .
    print_status "Docker image built successfully!"
}

# Function to run headless exploration
run_headless() {
    print_status "Starting headless drone exploration..."
    
    # Stop any existing container
    docker stop ${CONTAINER_NAME} 2>/dev/null || true
    docker rm ${CONTAINER_NAME} 2>/dev/null || true
    
    # Run new container
    docker run -d \
        --name ${CONTAINER_NAME} \
        --restart unless-stopped \
        -v $(pwd)/q_table.npy:/drone_ws/q_table.npy \
        -v $(pwd)/metrics:/drone_ws/metrics \
        -v $(pwd)/log:/drone_ws/log \
        ${IMAGE_NAME}:${IMAGE_TAG} headless
    
    print_status "Container started. Use 'docker logs -f ${CONTAINER_NAME}' to view output"
    print_info "To stop: docker stop ${CONTAINER_NAME}"
}

# Function to run with GUI access
run_gui() {
    print_status "Starting GUI-enabled drone exploration..."
    print_warning "Make sure to access VNC on port 5900 or web interface on port 6080"
    
    # Stop any existing container
    docker stop ${CONTAINER_NAME}-gui 2>/dev/null || true
    docker rm ${CONTAINER_NAME}-gui 2>/dev/null || true
    
    # Run new container with GUI support
    docker run -d \
        --name ${CONTAINER_NAME}-gui \
        --restart unless-stopped \
        -p 5900:5900 \
        -p 6080:6080 \
        -v $(pwd)/q_table.npy:/drone_ws/q_table.npy \
        -v $(pwd)/metrics:/drone_ws/metrics \
        -v $(pwd)/log:/drone_ws/log \
        ${IMAGE_NAME}:${IMAGE_TAG} gui
    
    print_status "GUI container started!"
    print_info "VNC Access: localhost:5900 (password not required)"
    print_info "Web VNC: http://localhost:6080"
    print_info "To stop: docker stop ${CONTAINER_NAME}-gui"
}

# Function to start development environment
run_dev() {
    print_status "Starting development environment..."
    
    docker run -it --rm \
        --name ${CONTAINER_NAME}-dev \
        -v $(pwd):/drone_ws \
        -e DISPLAY=${DISPLAY} \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        ${IMAGE_NAME}:${IMAGE_TAG} bash
}

# Function to view logs
view_logs() {
    if [ $# -eq 2 ]; then
        container_suffix=$2
    else
        container_suffix=""
    fi
    
    container_name="${CONTAINER_NAME}${container_suffix}"
    
    if docker ps -a | grep -q ${container_name}; then
        print_status "Showing logs for ${container_name}..."
        docker logs -f ${container_name}
    else
        print_error "Container ${container_name} not found"
    fi
}

# Function to stop containers
stop_containers() {
    print_status "Stopping all drone exploration containers..."
    
    docker stop ${CONTAINER_NAME} 2>/dev/null || true
    docker stop ${CONTAINER_NAME}-gui 2>/dev/null || true
    docker stop ${CONTAINER_NAME}-dev 2>/dev/null || true
    
    print_status "All containers stopped"
}

# Function to clean up
cleanup() {
    print_status "Cleaning up containers and unused images..."
    
    # Stop containers
    stop_containers
    
    # Remove containers
    docker rm ${CONTAINER_NAME} 2>/dev/null || true
    docker rm ${CONTAINER_NAME}-gui 2>/dev/null || true
    docker rm ${CONTAINER_NAME}-dev 2>/dev/null || true
    
    # Remove dangling images
    docker image prune -f
    
    print_status "Cleanup complete"
}

# Function to show usage
show_usage() {
    echo "Docker Management Script for Drone Exploration System"
    echo ""
    echo "Usage: $0 [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  pull      - Pull pre-built image from Docker Hub"
    echo "  build     - Build the Docker image locally"
    echo "  run       - Run headless exploration (default)"
    echo "  gui       - Run with GUI access (VNC)"
    echo "  dev       - Start development environment"
    echo "  logs      - View logs from running container"
    echo "  logs-gui  - View logs from GUI container"
    echo "  stop      - Stop all running containers"
    echo "  clean     - Stop containers and clean up images"
    echo "  status    - Show container status"
    echo ""
    echo "Examples:"
    echo "  $0 pull && $0 gui     - Pull and run with GUI"
    echo "  $0 build && $0 run    - Build and run headless"
    echo "  $0 logs              - View current logs"
    echo "  $0 clean             - Full cleanup"
    echo ""
    echo "System Status:"
    if docker ps | grep -q drone-exploration; then
        echo -e "  Status: ${GREEN}Running${NC}"
    else
        echo -e "  Status: ${RED}Stopped${NC}"
    fi
}

# Function to show container status
show_status() {
    print_status "Container Status:"
    echo ""
    
    # Check headless container
    if docker ps | grep -q ${CONTAINER_NAME}; then
        echo -e "  Headless: ${GREEN}Running${NC}"
    else
        echo -e "  Headless: ${RED}Stopped${NC}"
    fi
    
    # Check GUI container
    if docker ps | grep -q ${CONTAINER_NAME}-gui; then
        echo -e "  GUI: ${GREEN}Running${NC} (VNC: localhost:5900, Web: localhost:6080)"
    else
        echo -e "  GUI: ${RED}Stopped${NC}"
    fi
    
    echo ""
    print_info "Use '$0 logs' to view running container logs"
}

# Main script logic
check_docker

case "${1:-run}" in
    "pull")
        pull_image
        ;;
    "build")
        build_image
        ;;
    "run"|"headless")
        run_headless
        ;;
    "gui")
        run_gui
        ;;
    "dev"|"development")
        run_dev
        ;;
    "logs")
        view_logs $@
        ;;
    "logs-gui")
        view_logs $@ "-gui"
        ;;
    "stop")
        stop_containers
        ;;
    "clean"|"cleanup")
        cleanup
        ;;
    "status")
        show_status
        ;;
    "help"|"--help"|"-h")
        show_usage
        ;;
    *)
        print_error "Unknown command: $1"
        echo ""
        show_usage
        exit 1
        ;;
esac