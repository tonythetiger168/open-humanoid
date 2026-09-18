#!/bin/bash
# OpenHumanoid v1.0 - GitHub Push Script
# Usage: ./scripts/push_to_github.sh [your-github-username] [repo-name]

set -e

GITHUB_USER=${1:-"your-username"}
REPO_NAME=${2:-"open-humanoid-v1"}
EMAIL=${3:-"your.email@example.com"}

echo "========================================"
echo "OpenHumanoid v1.0 - GitHub Push"
echo "========================================"
echo "User: $GITHUB_USER"
echo "Repo: $REPO_NAME"
echo "Email: $EMAIL"
echo ""

# Check git
if ! command -v git &> /dev/null; then
    echo "ERROR: git not installed"
    echo "Install: sudo apt install git"
    exit 1
fi

# Check GitHub CLI (optional but recommended)
if command -v gh &> /dev/null; then
    echo "[OK] GitHub CLI (gh) detected"
    USE_GH=true
else
    echo "[INFO] GitHub CLI not found, using git + HTTPS/SSH"
    USE_GH=false
fi

# Configure git (if not already set)
if [ -z "$(git config --global user.name)" ]; then
    git config --global user.name "$GITHUB_USER"
fi
if [ -z "$(git config --global user.email)" ]; then
    git config --global user.email "$EMAIL"
fi

cd "$(dirname "$0")/.."
PROJECT_DIR=$(pwd)

echo "[1/7] Project directory: $PROJECT_DIR"

# Initialize git if not already
if [ ! -d ".git" ]; then
    echo "[2/7] Initializing git repository..."
    git init
    git branch -m main
else
    echo "[2/7] Git repository already initialized"
fi

# Add all files
echo "[3/7] Adding files to staging..."
git add -A

# Check status
echo "[4/7] Git status:"
git status --short | head -20
if [ $(git status --short | wc -l) -gt 20 ]; then
    echo "... and $(git status --short | wc -l) more files"
fi

# Commit
echo "[5/7] Creating commit..."
git commit -m "OpenHumanoid v1.0 - Initial release

Complete open-source humanoid robot platform including:
- 28-DoF URDF model and joint definitions
- PyBullet digital twin simulation environment
- ZMP Preview balance controller + 3D gait generator
- EKF state estimator with leg odometry
- LeRobot AI interface (ACT/Diffusion Policy)
- ODrive Pro CAN-FD motor driver interface
- Single joint validation suite (SJV-1~6)
- ROS2 Humble integration node
- Docker containerization
- CI/CD pipeline (GitHub Actions)
- Hardware BOM (~\$11,700) and assembly guide
- Sim-to-Real migration documentation

Status: Phase 0-8 + Follow-up + SJV Complete"

# Setup remote
REMOTE_URL="git@github.com:$GITHUB_USER/$REPO_NAME.git"
if git remote | grep -q "origin"; then
    echo "[6/7] Updating remote origin..."
    git remote set-url origin "$REMOTE_URL"
else
    echo "[6/7] Adding remote origin..."
    git remote add origin "$REMOTE_URL"
fi

# Push
echo "[7/7] Pushing to GitHub..."
echo "Remote: $REMOTE_URL"

if [ "$USE_GH" = true ]; then
    # Use GitHub CLI for authentication
    if ! gh auth status &> /dev/null; then
        echo "GitHub CLI not authenticated. Running gh auth login..."
        gh auth login
    fi

    # Create repo if not exists
    if ! gh repo view "$GITHUB_USER/$REPO_NAME" &> /dev/null; then
        echo "Creating GitHub repository..."
        gh repo create "$REPO_NAME" --public --description "OpenHumanoid v1.0 - Open-source bipedal humanoid robot platform" --source=. --push
    else
        git push -u origin main
    fi
else
    echo ""
    echo "Please ensure you have:"
    echo "  1. Created repository: https://github.com/new"
    echo "  2. Set up SSH key: https://docs.github.com/en/authentication/connecting-to-github-with-ssh"
    echo "  3. Or use HTTPS with Personal Access Token"
    echo ""
    read -p "Press ENTER to push, or Ctrl+C to cancel..."
    git push -u origin main
fi

echo ""
echo "========================================"
echo "Push complete!"
echo "Repository: https://github.com/$GITHUB_USER/$REPO_NAME"
echo "========================================"
