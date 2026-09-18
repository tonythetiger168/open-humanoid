# OpenHumanoid v1.0 - GitHub Backup Guide

## Quick Start (One-Command Push)

```bash
cd open_robot_project
./scripts/push_to_github.sh your-github-username open-humanoid-v1 your@email.com
```

## Manual Steps

### Step 1: Create GitHub Repository

1. Go to https://github.com/new
2. Repository name: `open-humanoid-v1`
3. Description: `OpenHumanoid v1.0 - Open-source bipedal humanoid robot platform`
4. Visibility: **Public** (recommended for open source)
5. Do NOT initialize with README (we already have one)
6. Click **Create repository**

### Step 2: Set Up Git Authentication

**Option A: SSH Key (Recommended)**
```bash
# Generate SSH key (if not exists)
ssh-keygen -t ed25519 -C "your@email.com"

# Add to SSH agent
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519

# Copy public key to GitHub Settings -> SSH Keys
cat ~/.ssh/id_ed25519.pub
```

**Option B: Personal Access Token (HTTPS)**
1. Go to https://github.com/settings/tokens
2. Generate new token (classic)
3. Scopes: `repo` (full control)
4. Copy token and use as password when pushing

### Step 3: Initialize and Push

```bash
cd open_robot_project

# Initialize git
git init
git branch -m main

# Configure user
git config user.name "Your Name"
git config user.email "your@email.com"

# Add all files
git add -A

# Commit
git commit -m "OpenHumanoid v1.0 - Initial release

Complete open-source humanoid robot platform"

# Add remote (SSH)
git remote add origin git@github.com:YOUR_USERNAME/open-humanoid-v1.git

# Push
git push -u origin main
```

### Step 4: Verify

Visit: `https://github.com/YOUR_USERNAME/open-humanoid-v1`

You should see all 53 files and the README rendered.

## Repository Settings

### Enable GitHub Actions
1. Settings -> Actions -> General
2. Allow all actions and reusable workflows
3. Save

### Branch Protection (Optional)
1. Settings -> Branches -> Add rule
2. Branch name pattern: `main`
3. Enable:
   - Require pull request reviews
   - Require status checks to pass (CI)
   - Include administrators

### Topics and Description
1. Click gear icon next to "About"
2. Add topics: `humanoid-robot`, `bipedal`, `ros2`, `pybullet`, `lerobot`, `open-source`
3. Add website: (your project page or demo video link)

## Release Tags

```bash
# Tag the initial release
git tag -a v1.0.0 -m "OpenHumanoid v1.0.0 - Complete platform release"
git push origin v1.0.0
```

Then on GitHub:
1. Go to Releases -> Draft a new release
2. Choose tag: `v1.0.0`
3. Title: `OpenHumanoid v1.0.0`
4. Attach the zip file: `open_humanoid_project_v1.0.zip`
5. Publish release

## Future Updates

```bash
# After making changes
git add -A
git commit -m "Description of changes"
git push origin main

# For version updates
git tag -a v1.1.0 -m "Version 1.1.0"
git push origin v1.1.0
```

## Large File Handling

If you add large files (models, datasets):
```bash
# Install Git LFS
git lfs install

# Track large files
git lfs track "*.pt"
git lfs track "*.pth"
git lfs track "*.hdf5"
git lfs track "datasets/*.npz"
```

## Backup Checklist

- [ ] Repository created on GitHub
- [ ] All 53 files pushed successfully
- [ ] README.md renders correctly
- [ ] GitHub Actions CI workflow enabled
- [ ] v1.0.0 tag created and released
- [ ] Zip file attached to release
- [ ] Topics and description added
- [ ] Branch protection configured (optional)
