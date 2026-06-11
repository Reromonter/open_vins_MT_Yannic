#!/usr/bin/env bash
# Cross-build all OAK-D + OpenVINS containers for Raspberry Pi 4 (linux/arm64)
# and optionally deploy them directly onto the Pi.
#
# Usage:
#   ./crossbuild_pi.sh
#
# Requirements (host):
#   docker buildx, QEMU binfmt support (installed below if missing)
#
# Requirements (Pi):
#   Docker installed, SSH access

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/dist/arm64"
PLATFORM="linux/arm64"
BUILDER_NAME="ov-arm64-builder"

# Docker image names — must match what `docker compose` expects
# (project name = directory name, lowercased)
PROJECT="open_vins_mt_yannic"
IMG_OAK_DRIVER="${PROJECT}-oak_driver"
IMG_OPENVINS="${PROJECT}-openvins"
IMG_LOGGER="ros-humble-ros-base-arm64"   # pulled, not built
IMG_BASALT="${PROJECT}-basalt"

TAR_OAK_DRIVER="$BUILD_DIR/oak_driver.tar"
TAR_OPENVINS="$BUILD_DIR/openvins.tar"
TAR_LOGGER="$BUILD_DIR/logger.tar"
TAR_BASALT="$BUILD_DIR/basalt.tar"

# Note: build/ is owned by root (created by colcon inside Docker)
# dist/ is used instead to avoid permission errors

# ─────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────
log()  { echo ""; echo "▶  $*"; }
ok()   { echo "   ✓  $*"; }
warn() { echo "   ⚠  $*"; }
ask()  { read -rp "   $* " REPLY; echo "$REPLY"; }

confirm() {
    local prompt="$1"
    local answer
    answer=$(ask "$prompt [Y/n]:")
    [[ "${answer:-Y}" =~ ^[Yy]$ ]]
}

# ─────────────────────────────────────────────────────────────
# 1. Preflight checks
# ─────────────────────────────────────────────────────────────
log "Checking prerequisites..."

if ! command -v docker &>/dev/null; then
    echo "ERROR: docker not found." >&2; exit 1
fi
if ! docker buildx version &>/dev/null; then
    echo "ERROR: docker buildx not available (Docker >= 19.03 required)." >&2; exit 1
fi
ok "Docker $(docker --version | awk '{print $3}' | tr -d ',')"

# ─────────────────────────────────────────────────────────────
# 2. QEMU binfmt support
# ─────────────────────────────────────────────────────────────
log "Checking QEMU binfmt support for arm64..."
if ! ls /proc/sys/fs/binfmt_misc/qemu-aarch64 &>/dev/null; then
    warn "QEMU arm64 binfmt not registered — installing now (requires sudo)."
    docker run --privileged --rm tonistiigi/binfmt --install arm64
    ok "QEMU arm64 registered."
else
    ok "QEMU arm64 already registered."
fi

# ─────────────────────────────────────────────────────────────
# 3. Create / reuse buildx builder
# ─────────────────────────────────────────────────────────────
log "Setting up buildx builder ($BUILDER_NAME)..."
if ! docker buildx inspect "$BUILDER_NAME" &>/dev/null; then
    docker buildx create \
        --name "$BUILDER_NAME" \
        --driver docker-container \
        --platform linux/amd64,linux/arm64 \
        --use
    docker buildx inspect --bootstrap "$BUILDER_NAME"
    ok "Builder created."
else
    docker buildx use "$BUILDER_NAME"
    ok "Reusing existing builder."
fi

mkdir -p "$BUILD_DIR"

# ─────────────────────────────────────────────────────────────
# 4. Select which images to build
# ─────────────────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════"
echo "  Cross-build targets (linux/arm64)"
echo "════════════════════════════════════════"
echo "  [1] oak_driver     (Dockerfile_oak_driver)"
echo "  [2] openvins       (Dockerfile_ros2_22_04)    — slow: ~40 min"
echo "  [3] logger         (ros:humble-ros-base pull)"
echo "  [4] basalt         (Dockerfile_basalt)"
echo "  [5] All of the above"
echo ""
TARGET=$(ask "Which targets to build? [1/2/3/4/5]:")

BUILD_OAK=false; BUILD_OV=false; BUILD_LOG=false; BUILD_BASALT=false
case "${TARGET:-5}" in
    1) BUILD_OAK=true ;;
    2) BUILD_OV=true ;;
    3) BUILD_LOG=true ;;
    4) BUILD_BASALT=true ;;
    5|*) BUILD_OAK=true; BUILD_OV=true; BUILD_LOG=true; BUILD_BASALT=true ;;
esac

# ─────────────────────────────────────────────────────────────
# 5. Build images → tar files
# ─────────────────────────────────────────────────────────────
build_image() {
    local name="$1" dockerfile="$2" tar="$3"
    log "Building $name for $PLATFORM..."
    warn "Output: $tar"
    docker buildx build \
        --platform "$PLATFORM" \
        --file "$SCRIPT_DIR/$dockerfile" \
        --tag "$name:latest" \
        --output "type=docker,dest=$tar" \
        "$SCRIPT_DIR"
    ok "$name built → $(du -sh "$tar" | cut -f1) saved to $tar"
}

if $BUILD_OAK; then
    build_image "$IMG_OAK_DRIVER" "Dockerfile_oak_driver" "$TAR_OAK_DRIVER"
fi

if $BUILD_OV; then
    warn "openvins build runs colcon under QEMU — expect ~40 minutes."
    build_image "$IMG_OPENVINS" "Dockerfile_ros2_22_04" "$TAR_OPENVINS"
fi

if $BUILD_LOG; then
    log "Pulling ros:humble-ros-base for $PLATFORM..."
    # Pull arm64 variant and re-tag, then export
    docker buildx build \
        --platform "$PLATFORM" \
        --tag "$IMG_LOGGER:latest" \
        --output "type=docker,dest=$TAR_LOGGER" \
        - <<'EOF'
FROM ros:humble-ros-base
EOF
    ok "logger image saved → $(du -sh "$TAR_LOGGER" | cut -f1)"
fi

if $BUILD_BASALT; then
    build_image "$IMG_BASALT" "Dockerfile_basalt" "$TAR_BASALT"
fi

echo ""
log "Build complete. Tarballs in $BUILD_DIR:"
ls -lh "$BUILD_DIR"/*.tar 2>/dev/null || true

# ─────────────────────────────────────────────────────────────
# 6. Optional: deploy to Raspberry Pi
# ─────────────────────────────────────────────────────────────
echo ""
if confirm "Deploy images to a Raspberry Pi now?"; then

    PI_HOST=$(ask "Pi hostname or IP (e.g. raspberrypi.local or 192.168.1.42):")
    PI_USER=$(ask "SSH user on the Pi [pi]:")
    PI_USER="${PI_USER:-pi}"
    PI_DIR=$(ask "Destination directory on the Pi [~/openvins]:")
    PI_DIR="${PI_DIR:-~/openvins}"

    SSH_TARGET="${PI_USER}@${PI_HOST}"

    log "Creating remote directory $PI_DIR on $SSH_TARGET..."
    ssh "$SSH_TARGET" "mkdir -p $PI_DIR"

    # Copy tarballs
    TARS_TO_COPY=()
    $BUILD_OAK    && [[ -f "$TAR_OAK_DRIVER"    ]] && TARS_TO_COPY+=("$TAR_OAK_DRIVER")
    $BUILD_OV     && [[ -f "$TAR_OPENVINS"      ]] && TARS_TO_COPY+=("$TAR_OPENVINS")
    $BUILD_LOG    && [[ -f "$TAR_LOGGER"         ]] && TARS_TO_COPY+=("$TAR_LOGGER")
    $BUILD_BASALT && [[ -f "$TAR_BASALT"  ]] && TARS_TO_COPY+=("$TAR_BASALT")

    if [[ ${#TARS_TO_COPY[@]} -gt 0 ]]; then
        log "Copying tarballs to $SSH_TARGET:$PI_DIR  (may take a while)..."
        scp "${TARS_TO_COPY[@]}" "${SSH_TARGET}:${PI_DIR}/"
        ok "Tarballs copied."
    fi

    # Copy project files needed on the Pi
    log "Copying docker-compose.yml, config, and start_session.sh..."
    scp "$SCRIPT_DIR/docker-compose.yml" \
        "$SCRIPT_DIR/start_session.sh" \
        "${SSH_TARGET}:${PI_DIR}/"
    ssh "$SSH_TARGET" "chmod +x $PI_DIR/start_session.sh"
    rsync -az --info=progress2 \
        "$SCRIPT_DIR/config/" \
        "${SSH_TARGET}:${PI_DIR}/config/"
    ok "Project files copied."

    # Load images on the Pi
    log "Loading Docker images on the Pi..."
    ssh "$SSH_TARGET" bash <<REMOTE
set -e
cd "$PI_DIR"
$(if $BUILD_OAK && [[ -f "$TAR_OAK_DRIVER" ]]; then
    echo "echo '  Loading oak_driver...' && docker load -i dist/arm64/oak_driver.tar 2>/dev/null || docker load -i oak_driver.tar"
fi)
$(if $BUILD_OV && [[ -f "$TAR_OPENVINS" ]]; then
    echo "echo '  Loading openvins...'   && docker load -i dist/arm64/openvins.tar 2>/dev/null || docker load -i openvins.tar"
fi)
$(if $BUILD_LOG && [[ -f "$TAR_LOGGER" ]]; then
    echo "echo '  Loading logger...'         && docker load -i dist/arm64/logger.tar 2>/dev/null || docker load -i logger.tar"
fi)
$(if $BUILD_BASALT && [[ -f "$TAR_BASALT" ]]; then
    echo "echo '  Loading basalt...'  && docker load -i dist/arm64/basalt.tar 2>/dev/null || docker load -i basalt.tar"
fi)
echo "  Done."
REMOTE
    ok "All images loaded on the Pi."

    # Tag images so docker compose finds them without rebuilding
    log "Tagging images to match docker compose service names..."
    ssh "$SSH_TARGET" bash <<REMOTE
set -e
$(if $BUILD_OAK; then
    echo "docker tag ${IMG_OAK_DRIVER}:latest ${PROJECT}-oak_driver:latest 2>/dev/null || true"
fi)
$(if $BUILD_OV; then
    echo "docker tag ${IMG_OPENVINS}:latest ${PROJECT}-openvins:latest 2>/dev/null || true"
fi)
$(if $BUILD_LOG; then
    echo "docker tag ${IMG_LOGGER}:latest ros:humble-ros-base 2>/dev/null || true"
fi)
$(if $BUILD_BASALT; then
    echo "docker tag ${IMG_BASALT}:latest ${PROJECT}-basalt:latest 2>/dev/null || true"
fi)
REMOTE
    ok "Images tagged."

    echo ""
    echo "════════════════════════════════════════"
    echo "  Deployment complete!"
    echo "════════════════════════════════════════"
    echo ""
    echo "  On the Pi, run:"
    echo "    cd $PI_DIR"
    echo "    ./start_session.sh"
    echo ""
    echo "  Or manually:"
    echo "    export RUN_TIMESTAMP=\$(date +%Y%m%d_%H%M%S)"
    echo "    docker compose up"
    echo ""
    echo "  Note: docker compose will use the pre-loaded images."
    echo "  Add 'image: ${PROJECT}-openvins' / 'image: ${PROJECT}-oak_driver'"
    echo "  to docker-compose.yml to skip the build step entirely."
fi

echo ""
ok "All done."
