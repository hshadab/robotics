#!/usr/bin/env bash
set -euo pipefail

# Build atlas_argmax_prover in a Docker container and extract the binary.
# Usage: ./build_helper.sh [image_tag] [jolt_ref]
#   image_tag: optional docker tag (default: atlas-helper:latest)
#   jolt_ref:  optional git ref for ICME-Lab/jolt-atlas (default: main)

TAG="${1:-atlas-helper:latest}"
JOLT_REF="${2:-main}"

HERE="$(cd "$(dirname "$0")" && pwd)"

ENGINE=""
if command -v docker >/dev/null 2>&1; then
  if docker info >/dev/null 2>&1; then
    ENGINE="docker"
  fi
fi
if [ -z "$ENGINE" ] && command -v podman >/dev/null 2>&1; then
  ENGINE="podman"
fi
if [ -z "$ENGINE" ]; then
  echo "Neither a running docker engine nor podman found. Please install or start one and rerun." >&2
  exit 2
fi

echo "[+] Building atlas_argmax_prover container with ${ENGINE} (ref=${JOLT_REF})..."
${ENGINE} build \
  --build-arg JOLT_ATLAS_REF="${JOLT_REF}" \
  -t "${TAG}" \
  -f "${HERE}/Dockerfile" "${HERE}"

echo "[+] Extracting atlas_argmax_prover binary from image..."
cid=$(${ENGINE} create "${TAG}")
mkdir -p "${HERE}/bin"
${ENGINE} cp "$cid":/usr/local/bin/atlas_argmax_prover "${HERE}/bin/atlas_argmax_prover"
${ENGINE} rm "$cid" >/dev/null
chmod +x "${HERE}/bin/atlas_argmax_prover"

echo "[+] Done. Binary at: ${HERE}/bin/atlas_argmax_prover"
echo "    Add to PATH: export PATH=\"${HERE}/bin:$PATH\""
