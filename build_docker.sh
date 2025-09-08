#!/usr/bin/env bash

###################### COLOR

RED='\033[1;31m'
BLUE='\033[1;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

###################### CONFIG

FOLDER_PATH="$(pwd)"
BRANCH="$(git rev-parse --abbrev-ref HEAD)"
DOCKERFILE='dockerfile'
DOCKER_IMAGE="sipoc:${BRANCH}"
DOCKER_BUILDKIT=1
CUSTOM_BUILD_ARGS=''

function info() { echo -e "${BLUE}$1${NC}"; }
function error() { echo -e "${RED}$1${NC}"; }

###################### VALIDATE

info ">> Check prerequisites"

if [ -d "$FOLDER_PATH" ]; then
  info ">> Directory '$FOLDER_PATH' exists"
else
  error ">> Directory '$FOLDER_PATH' does not exist"
  exit 1
fi

if [ -f "$FOLDER_PATH/$DOCKERFILE" ]; then
  info ">> Dockerfile '$DOCKERFILE' exists"
else
  error ">> Dockerfile '$FOLDER_PATH/$DOCKERFILE' does not exist"
  exit 1
fi

if [[ "${2}" = --build-arg* ]]
then
	info ">> Found additional build args: ${2}"
  CUSTOM_BUILD_ARGS="${2}"
fi

###################### RUN

info ">> Start building ${DOCKER_IMAGE}"

docker build \
    --pull \
    --force-rm=true \
    --network=host \
    --progress=plain \
    --no-cache \
    --platform linux/amd64 \
    $CUSTOM_BUILD_ARGS \
    -t "${DOCKER_IMAGE}" .

info ">> Done"


