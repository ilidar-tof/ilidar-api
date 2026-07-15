#!/bin/sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
BUILD_TYPE=${BUILD_TYPE:-Release}
PYTHON=${PYTHON:-python3}

BUILD_ITFS=0
BUILD_LITE=0
BUILD_ITFS_PYWRAPPER=0
BUILD_LITE_PYWRAPPER=0

usage() {
    echo "Usage: ./build.sh [--itfs] [--lite] [--itfs_pywrapper] [--lite_pywrapper]"
    echo
    echo "If no target is given, all targets are built."
    echo "Environment:"
    echo "  BUILD_TYPE=Release|Debug   CMake build type, default: Release"
    echo "  PYTHON=python3             Python command, default: python3"
}

if [ "$#" -eq 0 ]; then
    BUILD_ITFS=1
    BUILD_LITE=1
    BUILD_ITFS_PYWRAPPER=1
    BUILD_LITE_PYWRAPPER=1
fi

while [ "$#" -gt 0 ]; do
    case "$1" in
        --itfs)
            BUILD_ITFS=1
            ;;
        --lite)
            BUILD_LITE=1
            ;;
        --itfs_pywrapper|--itfs-pywrapper|--itfs_pyswrapper)
            BUILD_ITFS_PYWRAPPER=1
            ;;
        --lite_pywrapper|--lite-pywrapper|--lite_pyswrapper)
            BUILD_LITE_PYWRAPPER=1
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
    shift
done

if [ "$BUILD_ITFS" -eq 1 ]; then
    echo "[build] C++ iTFS examples"
    cd "$ROOT_DIR/itfs"
    cmake -S . -B build -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
    cmake --build build
fi

if [ "$BUILD_LITE" -eq 1 ]; then
    echo "[build] C++ iTFS-LITE examples"
    cd "$ROOT_DIR/lite"
    cmake -S . -B build -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
    cmake --build build
fi

if [ "$BUILD_ITFS_PYWRAPPER" -eq 1 ]; then
    echo "[build] Python iTFS wrapper"
    cd "$ROOT_DIR/itfs_pywrapper"
    "$PYTHON" setup.py build_ext --inplace
fi

if [ "$BUILD_LITE_PYWRAPPER" -eq 1 ]; then
    echo "[build] Python iTFS-LITE wrapper"
    cd "$ROOT_DIR/lite_pywrapper"
    "$PYTHON" setup.py build_ext --inplace
fi

echo "[build] done"
