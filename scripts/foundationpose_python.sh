#!/usr/bin/env bash
# 使用项目专用 FoundationPose 环境和 CUDA 工具链，不激活或改写 Isaac 环境。
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
installation="$project_root/outputs/foundationpose_install"
pose_python="$installation/venv/bin/python"
toolchain="$installation/toolchain"
if [[ ! -x "$pose_python" || ! -x "$toolchain/bin/nvcc" ]]; then
    echo 'FoundationPose 环境或 CUDA 工具链尚未安装，见 docs/foundationpose.md。' >&2
    exit 1
fi
export CUDA_HOME="$toolchain/targets/x86_64-linux"
export PATH="$toolchain/bin:$toolchain/nvvm/bin:$PATH"
export CC="$toolchain/bin/x86_64-conda-linux-gnu-gcc"
export CXX="$toolchain/bin/x86_64-conda-linux-gnu-g++"
export CUDAHOSTCXX="$CXX"
export MAX_JOBS="${MAX_JOBS:-2}"
export TORCH_EXTENSIONS_DIR="$installation/torch_extensions"
cuda_headers="$("$pose_python" -c 'import sys,pathlib; p=pathlib.Path(sys.base_prefix)/"lib/python3.11/site-packages/nvidia"; print(":".join(str(x) for x in p.glob("*/include")))')"
export CPATH="$cuda_headers${CPATH:+:$CPATH}"
exec "$pose_python" "$@"
