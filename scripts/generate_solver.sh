#!/bin/bash
set -e

CODEGEN_PYTHON=$1
PYTHON_SCRIPT=$2
GEN_CODE_PATH=$3
NUM_OBS=$4
HORIZON=$5

echo "--- ACADOS codegen"
echo "Python: ${CODEGEN_PYTHON}"
echo "Script: ${PYTHON_SCRIPT}"
echo "Out   : ${GEN_CODE_PATH}"
echo "NumObs: ${NUM_OBS}"
echo "Horizon: ${HORIZON}"

mkdir -p "${GEN_CODE_PATH}"

"${CODEGEN_PYTHON}" "${PYTHON_SCRIPT}" \
  --gen_code_path "${GEN_CODE_PATH}" \
  --num_obs "${NUM_OBS}" \
  --horizon "${HORIZON}"

echo "--- Codegen done."