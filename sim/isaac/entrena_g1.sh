#!/bin/bash
# ENTRENAMIENTO de la politica de locomocion del G1 (peldano 6, fase 2).
#
# Por que entrenar y no descargar: NVIDIA no publica politica para el G1, y los checkpoints de
# terceros que hay en HuggingFace vienen de Isaac Gym (formato de observacion distinto) y de
# origen no verificado. Entrenando con Isaac Lab controlamos la recompensa y sabemos que hace.
#
# OJO A LA MEMORIA: en esta maquina Kimi ocupa ~20.7 GB de cada GPU, asi que quedan ~3.5 GB.
# El numero de entornos paralelos se ajusta a eso (por defecto 4096, aqui muchos menos).
#
#   NUM_ENVS=512 MAX_IT=3000 bash entrena_g1.sh
NUM_ENVS=${NUM_ENVS:-512}
MAX_IT=${MAX_IT:-3000}
TAREA=${TAREA:-Isaac-Velocity-Flat-G1-v0}
cd /ws/IsaacLab
echo "== entrenando $TAREA con $NUM_ENVS entornos, $MAX_IT iteraciones =="
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task "$TAREA" \
  --num_envs "$NUM_ENVS" \
  --max_iterations "$MAX_IT" \
  --headless \
  2>&1 | grep -vE "^\[|Warning|deprecat"
