#!/bin/bash
# Verificacion completa tras el cambio de driver (22-ago). Lanzar DESPUES del reinicio.
echo "=== 1. DRIVER ==="
nvidia-smi --query-gpu=driver_version,name --format=csv,noheader
echo
echo "=== 2. CUDA / TORCH (de esto depende el servidor de percepcion) ==="
PYTHONPATH=/home/ros/g1env/lib/python3.10/site-packages /usr/bin/python3.10 -c "
import torch
print('torch', torch.__version__, '| cuda:', torch.cuda.is_available(), '| GPUs:', torch.cuda.device_count())
a = torch.randn(1000,1000, device='cuda:0') @ torch.randn(1000,1000, device='cuda:0')
torch.cuda.synchronize(); print('matmul en GPU: OK')
" 2>&1 | tail -3
echo
echo "=== 3. SESION GRAFICA (autologin) ==="
who | grep -q ":1" && echo ":1 presente (autologin OK)" || echo ":1 ausente -> usar tools/arranca_percepcion.sh (Xvfb)"
echo
echo "=== 4. ISAAC SIM: la prueba de fuego ==="
timeout 420 docker run --rm --gpus all --network host --user root --entrypoint ./python.sh \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e OMNI_KIT_ALLOW_ROOT=1 -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /home/ros/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v /home/ros/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v /home/ros/isaac_ws:/ws:rw \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  -c "from isaacsim import SimulationApp; app=SimulationApp({'headless':True}); print('=== ISAAC ARRANCA OK ==='); app.close()" 2>&1 \
  | grep -E "ISAAC ARRANCA OK|Crash detected|error running" | head -3
