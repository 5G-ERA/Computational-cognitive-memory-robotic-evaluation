"""Color REAL del entorno, muestreado de los fotogramas del G1.

Suelo: tercio inferior central de los fotogramas de travesia (el robot mira al frente, y esa
zona es moqueta casi siempre). Pared/ambiente: banda superior. Se usa la MEDIANA por canal
sobre cientos de fotogramas, que es robusta a los objetos que crucen el encuadre.
"""
import glob, json
from PIL import Image
import numpy as np

fs = sorted(glob.glob("dataset/20260820_1[56]*_ours_[AB]_t*.jpg"))
fs += sorted(glob.glob("dataset/20260821_*_ours_[AB]_t*.jpg"))
print("fotogramas:", len(fs))

suelo, alto = [], []
for p in fs:
    try:
        a = np.asarray(Image.open(p).convert("RGB"), dtype=np.float32)
    except Exception:
        continue
    h, w = a.shape[:2]
    suelo.append(a[int(h*0.72):, int(w*0.3):int(w*0.7)].reshape(-1, 3).mean(axis=0))
    alto.append(a[:int(h*0.18), :].reshape(-1, 3).mean(axis=0))

s = np.median(np.array(suelo), axis=0)
t = np.median(np.array(alto), axis=0)
print("SUELO  RGB mediano: %.0f %.0f %.0f" % tuple(s))
print("BANDA ALTA (pared/techo) RGB mediano: %.0f %.0f %.0f" % tuple(t))
json.dump({"suelo": [round(float(v)/255, 3) for v in s],
           "pared": [round(float(v)/255, 3) for v in t],
           "n_frames": len(suelo)},
          open("dataset/colores_reales.json", "w"), indent=1)
print("escrito dataset/colores_reales.json")
