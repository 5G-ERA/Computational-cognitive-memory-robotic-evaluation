#!/usr/bin/env python3
"""Manifiesto de corte válido (Renxi, 10-nov-2026, I108).

«Define a valid-cut manifest containing the traversal ID, alignment and freeze
timestamps, pose freshness, component timestamps, file hashes and software versions.»
«Report attempted, aligned, frozen, valid and classified records separately.»

Uso:
  manifiesto_corte.py CARPETA_TRAVESIA [--marca RUTA]     -> escribe MANIFIESTO_CORTE.json
  manifiesto_corte.py --lote CARPETA_LOTE [--intentadas N] -> recuento del lote

Sólo lee. No abre etiquetas ni referencia. No decide nada del experimento:
las comprobaciones son de adquisición (tiempos, frescura, sincronía, integridad).
El umbral de frescura es el que la propia sombra declaró al armarse
(SOMBRA_ARMADA.json -> umbral.frescura_s); aquí no se fija ningún valor nuevo.
"""
import argparse, hashlib, json, sys
from pathlib import Path

VERSION = 'manifiesto_corte 0.3 (10-nov-2026; marca del robot real e indicadores de validez; ContextID, PairID, CutID del documento de contexto de Renxi)'


def sha(p):
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''):
            h.update(b)
    return h.hexdigest()


def carga(p):
    try:
        return json.loads(Path(p).read_text())
    except Exception:
        return None


def lineas(p):
    try:
        return [json.loads(l) for l in Path(p).read_text().splitlines() if l.strip()]
    except Exception:
        return []


def manifiesto(carpeta, marca=None, cuenta=None, contexto=None, par=None):
    c = Path(carpeta)
    armada = carga(c / 'SOMBRA_ARMADA.json') or {}
    ejec = carga(c / 'SOMBRA_EJECUCION.json') or {}
    inicio = carga(c / 'salidas' / 'inicio_ejecutor.json') or {}
    reg = lineas(c / 'salidas' / 'registro.jsonl')
    fronteras = lineas(c / 'salidas' / 'fronteras.jsonl')
    veredicto_ordenes = (carga(cuenta) or {}).get('veredicto') if cuenta else None
    decisiones = lineas(c / 'decisor_llm.jsonl')

    def ev(tipo):
        return [r for r in reg if r.get('tipo') == tipo]

    al = (ev('sombra_corte_alineado') or [{}])[0].get('contenido', {})
    fr = [r.get('contenido', {}) for r in ev('frontera')]
    f0 = fr[0] if fr else {}
    E = f0.get('E_source', {})
    scan = E.get('scan', {})
    nav = (f0.get('v2') or {}).get('navegacion', {})
    umbral = armada.get('umbral', {})
    ruta_marca = marca or umbral.get('marca')
    m = carga(ruta_marca) if ruta_marca else None
    acuse = carga(str(ruta_marca) + '.acuse') if ruta_marca else None

    t_marca = (al.get('marca') or {}).get('pared_s') or (m or {}).get('pared_s')
    t_captura = E.get('captura_pared_s')
    t_pose = E.get('pose_stamp')
    t_scan = scan.get('stamp')

    def resta(a, b):
        return None if a is None or b is None else round(a - b, 4)

    tiempos = {
        'armada_pared_s': armada.get('armada_pared_s'),
        'alineado_marca_pared_s': t_marca,
        'marca_leida_pared_s': al.get('leida_pared_s'),
        'congelado_captura_pared_s': t_captura,
        'corte_sensor_s': E.get('corte_sensor_s'),
        'acuse_pared_s': (acuse or {}).get('pared_s'),
        'componentes': {
            'scan_stamp_s': t_scan,
            'pose_stamp_s': t_pose,
            'plan_sello_s': nav.get('plan_sello_s'),
        },
        'derivados_s': {
            'marca_a_captura': resta(t_captura, t_marca),
            'edad_pose_en_captura': resta(t_captura, t_pose),
            'edad_scan_en_captura': resta(t_captura, t_scan),
            'scan_menos_pose': resta(t_scan, t_pose),
            'marca_a_acuse': resta((acuse or {}).get('pared_s'), t_marca),
        },
    }
    frescura = umbral.get('frescura_s')
    d = tiempos['derivados_s']
    huellas_decl = ejec.get('huellas', {})
    huellas_ok = {k: (sha(c / k) == v) for k, v in huellas_decl.items() if (c / k).is_file()}

    def dentro(x):
        return None if x is None or frescura is None else (0 <= x <= frescura)

    comprob = {
        'tiempos': {
            'una_sola_frontera_alineada': len(fronteras) == 1 and fronteras[0].get('tipo') == 'continuacion:alineado',
            'captura_posterior_a_la_marca': None if d['marca_a_captura'] is None else d['marca_a_captura'] > 0,
            'marca_posterior_al_armado': None if t_marca is None or not armada.get('armada_pared_s') else t_marca > armada['armada_pared_s'],
            'robot_parado_declarado': (bool((al.get('marca') or {}).get('nomotion_previa'))
                                       or (al.get('marca') or {}).get('quien') == 'summit_ab_robusto.sh')
                                      and bool((f0.get('parada') or {}).get('quieto_mono_ns')),
            'ids_de_travesia_y_corte_casan': bool(fronteras) and str(fronteras[0].get('boundary_id', '')).split('/')[0] == (ejec.get('id') or c.name) == fronteras[0].get('episode_id'),
        },
        'frescura': {
            'umbral_declarado_s': frescura,
            'pose_fresca': dentro(d['edad_pose_en_captura']),
            'scan_fresco': dentro(d['edad_scan_en_captura']),
            'pose_posterior_a_la_marca': None if t_pose is None or t_marca is None else t_pose > t_marca,
            'fuente_pose': E.get('fuente_pose'),
        },
        'sincronia': {
            'scan_menos_pose_s': d['scan_menos_pose'],
            'scan_y_pose_dentro_del_umbral': None if d['scan_menos_pose'] is None or frescura is None else abs(d['scan_menos_pose']) <= frescura,
        },
        'integridad': {
            'scan_presente': bool(scan.get('ranges')),
            'pose_presente': bool(E.get('pose_amcl')),
            'tf_laser_presente': bool(E.get('tf_laser')),
            'proyeccion_dkp_presente': bool((f0.get('v2') or {}).get('v3')),
            'huellas_declaradas_coinciden': all(huellas_ok.values()) if huellas_ok else None,
            'cadena_del_registro_enlazada': all(r.get('prev_sha256') for r in reg) and (reg[0].get('prev_sha256') == 'genesis' if reg else False),
            'ordenes_de_la_sombra': veredicto_ordenes or 'SIN_CONTADOR (se mide aparte con cuenta_ordenes_sombra.py)',
            'acuse': (acuse or {}).get('estado'),
            'rc': ejec.get('rc'),
        },
    }

    def todos(dic, claves):
        return all(dic.get(k) is True for k in claves)

    valido = (todos(comprob['tiempos'], ['una_sola_frontera_alineada', 'captura_posterior_a_la_marca', 'marca_posterior_al_armado', 'robot_parado_declarado', 'ids_de_travesia_y_corte_casan'])
              and todos(comprob['frescura'], ['pose_fresca', 'scan_fresco', 'pose_posterior_a_la_marca'])
              and comprob['sincronia']['scan_y_pose_dentro_del_umbral'] is True
              and todos(comprob['integridad'], ['scan_presente', 'pose_presente', 'tf_laser_presente', 'proyeccion_dkp_presente', 'cadena_del_registro_enlazada'])
              and veredicto_ordenes in (None, 'CERO_ORDENES_DE_LA_SOMBRA')
              and comprob['integridad']['huellas_declaradas_coinciden'] is not False
              and comprob['integridad']['rc'] == 0)

    ficheros = {str(p.relative_to(c)): sha(p) for p in sorted(c.rglob('*'))
                if p.is_file() and p.name != 'MANIFIESTO_CORTE.json'}
    man = {
        'manifiesto': VERSION,
        'context_id': contexto,
        'pair_id': par,
        'cut_id': (fronteras[0].get('boundary_id') if fronteras else None),
        'traversal_id': ejec.get('id') or c.name,
        't_aligned': t_marca,
        't_freeze': t_captura,
        'lote': ejec.get('lote'),
        'modo_decisor_en_linea': ejec.get('modo'),
        'estado': {
            'alineado': t_marca is not None,
            'congelado': bool(fronteras) and t_captura is not None,
            'valido': bool(valido),
            'clasificado_fuera_de_linea': False,
        },
        'tiempos': tiempos,
        'comprobaciones': comprob,
        'versiones': {
            'lazo': armada.get('version'),
            'orquestador': ejec.get('version'),
            'ejecutor': inicio.get('version'),
            'ejecutor_sha256': inicio.get('ejecutor_sha256'),
            'proyeccion_dkp': ((f0.get('v2') or {}).get('v3') or {}).get('version'),
            'huellas_declaradas': huellas_decl,
        },
        'validez_percepcion': {
            'enclavamiento_del_vigilante_de_la_sombra': ((f0.get('v2') or {}).get('seguridad') or {}).get('enclavada'),
            'observado_antes_del_corte': [r.get('contenido', {}).get('causa') for r in ev('sombra_enclavamiento_observado')],
            'sigma_xy_m': E.get('sigma_xy'), 'sigma_yaw_rad': E.get('sigma_yaw'),
            'nota': 'indicadores, no umbrales: aquí no se decide si la percepción vale',
        },
        'limites_conocidos': ['robot_parado_declarado sale de lo que declara el guion de la travesía (para la base y '
                              'pide pose antes de escribir la marca), no de una medida de velocidad',
                              'el sello de la pose es el del escaneo del corte: scan_menos_pose es 0 por construcción'],
        'evidencia_sha256': (fronteras[0].get('evidencia_sha256') if fronteras else None),
        'ficheros_sha256': ficheros,
        'nota': 'Datos de desarrollo. La decisión en línea es de respuesta fija y no cuenta como clasificación; '
                'clasificado_fuera_de_linea lo pone a True el paso de C2/C3 fuera de línea.',
    }
    (c / 'MANIFIESTO_CORTE.json').write_text(json.dumps(man, ensure_ascii=False, indent=1))
    return man


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('carpeta', nargs='?')
    ap.add_argument('--marca')
    ap.add_argument('--contexto', help='ContextID: summit_sim | summit_physical | g1_physical')
    ap.add_argument('--par', help='PairID: escenario nominal que comparten el registro simulado y el real')
    ap.add_argument('--cuenta', help='salida JSON de cuenta_ordenes_sombra.py, si se corrió')
    ap.add_argument('--lote')
    ap.add_argument('--intentadas', type=int, help='travesías lanzadas (hoja de la sesión), si alguna no dejó carpeta')
    a = ap.parse_args()
    if a.lote:
        ms = [carga(p) for p in sorted(Path(a.lote).glob('*/MANIFIESTO_CORTE.json'))]
        ms = [m for m in ms if m]
        n = lambda k: sum(1 for m in ms if m['estado'][k])
        print(json.dumps({'intentadas': a.intentadas if a.intentadas is not None else len(ms),
                          'con_carpeta': len(ms), 'alineadas': n('alineado'), 'congeladas': n('congelado'),
                          'validas': n('valido'), 'clasificadas': n('clasificado_fuera_de_linea'),
                          'no_validas': [m['traversal_id'] for m in ms if not m['estado']['valido']]},
                         ensure_ascii=False, indent=1))
        return
    if not a.carpeta:
        ap.error('falta la carpeta')
    m = manifiesto(a.carpeta, a.marca, a.cuenta, a.contexto, a.par)
    print(json.dumps({'traversal_id': m['traversal_id'], 'estado': m['estado'],
                      'derivados_s': m['tiempos']['derivados_s'],
                      'comprobaciones': m['comprobaciones']}, ensure_ascii=False, indent=1))
    sys.exit(0 if m['estado']['valido'] else 3)


if __name__ == '__main__':
    main()
