#!/usr/bin/env python3
"""Construye nav2_sim.yaml para el simulador 2D a partir de los `ros2 param dump` reales del robot (17-sep-2026).

Todo lo que se pudo extraer del robot va tal cual: AMCL, bt_navigator, planner (NavFn), los dos costmaps, behavior
server, velocity_smoother, smoother_server, map_server. Lo único que no es del robot es el controlador: TEB no está en
la estación, así que el controller_server sale de la plantilla de nav2_bringup con DWB y los límites de TEB
(0,25 m/s, 0,6 rad/s, acc 0,2/0,5). Para la puerta importa el planificador, los costmaps y el behavior server, no TEB.

  python3 genera_params_sim.py DUMPS_DIR MAPA_YAML SALIDA_YAML
"""
import copy, sys, yaml

dumps, mapa, salida = sys.argv[1:4]
PLANTILLA = "/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml"
tpl = yaml.safe_load(open(PLANTILLA))
out = {}


def dump(nombre):
    d = yaml.safe_load(open("%s/p_%s.yaml" % (dumps, nombre)))
    k = list(d.keys())[0]                      # p. ej. /robot/local_costmap/local_costmap
    return d[k]["ros__parameters"]


def limpia(p):
    """Quita lo que no es del robot y arregla un artefacto de `ros2 param dump`: las listas de cadenas vacías salen
    como '' y Nav2 aborta al leerlas («parameter_value_from failed for parameter 'filters'»); un `[]` tampoco vale
    (una lista vacía no tiene tipo para rclcpp): la clave se quita y manda el valor por defecto."""
    for k in ("use_sim_time", "qos_overrides", "start_type_description_service"):
        p.pop(k, None)
    for k in [k for k in p if k.startswith("/")]:
        p.pop(k)        # parámetros globales del dump (p. ej. /bond_disable_heartbeat_timeout): el nodo ya los declara
    for k, v in list(p.items()):
        if isinstance(v, dict):
            limpia(v)
        elif v in ("", []) and k in ("observation_sources", "filters", "plugins", "critics", "behavior_plugins", "controller_plugins",
                               "goal_checker_plugins", "planner_plugins", "smoother_plugins", "plugin_lib_names", "navigators",
                               "error_code_names", "waypoint_task_executor_plugin"):
            p.pop(k)
    return p


# --- del robot, tal cual --------------------------------------------------------------------------------------
for n in ("amcl", "bt_navigator", "planner_server", "behavior_server", "velocity_smoother", "smoother_server", "map_server"):
    out[n] = {"ros__parameters": limpia(dump(n))}
for n in ("local_costmap", "global_costmap"):
    out[n] = {n: {"ros__parameters": limpia(dump(n))}}

out["map_server"]["ros__parameters"]["yaml_filename"] = mapa
out["map_server"]["ros__parameters"]["topic_name"] = "map"          # relativo: con namespace queda /robot/map
out["amcl"]["ros__parameters"]["map_topic"] = "map"
gc = out["global_costmap"]["global_costmap"]["ros__parameters"]
for capa in ("static_layer",):
    if capa in gc and isinstance(gc[capa], dict):
        gc[capa]["map_topic"] = "/robot/map"                           # en el robot era /robot/map_nav (copia del mismo mapa)
gc["map_topic"] = "/robot/map"
out["local_costmap"]["local_costmap"]["ros__parameters"]["map_topic"] = "/robot/map"
for n in ("bt_navigator",):
    p = out[n]["ros__parameters"]
    for k in list(p):
        if k.startswith("default_nav") and "bt_xml" in k:
            p.pop(k)                                                    # que use el BT por defecto de la estación

# --- controlador: plantilla con DWB, límites del TEB real ---------------------------------------------------------
cs = copy.deepcopy(tpl["controller_server"]); p = cs["ros__parameters"]
p["odom_topic"] = "/robot/robotnik_base_control/odom"
p["controller_frequency"] = 20.0
fp = p["FollowPath"]
fp.update({"max_vel_x": 0.25, "min_vel_x": -0.15, "max_vel_y": 0.0, "min_vel_y": 0.0, "max_vel_theta": 0.6, "min_speed_xy": 0.0,
           "max_speed_xy": 0.25, "min_speed_theta": 0.0, "acc_lim_x": 0.2, "acc_lim_y": 0.0, "acc_lim_theta": 0.5,
           "decel_lim_x": -0.2, "decel_lim_y": 0.0, "decel_lim_theta": -0.5, "vx_samples": 20, "vy_samples": 1, "vtheta_samples": 20})
out["controller_server"] = cs
for n in ("waypoint_follower",):
    out[n] = copy.deepcopy(tpl[n])

yaml.safe_dump(out, open(salida, "w"), sort_keys=False, allow_unicode=True)
print("escrito %s: %s" % (salida, ", ".join(out)))
