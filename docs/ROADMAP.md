# Roadmap

## v1 — en place

- [x] Trois AMR, namespaces `/robot_1` `/robot_2` `/robot_3`, TF séparés, `/tf` global.
- [x] Carte d’entrepôt partagée, un seul publisher `/map`.
- [x] A* par robot. Le chemin reste à 0,25 m des obstacles. Le suivi colle à la ligne. Le pivot n’a lieu que si un côté est un mur (moins de 0,50 m) et l’autre laisse 0,80 m.
- [x] `fleet_manager` : mission au robot libre le plus proche.
- [x] Recovery : timeout sans progression, exclusion, cooldown, retry si tout le monde est exclu.
- [x] Tests unitaires de l’affectation et d’un chemin sur la carte réelle.
- [x] Cartographie à un robot (`start_slam_demo.sh`) et export `save_map.py`, comme le projet 1.
- [x] La démo charge `src/fleet_nav/maps/warehouse_map`.
- [x] Clic RViz : sélection du robot, puis cible (`/fleet/direct`).
- [x] Évitement lidar des robots et des personnes : parallèle à 0,75 m, droite de chacun en face à face, arrêt si un mobile coupe la trajectoire par la droite.
- [x] Trois personnes en allée, absentes du monde de cartographie.

## Hors de ce dépôt

- [ ] Nav2 multi-instance à la place de l’A* custom.
- [ ] Open-RMF.
- [ ] SLAM distribué, une carte par robot.
- [x] Publication GitHub : [Maxence-Santos/ros2-fleet-nav2](https://github.com/Maxence-Santos/ros2-fleet-nav2).
