# 3D LiDAR Negative Obstacle & Terrain Slope Detector

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![C++](https://img.shields.io/badge/Language-C%2B%2B17-orange.svg)]()
[![PCL](https://img.shields.io/badge/Library-PCL_1.12-green.svg)](https://pointclouds.org/)

Valós idejű autonóm terepjáró szoftvercsomag negatív tereptárgyak (árkok, útszakadások, mély kátyúk) detektálására, tereplejtés- és rézsűbecslésre, valamint 3D RViz műszerfali megjelenítésre 3D LiDAR adatok alapján.

**Készítő:** Kincses Ádám (Neptun: ZY2STZ)  
**Tantárgy:** Autonóm járművek és robotok programozása  
**Nyilatkozat:** A dokumentáció, a szimulációs környezet és a C++ forráskódok fejlesztése a Google Gemini (LLM) segítségével készült.

---

## Működés közben (Demo)

<p align="center">
  <img src="./demo.gif" width="850" alt="Élő szimuláció" onerror="this.onerror=null; this.src='./rviz_screenshot.png';" />
</p>

---

## Fő Képességek

* **Robusztus negatív akadály detektálás:** Sugárirányú poláris szektorokra (Azimuthal Slicing) bontott pontfelhő-feldolgozás, amely kiszűri a lankás domboldalakat és csak a kritikus szinttöréseket azonosítja.
* **Valós idejű lejtés- és rézsűmérés:** Menetirány szerinti hosszirányú emelkedés/lejtés (`CLIMB / DESCENT %`) és keresztirányú oldaldőlés (`BANK %`) számítása.
* **Veszélyosztályozás és manőverjavaslat:**
  * `ROAD TRENCH` / `EXTREME RAVINE` $ightarrow$ Automatikus lassítás vagy vészfék (`DECELERATE` / `EMERGENCY BRAKE`).
  * `LANE COLLAPSE` (kátyú, féloldali omlás) $ightarrow$ Elkerülő sáv javaslata (`STEER LEFT / RIGHT`).
* **Beépített szimuláció:** Procedurális 3D terep változatos akadályokkal és merev testként felépített terepjáró pickup modellel.

---

## Fejlesztési Mérföldkövek

| 1. Kezdeti Prototípus | 2. Akadálykövetés & HUD | 3. Teljes Aszimmetrikus Terep |
| :---: | :---: | :---: |
| <img src="./rviz_screenshot2.png" width="280" /> | <img src="./rviz_screenshot1.png" width="280" /> | <img src="./rviz_screenshot.png" width="280" /> |
| Diszkrét sugárirányú peremdetektálás | Dinamikus mélység- és távolságmérés | 3D Pickup, lejtésszámítás & döntési logika |

---

## Rendszerarchitektúra és Topic Interfész

```mermaid
graph LR
    subgraph Bemenet
        PC[/points_raw<br/>PointCloud2/]
    end
    
    subgraph Feldolgozás
        D[detector_node<br/>Polar Slicing & Slope Est.]
    end
    
    subgraph Kimeneti Topicok
        D --> M[/negative_obstacles_marker<br/>MarkerArray/]
        D --> W[/hazard_warning<br/>Bool/]
        D --> DIST[/hazard_distance<br/>Float32/]
        D --> SLP[/terrain_slope_pct<br/>Float32/]
    end
    
    PC --> D
```

### Publikált témakörök járművezérléshez
| Topic | Típus | Funkció |
|---|---|---|
| `/hazard_warning` | `std_msgs/msg/Bool` | Aktív veszélyjelzés (`true` / `false`) |
| `/hazard_distance` | `std_msgs/msg/Float32` | Legközelebbi akadály távolsága méterben |
| `/terrain_slope_pct` | `std_msgs/msg/Float32` | Útfelület aktuális hosszirányú emelkedése/lejtése (%) |
| `/negative_obstacles_marker` | `visualization_msgs/msg/MarkerArray` | Árokperem, jelölőgömbök és a 3D HUD műszerfal |
| `/vehicle_marker` | `visualization_msgs/msg/MarkerArray` | 3D terepjáró járműmodell |

---

## Telepítés és Futtatás

### Fordítás
```bash
cd ~/ros2_ws
colcon build --packages-select negative_obstacle_detector --symlink-install
source install/setup.bash
```

### Indítás
```bash
ros2 launch negative_obstacle_detector demo.launch.py
```

---

## Hivatkozások és Felhasznált Irodalom
1. **ROS 2 Documentation (Humble Hawksbill):** Standard interfaces, Node and Launch system ([docs.ros.org](https://docs.ros.org/en/humble/))
2. **Point Cloud Library (PCL):** Point Cloud conversions, radial geometry analysis ([pointclouds.org](https://pointclouds.org/))
3. **RViz2 Visualization Tutorials:** MarkerArray & PointCloud2 display conventions
