\# negative\_obstacle\_detector csomag



ROS 2 Humble környezetben futó, fejlett C++ autonóm terepjáró csomag negatív akadályok (árkok, leszakadások, mély kátyúk) valós idejű detektálására 3D LiDAR pontfelhő alapján, tereplejtés-becsléssel és 3D műszerfali vizualizációval.



\## Készítő

\* \*\*Név:\*\* Kincses Ádám

\* \*\*Neptun-kód:\*\* ZY2STZ

\* \*\*Tantárgy:\*\* Autonóm járművek és robotok programozása



\---



\## Vizualizáció és Fejlesztési Mérföldkövek



\### 1. Végleges rendszer: Dinamikus aszimmetrikus terep, 3D Pickup és lejtésmérő HUD

A jármű valós időben követi a domborzatot, becsli a hosszanti lejtést (`CLIMB / DESCENT %`) és az oldalirányú dőlést (`BANK %`), valamint osztályozza az akadályokat:



!\[Végleges rendszer](rviz\_screenshot.png)



\### 2. Köztes állapot: Dinamikus akadálykövetés és távolságbecslés

Az akadályok távolságának és letörési mélységének folytonos számszerűsítése RViz lebegő HUD felirattal:



!\[Köztes állapot](rviz\_screenshot1.png)



\### 3. Kezdeti funkcionális prototípus

Poláris szektorokra bontott LiDAR pontfelhő feldolgozása diszkrét árokperem-detektálással:



!\[Kezdeti prototípus](rviz\_screenshot2.png)



\---



\## Rendszerarchitektúra és Adatfolyam



```mermaid

graph LR

&#x20;   P1\[sensor\_height param] --> D\[detector\_node]

&#x20;   P2\[drop\_threshold param] --> D

&#x20;   P3\[min\_range / max\_range] --> D

&#x20;   

&#x20;   PC\[/points\_raw<br/>sensor\_msgs/PointCloud2/] --> D

&#x20;   

&#x20;   D --> M\[/negative\_obstacles\_marker<br/>visualization\_msgs/MarkerArray/]

&#x20;   D --> W\[/hazard\_warning<br/>std\_msgs/Bool/]

&#x20;   D --> DIST\[/hazard\_distance<br/>std\_msgs/Float32/]

&#x20;   D --> SLP\[/terrain\_slope\_pct<br/>std\_msgs/Float32/]

&#x20;   

&#x20;   MOCK\[mock\_terrain\_node] --> PC

&#x20;   MOCK --> CAR\[/vehicle\_marker<br/>visualization\_msgs/MarkerArray/]

