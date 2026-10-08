# Masterarbeit: Aktive Kamerastabilisierung auf dem hybriden Laufroboter Unitree A2W

Dieses Repository enthält den Quellcode (MATLAB/Simulink, Python, ROS) sowie die Simulationsmodelle, die im Rahmen meiner Masterarbeit entwickelt werden. Gegenstand der Arbeit ist die systematische Evaluierung der Fortbewegungsdynamik der radbasierten vierbeinigen Plattform (Wheeled-Legged Robot) **Unitree A2W** sowie die modellbasierte Regelung eines darauf montierten Manipulators zur aktiven Stabilisierung eines Kamerasystems.

## Wissenschaftliche Zielsetzung
Die Arbeit fokussiert sich auf die Schnittstelle zwischen mobiler Lokomotion und präziser Manipulation. Im Kern werden folgende Forschungsfragen untersucht:
1.
2. 
3. 

## Repository-Struktur
Das Projekt ist modular aufgebaut, um die verschiedenen Softwareschichten (Simulation, Regelung, Hardware-Kommunikation) sauber zu trennen:

* **`Docs/`** - Dokumentation, Spezifikationen, Literatur-Metadaten und das Exposé.
* **`Matlab/`** - Kernbereich der Simulation und Regelung.
* **`Python/`** - Skripte für die Auswertung, Datenanalyse und das Benchmarking der Testergebnisse.
* **`ros/`** - ROS-Workspaces (Robot Operating System) für die Kommunikation und den Datenaustausch mit der realen Unitree A2W Hardware.

## Systemanforderungen
Um die Modelle reproduzieren und simulieren zu können, wird folgende Softwareumgebung vorausgesetzt:
* **MATLAB / Simulink:** Release R2026a (oder neuer)


## Autor
Johannes Hartmann
Hochschule für Technik und Wirtschaft (HTW) Berlin
Studiengang: Elektrotechnik (Master)
