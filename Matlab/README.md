# Kamerastabilisierung Unitree Z1 auf A2W

Kamera am Z1 hält ihre Orientierung, während der A2W fährt.
Grundgerüst nach Roncone (2014): Gyro-Aufschaltung + Rückführung über J_ω#.
Doku: Bericht „Kamerastabilisierung Z1 auf A2W – Stufe 1" (Claude-Doc).

## Bedienung
`start_model.m` ausführen → Parameter, Tests, alle Testfälle in Simulink,
Grafiken in `ergebnisse/` (werden bei jedem Lauf überschrieben).
- Grundpose: `par_sim("vorn" | "seite")`
- Störung: `par_stoerung("fahren", SIM, v=2.0, klasse="E")`
- `TESTLAUF = false` → nur Modell öffnen; `DETAILS_ANZEIGEN = true` → alle Figures

## Ordner
| Ordner | Inhalt |
|---|---|
| `parameter/` | SIM, Z1 (aus URDF), MONT, STOER, REG |
| `funktionen/` | Kinematik, Jacobi, Fehler, Pseudoinverse, Störung, Plots |
| `tests/` | Prüfung jedes Bausteins (Toolbox nur hier) |
| `auswertung/` | Testfälle, Simulation, Grafiken |
| `Kamerastabilisierung.slx` | Regelkreis |

## Stand
- [x] Stufe 1 (ideal): Kamerafehler −99,98 %, Rückführung = PT1
- [ ] Stufen umschaltbar (Variant Subsystems), Stufe 1 als Regressionstest
- [ ] Stufe 2: Abtastung, Totzeit, PT1, Sättigung
- [ ] Stufe 3: Simscape-Dynamik

Platzhalter (T_6C, T_B0, A2W-Geometrie, τ) sind im Code mit `PLATZHALTER` markiert.