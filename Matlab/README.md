# Kamerastabilisierung Unitree Z1 auf A2W

Kamera am Z1 hält ihre Orientierung, während der A2W fährt.
Grundgerüst nach Roncone (2014): Gyro-Aufschaltung + Rückführung über J_ω#.
Doku: Lern-Dokumentation „Stufe 1 und 2" (PDF, Release v0.2-stufe2).

## Bedienung
`start_model.m` ausführen → Parameter, Tests, alle Testfälle in Simulink,
Grafiken und `93_Kennwerte.csv` in `ergebnisse/` (werden jedes Mal überschrieben).
Am Ende prüft `pruefe_regression` automatisch, ob Stufe 1 und der Totzeit-Test stimmen.

Schalter in `start_model.m`, Abschnitt 10:
| Schalter | Wirkung |
|---|---|
| `STUFE` | aktive Variante beim manuellen Arbeiten im Modell (1 ideal, 2 realistisch) |
| `TESTLAUF` | `false` → nur Modell öffnen |
| `STUFEN` | welche Stufen alle Basis-Testfälle rechnen, z. B. `[1 2]` |
| `V_STUDIE` | zusätzlich v = 1…5 m/s, Posen vorn/seite (20 Läufe) |
| `TAU_STUDIE` | zusätzlich Stufe 2 mit τ = 2…40 ms (10 Läufe) |
| `DETAILS_ANZEIGEN` | `true` → auch alle Detail-Figures öffnen |

## Stufen (Variant Subsystems „Sensor IMU" und „Gelenkmodell")
| Stufe | Sensor | Gelenk | Zweck |
|---|---|---|---|
| 1 ideal | durchverbunden | q = ∫ q̇_cmd | Mathematik prüfen (Fehler ≈ 0) |
| 2 realistisch | ZOH 2 ms + Totzeit τ | ZOH → Sättigung → PT1 → ∫ mit Grenzen | reale Grenzen |

## Ordner
| Ordner | Inhalt |
|---|---|
| `parameter/` | SIM, Z1 (aus URDF), MONT, STOER, REG, SEN (τ), GEL (T_Gelenk, Grenzen) |
| `funktionen/` | Kinematik, Jacobi, Fehler, Pseudoinverse, Störung, Plots |
| `tests/` | Prüfung jedes Bausteins (Toolbox nur hier) |
| `auswertung/` | Testfälle, Simulation, Grafiken, Regressionstest |
| `referenz/stufe2/` | Ergebnisse v0.2 als Vergleichsbasis für Add-ons |
| `Kamerastabilisierung.slx` | Regelkreis |

## Stand (v0.2-stufe2)
- [x] Stufe 1 (ideal): Kamerafehler −99,98 %, Rückführung = PT1
- [x] Stufen umschaltbar, Stufe 1 als Regressionstest
- [x] Stufe 2: −65 % bei 2 m/s (τ = 20 ms), Totzeit-Test = Theorie
- [x] Studien: v = 1…5 m/s (78 → 44 %), τ = 2…40 ms (85 → 46 %)
- [ ] IMU-Rauschen und Bias im Sensor
- [ ] Add-on Prädiktion (Woolfrey 2021)
- [ ] Stufe 3: Simscape-Dynamik

Erkenntnis: Begrenzend ist die effektive Verzögerung τ_eff = τ + T_s/2 + T_Gelenk,
nicht die Gelenkgeschwindigkeit.

Platzhalter (T_6C, T_B0, A2W-Geometrie, τ, T_Gelenk) sind im Code mit `PLATZHALTER` markiert.