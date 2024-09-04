# eval-dynamic

## Beschreibung der Skripte

- [`calibrated.py`](calibrated.py) Visualisierung der kalibrierten Frequenzen
- [`calibrated_pwm.py`](calibrated_pwm.py) Formatierungshilfe für LaTeX der gemessenen PWM Frequenzen
- [`config.py`](config.py) Diverse allgemeine Konfigurations-Optionen für die Analyse + Auswertung
- [`debug.py`](debug.py) Debug-Optionen
- [`dynamic-conceptual.py`](dynamic-coneptual.py) Versuch, alles bis auf die automatische Positionserkennung zum Laufen zu bringen *(nicht mehr verwendet)*
- [`dynamic.py`](dynamic.py) Analyse-Programm
- [`image_sequence.py`](image_sequence.py) Speichern von Bildern und Positionserkennung des Gerätes
- [`plotter.py`](plotter.py) Auswertungs-Programm
- [`plotter_bad.py`](plotter_bad.py) Auswertungs-Programm; Erste Version *(nicht mehr verwendet)*
- [`position_sequence.py`](position_sequence.py) Speichern von Positionen und PWM-Kalibration

Wird `dynamic.py` auf ein Video angewendet, werden Messwerte in `measurements` gespeichert. 
Nach und nach werden gleichzeitig in `measurements/plotcommands.txt` äquivalente Python-Funktionsaufrufe
angefügt, welche in `plotter.py` verwendet werden können.

Wird `dynamic.py` nur auf einer Videodatei angewendet, wird bei Beenden direkt geplottet.
Es ist auch möglich, mehrere Videodateien zugleich abzuarbeiten.
Dann werden jedoch weniger Informationen ausgegeben und nicht geplottet.
(Die `measurements/plotcommands` werden in jedem Fall erstellt.)

