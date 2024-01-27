#import "template.typ": *
#show: ams-article.with(
  title: [JIF - Roboter & Controller GUI],
  authors: (
    (
      name: "Jonas Nicklas",
    ),
    (
      name: "Ilian Odenbach",
    ),
    (
      name: "Felix Schreiber",
    ),
  ),
  abstract: {},
  bibliography-file: "refs.bib",
)

= Containerisierung des ROS2 environments für Entwicklung auf einem nicht-Ubuntu Gerät

Zum aufsetzen eines simplen Containers für ROS2 mussten wir uns zuerst mehrere essentielle Fragen stellen: Welche Ordner sollten dauerhaft bestehen bleiben? Wollen wir graphische Oberflächen unterstützen? Welche Software wollen wir nutzen? Wie speichern wir unseren Containerstand zwischen?

== Podman
Podman ist unser Container Manager. Jetzt fragt man sich, warum nicht Docker? Hier eine kurze Übersicht warum ich mich für Podman entschieden habe.

- Kein Daemon, daher keine verpflichtenden Admin Rechte
  - Daher keine Nutzerrollen nötig um einen Container ohne sudo rechte zu starten


- Sogenannte 'Pods', die es ermöglichen mehrere Container zu gruppieren und zu Verwalten
  - Daher ist es auch möglich in diesem Fall eine Robotersimulation und einen Client-Container in ein privates Netzwerk zu packen und eine ungehinderte Simulation bzw. Testphase ablaufen zu lassen.

\
 Außerdem hab ich mehrere Probleme mit Docker gehabt und mich deshalb dazu entschieden mal etwas anderes zu probieren.

== Graphische Oberflächen
Ich dachte mir zu beginn, dass ein reiner Developnment-Container keine GUIS bracht, da es ja genug Terminal Text-Editioren gibt. Nach jedoch etwas Überlegung sollte klar sein, dass ein GUI zur Kontrolle des Roboters fast unverzichtbar ist (Beispielsweise für die Kamera).

#sc[```bash
podman run -it \
    # ...
    --env DISPLAY=${DISPLAY} \ #Für Übergabe der Display env-var
    --env PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native \ #Für die audio Übergabe der GUIs
    --env PULSE_COOKIE=/run/pulse/cookie \ #Audio
    --env QT_AUTO_SCREEN_SCALE_FACTOR=1 \ #Set QT screen scale to 1 so you dont get weird scaling issues
    --env XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR} \ #Ordner mit Sockets und Pipes
#Viele verschiedene Geräte oder virtuelle Geräte für die Video Übertragung
    --device /dev/dri \
    --device /dev/input \
    --device /dev/nvidia0 \
    --device /dev/nvidiactl \
    --device /dev/nvidia-modeset \
    --device /dev/nvidia-uvm \
    --device /dev/nvidia-uvm-tools \
#Mounten(Anschließen) von Ordnern, die für Videobearbeitung verantwortlich sind
    --mount type=bind,source=${XDG_RUNTIME_DIR}/pipewire-0,target=${XDG_RUNTIME_DIR}/pipewire-0 \
    --mount type=bind,source=$XAUTHORITY,target=/tmp/.host_Xauthority,readonly \
    --mount type=bind,source=/etc/localtime,target=/etc/localtime,readonly \
    --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix \
    --mount type=bind,source=${XDG_RUNTIME_DIR}/pulse/native,target=${XDG_RUNTIME_DIR}/pulse/native \
    --mount type=bind,source=${HOME}/.config/pulse/cookie,target=/run/pulse/cookie \
    --mount type=bind,source=${HOME}/.local/share/fonts,target=/usr/share/fonts,readonly \
    --mount type=bind,source=/usr/share/fonts,target=/root/.local/share/fonts \
    # ...

```]
