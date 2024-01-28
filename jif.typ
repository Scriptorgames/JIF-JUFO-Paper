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

- Mir kommt das Containerstand speichern sehr simpel vor. Dies ist für später häufiger notwendig.

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
    # Das ist n bissel komisch eingerückt ...

```]

#pagebreak()

== Synchronisierte Ordner
Das Folgende ist nötig, da sonst die Konfigurationen und Änderungen verloren gehen würden. Im Grunde Mounten wir nur Konfig Dateien und den Workspace um keinen Datenverlust zu haben und damit unsere Daten nicht davon abhängig sid, ob der Container noch exestiert oder nicht.

#sc[```bash
# ...
    --mount type=bind,source=${HOME}/Dokumente/Docker/ros2-humble/workspace,target=/root/workspace\
    --mount type=bind,source=${HOME}/Dokumente/Docker/ros2-humble/shared,target=/root/shared\
    --mount type=bind,source=${HOME}/Dokumente/Docker/ros2-humble/.bashrc,target=/root/.bashrc \
    --mount type=bind,source=${HOME}/Dokumente/Docker/ros2-humble/.config,target=/root/.config \
# ...
```]

== Containerstände Zwischenspeichern

#term(
    ps1: [`$`], input:[podman export {ContainerName} -o {Dateiname}.tar],
    output:[Dieser Befehl speichert den momentanen Containerzustand in einer tar Datei])
Der Containerzustand ist jetzt in einer Datei gespeichert, jedoch ist es so nicht so simpel einen neuen Container mit `podman run` zu starten.
Daher müssen wir nun den Containerzustand als eigene Vorlage für einen neuen Container speichern.
#term(ps1: [`$`], input:[podman import {DateiName(Unsere tar)} {template-name}],
    output:[Dieser Befehl sorgt dafür, dass der Container der Datei als template genommen werden kann und daher Dinge wie `podman run {template-name}` möglich sind])

#pagebreak()
== Gesammtergebniss

#sc[```bash
#!/bin/bash

# sudo damit network-host funktioniert. Dies ist für ROS fast zwangsweise notwendig.
sudo podman run -it \
    --name ros2-humble-io \
    --userns=keep-id \
    --network=host \
    --env DISPLAY=${DISPLAY} \
    --env PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native \
    --env PULSE_COOKIE=/run/pulse/cookie \
    --env QT_AUTO_SCREEN_SCALE_FACTOR=1 \
    --env XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR} \
    --device /dev/dri \
    --device /dev/input \
    --device /dev/nvidia0 \
    --device /dev/nvidiactl \
    --device /dev/nvidia-modeset \
    --device /dev/nvidia-uvm \
    --device /dev/nvidia-uvm-tools \
    --device /dev/bus/usb \
    --device-cgroup-rule='c 81:* rmw' \
    -v /dev/video0:/dev/video0 \
    --mount type=bind,source=/sys,target=/sys \
    --mount type=bind,source=${HOME}/Dokumente/Docker/ros2-humble/workspace,target=/root/workspace\
    --mount type=bind,source=${HOME}/Dokumente/Docker/ros2-humble/shared,target=/root/shared\
    --mount type=bind,source=${HOME}/Dokumente/Docker/ros2-humble/.bashrc,target=/root/.bashrc \
    --mount type=bind,source=${HOME}/Dokumente/Docker/ros2-humble/.config,target=/root/.config \
    --mount type=bind,source=${XDG_RUNTIME_DIR}/pipewire-0,target=${XDG_RUNTIME_DIR}/pipewire-0 \
    --mount type=bind,source=$XAUTHORITY,target=/tmp/.host_Xauthority,readonly \
    --mount type=bind,source=/etc/localtime,target=/etc/localtime,readonly \
    --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix \
    --mount type=bind,source=${XDG_RUNTIME_DIR}/pulse/native,target=${XDG_RUNTIME_DIR}/pulse/native \
    --mount type=bind,source=${HOME}/.config/pulse/cookie,target=/run/pulse/cookie \
    --mount type=bind,source=${HOME}/.local/share/fonts,target=/usr/share/fonts,readonly \
    --mount type=bind,source=/usr/share/fonts,target=/root/.local/share/fonts \
    --rm \
    --workdir /root/workspace \
    ros2-humble-io \
    /bin/bash

```]
Das -it am Anfang steht für interactive Terminal. Dadurch kriegen wir eine SSH artige Shell die /bin/bash (Das man Ende Steht), also ganz normales Bash, öffnet. --workdir bestimmt, in welchem Ornder unser Terminal gestartet wird. Dies ist aus Praxisgründen unser Workspace. --rm sorgt dafür, dass der Container gelöscht wird, wenn er beendet wird. Dies ist kein Problem da wir alle wichtigen Daten an user reelles System gemountet haben.

#pagebreak()

= Kamera publisher
Damit der Fahrer des Roboters auch etwas sieht muss das Kamerabild irgendwie übertragen werden. Dies machen wir über den ROS2 image transport. Das Kamera Bild fangen wir mit OpenCV ab.

#sc[```cpp
// Ein Haufen imports

using namespace std::chrono_literals;

class MinimalImagePublisher : public rclcpp::Node {
public:
  // Der Konstruktor

private:
  //mehrere Funktionen
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // erstelle ROS Node
  auto node = std::make_shared<MinimalImagePublisher>();
  // ausführen der Ros calls bis zum Absturz oder Ctrl-C
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```]
Die main funktion initialisiert zuerst ROS2 und erstellt dann eine ROS2 Node mit dem MinimalImagePublisher. Dann wird der ROS Cyclus gestartet. Es wird die timer_callback Funktion in einem Loop ausgeführt, bist Ctrl-C gedrückt wird. Der Delay zwischen den Loops ist in timer\_ definiert.
#v(12pt)
\
*Der Konstruktor:*
#sc[```cpp
  MinimalImagePublisher() : Node("opencv_image_publisher"), count_(0) {
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("campub", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalImagePublisher::timer_callback, this));
  }
```]
Im Konstruktor wird die publisher node definiert. Diese sendet eine image sensor message. Der timer bestimmt den delay zwischen den loops.

#pagebreak()

*Die Funktion*
#sc[```cpp

  void timer_callback() {
    cv::Mat image;
    cv::VideoCapture cap(0);
    if (!cap.isOpened()){
      std::cout << "cannot open camera";
      return;
    }
    cap >> image;

    msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image)
              .toImageMsg();
    publisher_->publish(*msg_.get());
    RCLCPP_INFO(this->get_logger(), "Frame %ld published", count_);
    count_++;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  size_t count_;
```]
Zu Beginn wird eine Matrix erstellt die image heißt. Diese ist einfach nur eine Rohdarstellung der Bilddaten. Dann wird ein VideoCapture namens cap erstellt. Die `0` bei `cap(0)` bestimmt die ausgewählte Kammera tendentiell könnte man da eine andere Nummer eingeben und es würde eine andere Kamera ausgewählt werden, falls es eine auf dieser Position gibt. Die Erfolgreiche erstellung des Video captures wird daraufhin in Z. 4ff. überprüft.

In Z. 8 setzen wir image auf das Bild, das cap momentan Besitzt, also das, das die Kamera momentan aufnimmt. Daraufhin wird eine Image message erstellt, die versendet werden kann. Dazu müssen wir von dem OpenCV Matrix Format erst einmal in das ROS kompatibele Image Format konvertieren. Dann publishen wir das Image und senden es in das Netzwerk. Dann geben wir noch aus, wieviele Frames bereits geschickt wurden und erhöhen den counter für die bereits geschickten Frames.
