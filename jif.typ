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

= Roboter
== Fahrgestell und Karosserie 
Das Fahrgestell ist nicht von uns designt worden, eben so wenig wie der Kettenantrieb. Dies war auch nicht notwendig, da Dejan von HowToMechatronics eine wunderbare Basis für unser Vorhaben bereits geschaffen hatte. @howtomechatronics2024 Auf dieser Basis wurde eine Karosserie für unsere Zwecke in Fusion360 erstellt mit zwei aufklappbaren Deckeln, die die Wartung an dem Roboter erleichtern. Um die elektronischen Komponenten auf dem Fahrgestell zu befestigen, wurden Adapterstücke konstruiert, da die Löcher des Fahrgestells für andere Komponenten designt wurden. Ebenso wurden stärkere Federn in den Dämpfern eingebaut die 10 Newton in etwa maximal Belastung haben. Der Schlitz auf der Oberseite des Hinterendeckels ist für die Kabelführung der Webcam.
#image("pictures/JifFusion.png")

#pagebreak()
== Hardware
Der Roboter wird von zwei 37mm 12V DC-Motoren mit 66rpm betrieben, welche angesteuert werden über zwei L298N H-Brücken, wobei jeder Motor mit einem der Motoren verkabelt ist. Die Steuerung der H-Brücken hat ein ESP32 development board, welches seriell mit einem Raspberry Pi 3B+ verbunden ist, inne. Die Stromversorgung übernimmt ein 12V 5000mAh LiPo-Akku, der die Motortreiber direkt versorgt, parallel dazu geschaltet ist ein Abwärtswandler, welcher die 12V auf 5V konvertiert und so den Raspberry Pi mit Strom versorgt. Man stellt sich sicherlich die Frage warum zwei H-Brücken verbaut wurden, wenn man an einem 2 Motoren anschließen kann, aber eine H-Brücke kann maximal 2 Ampere abgeben, was sich dann auch auf beide Motoren aufteilen würde was zu einem Leistungsverlust führt, da jedem Motor nur 1 Ampere zugewiesen werden kann. Um diese Problematik zu lösen wurden zwei H-Brücken verbaut und so können jedem Motor 2 Ampere zugewiesen werden. Leider ist aber immer noch ein Spannungsabfall zu bemerken. So wird ein zukünftiger Schritt sein ein Motortreiber zu verbauen, welcher eine noch höhere Stromstärke stemmen kann. 
#image("pictures/SchaltplanUpdate.png")
== micro-ROS
Um den ESP32 vom ROS2-Netzwerk aus anzusteuern, wurde micro-ROS verwendet. Dazu wurde zuerst ein neues Workspace erstellt, welches für das Flashen und dem micro-ROS-Agent zuständig sein wird. Anschließend noch ein src Ordner in dem micro-ROS Workspace erstellen, wo dann das micro_ros_setup Package hinein geklont wurde. @microros2024 Nachdem ersten Build ist nun ein firmware Ordner zu sehen, in dem Verzeichnis ,".../microros_ws/firmware/freertos_apps/apps", wurde dann die ros_esp32cam_diffdrive App geklont. @reinbert2024 Nach dem Konfigurieren und dem Builden konnte der ESP32 auch schon geflasht werden. Nun fehlte nur noch der micro-ROS-Agent. Nach dessen Builden und Starten konnte man unter den Topics auch unser /cmd_vel finden. Somit war der erste Teil der Basis geschaffen.
#pagebreak()



= Containerisierung des ROS2 environments für Entwicklung auf einem nicht-Ubuntu Gerät

Zum Aufsetzen eines simplen Containers für ROS2 mussten wir uns zuerst mehrere essenzielle Fragen stellen: Welche Ordner sollten dauerhaft bestehen bleiben? Wollen wir grafische Oberflächen unterstützen? Welche Software wollen wir nutzen? Wie speichern wir unseren Containerstand zwischen?

== Podman
Podman ist unser Container Manager. Jetzt fragt man sich, warum nicht Docker? Hier eine kurze Übersicht, warum ich mich für Podman entschieden habe.

- Kein Daemon, daher keine verpflichtenden Admin-Rechte
  - Daher keine Nutzerrollen nötig, um einen Container ohne sudo rechte zu starten


- Sogenannte 'Pods', die es ermöglichen mehrere Container zu gruppieren und zu Verwalten
  - Daher ist es auch möglich, in diesem Fall eine Robotersimulation und einen Client-Container in ein privates Netzwerk zu packen und eine ungehinderte Simulation bzw. Testphase ablaufen zu lassen.

- Mir kommt das Containerstand speichern sehr simpel vor. Dies ist für später häufiger notwendig.

\
 Außerdem habe ich mehrere Probleme mit Docker gehabt und mich deshalb dazu entschieden, mal etwas anderes zu probieren.

== Graphische Oberflächen
Ich dachte mir zu Beginn, dass ein reiner Development-Container keine GUIs braucht, da es ja genug Terminal Text-Editioren gibt. Nach jedoch etwas Überlegung sollte klar sein, dass ein GUI zur Kontrolle des Roboters fast unverzichtbar ist (beispielsweise für die Kamera).

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


== Synchronisierte Ordner
Das Folgende ist nötig, da sonst die Konfigurationen und Änderungen verloren gehen würden. Im Grunde mounten wir nur Konfig Dateien und den Workspace, um keinen Datenverlust zu haben und damit unsere Daten nicht davon abhängig sind, ob der Container noch existiert oder nicht.

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
== Gesamtergebnis

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
Das -it am Anfang steht für Interactive Terminal. Dadurch kriegen wir eine SSH-artige Shell, die /bin/bash (Das man Ende Steht), also ganz normales Bash, öffnet. --workdir bestimmt, in welchem Ornder unser Terminal gestartet wird. Dies ist aus Praxisgründen unser Workspace. --rm sorgt dafür, dass der Container gelöscht wird, wenn er beendet wird. Dies ist kein Problem da wir alle wichtigen Daten an unser reelles System gemountet haben.

#pagebreak()

= Kamera publisher
Damit der Fahrer des Roboters auch etwas sieht, muss das Kamerabild irgendwie übertragen werden. Dies machen wir über den ROS2 image transport. Das Kamerabild fangen wir mit OpenCV ab.

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
Die Main Funktion initialisiert zuerst ROS2 und erstellt dann eine ROS2 Node mit dem MinimalImagePublisher. Dann wird der ROS Cyclus gestartet. Es wird die Timer_Callback-Funktion in einem Loop ausgeführt, bis Ctrl-C gedrückt wird. Der Delay zwischen den Loops ist in timer\_ definiert.
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
Im Konstruktor wird die publisher node definiert. Diese sendet eine Imagesensor-Message. Der timer bestimmt den delay zwischen den loops.

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
Zu Beginn wird eine Matrix erstellt, die Image heißt. Diese ist einfach nur eine Rohdarstellung der Bilddaten. Dann wird ein VideoCapture namens cap erstellt. Die `0` bei `cap(0)` bestimmt die ausgewählte Kamera. Tendenziell könnte man da eine andere Nummer eingeben und es würde eine andere Kamera ausgewählt werden, falls es eine auf dieser Position gibt. Die erfolgreiche Erstellung des Video captures wird daraufhin in Z. 4ff. Überprüft.

In Z. 8 setzen wir Image auf das Bild, das cap momentan besitzt, also das, das die Kamera momentan aufnimmt. Daraufhin wird eine Image-Message erstellt, die versendet werden kann. Dazu müssen wir von dem OpenCV Matrix Format erst einmal in das ROS kompatible Imageformat konvertieren. Dann publishen wir das Image und senden es in das Netzwerk. Dann geben wir noch aus, wie viele Frames bereits geschickt wurden und erhöhen den Counter für die bereits geschickten Frames.

Die letzten vier Zeilen deklarieren den Typ der öffentlich verfügbaren Variablen, die oben im Konstruktor gesetzt werden.
#pagebreak()
