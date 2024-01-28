#import "template.typ": *
#show: ams-article.with(title: [JIF - Roboter & Controller GUI], authors: (
  (name: "Jonas Nicklas"),
  (name: "Ilian Odenbach"),
  (name: "Felix Schreiber"),
), abstract: {}, bibliography-file: "refs.bib")

= Roboter
== Fahrgestell und Karosserie 
Das Fahrgestell ist nicht von uns designt worden, eben so wenig wie der
Kettenantrieb. Dies war auch nicht notwendig, da Dejan von HowToMechatronics
eine wunderbare Basis für unser Vorhaben bereits geschaffen hatte.
@howtomechatronics2024 Auf dieser Basis wurde eine Karosserie für unsere Zwecke
in Fusion360 erstellt mit zwei aufklappbaren Deckeln, die die Wartung an dem
Roboter erleichtern. Um die elektronischen Komponenten auf dem Fahrgestell zu
befestigen, wurden Adapterstücke konstruiert, da die Löcher des Fahrgestells für
andere Komponenten designt wurden. Ebenso wurden stärkere Federn in den Dämpfern
eingebaut die 10 Newton in etwa maximal Belastung haben. Der Schlitz auf der
Oberseite des Hinterendeckels ist für die Kabelführung der Webcam.
#image("pictures/JifFusion.png")

#pagebreak()
== Hardware
Der Roboter wird von zwei 37mm 12V DC-Motoren mit 66rpm betrieben, welche
angesteuert werden über zwei L298N H-Brücken, wobei jeder Motor mit einem der
Motoren verkabelt ist. Die Steuerung der H-Brücken hat ein ESP32 development
board, welches seriell mit einem Raspberry Pi 3B+ verbunden ist, inne. Die
Stromversorgung übernimmt ein 12V 5000mAh LiPo-Akku, der die Motortreiber direkt
versorgt, parallel dazu geschaltet ist ein Abwärtswandler, welcher die 12V auf
5V konvertiert und so den Raspberry Pi mit Strom versorgt. Man stellt sich
sicherlich die Frage warum zwei H-Brücken verbaut wurden, wenn man an einem 2
Motoren anschließen kann, aber eine H-Brücke kann maximal 2 Ampere abgeben, was
sich dann auch auf beide Motoren aufteilen würde was zu einem Leistungsverlust
führt, da jedem Motor nur 1 Ampere zugewiesen werden kann. Um diese Problematik
zu lösen wurden zwei H-Brücken verbaut und so können jedem Motor 2 Ampere
zugewiesen werden. Leider ist aber immer noch ein Spannungsabfall zu bemerken.
So wird ein zukünftiger Schritt sein ein Motortreiber zu verbauen, welcher eine
noch höhere Stromstärke stemmen kann. 
#image("pictures/SchaltplanUpdate.png")
== micro-ROS
Um den ESP32 vom ROS2-Netzwerk aus anzusteuern, wurde micro-ROS verwendet. Dazu
wurde zuerst ein neues Workspace erstellt, welches für das Flashen und dem
micro-ROS-Agent zuständig sein wird. Anschließend noch ein src Ordner in dem
micro-ROS Workspace erstellen, wo dann das micro_ros_setup Package hinein
geklont wurde. @microros2024 Nachdem ersten Build ist nun ein firmware Ordner zu
sehen, in dem Verzeichnis ,".../microros_ws/firmware/freertos_apps/apps", wurde
dann die ros_esp32cam_diffdrive App geklont. @reinbert2024 Nach dem
Konfigurieren und dem Builden konnte der ESP32 auch schon geflasht werden. Nun
fehlte nur noch der micro-ROS-Agent. Nach dessen Builden und Starten konnte man
unter den Topics auch unser /cmd_vel finden. Somit war der erste Teil der Basis
geschaffen.
#pagebreak()



= Containerisierung des ROS2 environments für Entwicklung auf einem nicht-Ubuntu Gerät

Zum Aufsetzen eines simplen Containers für ROS2 mussten wir uns zuerst mehrere
essenzielle Fragen stellen: Welche Ordner sollten dauerhaft bestehen bleiben?
Wollen wir grafische Oberflächen unterstützen? Welche Software wollen wir
nutzen? Wie speichern wir unseren Containerstand zwischen?

== Podman
Podman ist unser Container Manager. Jetzt fragt man sich, warum nicht Docker?
Hier eine kurze Übersicht, warum ich mich für Podman entschieden habe.

- Kein Daemon, daher keine verpflichtenden Admin-Rechte
  - Daher keine Nutzerrollen nötig, um einen Container ohne sudo rechte zu starten


- Sogenannte 'Pods', die es ermöglichen mehrere Container zu gruppieren und zu
  Verwalten
  - Daher ist es auch möglich, in diesem Fall eine Robotersimulation und einen
    Client-Container in ein privates Netzwerk zu packen und eine ungehinderte
    Simulation bzw. Testphase ablaufen zu lassen.

- Mir kommt das Containerstand speichern sehr simpel vor. Dies ist für später
  häufiger notwendig.

\
Außerdem habe ich mehrere Probleme mit Docker gehabt und mich deshalb dazu
entschieden, mal etwas anderes zu probieren.

== Graphische Oberflächen
Ich dachte mir zu Beginn, dass ein reiner Development-Container keine GUIs
braucht, da es ja genug Terminal Text-Editioren gibt. Nach jedoch etwas
Überlegung sollte klar sein, dass ein GUI zur Kontrolle des Roboters fast
unverzichtbar ist (beispielsweise für die Kamera).

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
Das Folgende ist nötig, da sonst die Konfigurationen und Änderungen verloren
gehen würden. Im Grunde mounten wir nur Konfig Dateien und den Workspace, um
keinen Datenverlust zu haben und damit unsere Daten nicht davon abhängig sind,
ob der Container noch existiert oder nicht.

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
  ps1: [`$`],
  input: [podman export {ContainerName} -o {Dateiname}.tar],
  output: [Dieser Befehl speichert den momentanen Containerzustand in einer tar Datei],
)
Der Containerzustand ist jetzt in einer Datei gespeichert, jedoch ist es so
nicht so simpel einen neuen Container mit `podman run` zu starten. Daher müssen
wir nun den Containerzustand als eigene Vorlage für einen neuen Container
speichern.
#term(
  ps1: [`$`],
  input: [podman import {DateiName(Unsere tar)} {template-name}],
  output: [Dieser Befehl sorgt dafür, dass der Container der Datei als template genommen
  werden kann und daher Dinge wie `podman run {template-name}` möglich sind],
)

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
Das -it am Anfang steht für Interactive Terminal. Dadurch kriegen wir eine
SSH-artige Shell, die /bin/bash (Das man Ende Steht), also ganz normales Bash,
öffnet. --workdir bestimmt, in welchem Ornder unser Terminal gestartet wird.
Dies ist aus Praxisgründen unser Workspace. --rm sorgt dafür, dass der Container
gelöscht wird, wenn er beendet wird. Dies ist kein Problem da wir alle wichtigen
Daten an unser reelles System gemountet haben.

#pagebreak()

= Kamera publisher
Damit der Fahrer des Roboters auch etwas sieht, muss das Kamerabild irgendwie
übertragen werden. Dies machen wir über den ROS2 image transport. Das Kamerabild
fangen wir mit OpenCV ab.

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
Die Main Funktion initialisiert zuerst ROS2 und erstellt dann eine ROS2 Node mit
dem MinimalImagePublisher. Dann wird der ROS Cyclus gestartet. Es wird die
Timer_Callback-Funktion in einem Loop ausgeführt, bis Ctrl-C gedrückt wird. Der
Delay zwischen den Loops ist in timer\_ definiert.
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
Im Konstruktor wird die publisher node definiert. Diese sendet eine
Imagesensor-Message. Der timer bestimmt den delay zwischen den loops.

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
Zu Beginn wird eine Matrix erstellt, die Image heißt. Diese ist einfach nur eine
Rohdarstellung der Bilddaten. Dann wird ein VideoCapture namens cap erstellt.
Die `0` bei `cap(0)` bestimmt die ausgewählte Kamera. Tendenziell könnte man da
eine andere Nummer eingeben und es würde eine andere Kamera ausgewählt werden,
falls es eine auf dieser Position gibt. Die erfolgreiche Erstellung des Video
captures wird daraufhin in Z. 4ff. Überprüft.

In Z. 8 setzen wir Image auf das Bild, das cap momentan besitzt, also das, das
die Kamera momentan aufnimmt. Daraufhin wird eine Image-Message erstellt, die
versendet werden kann. Dazu müssen wir von dem OpenCV Matrix Format erst einmal
in das ROS kompatible Imageformat konvertieren. Dann publishen wir das Image und
senden es in das Netzwerk. Dann geben wir noch aus, wie viele Frames bereits
geschickt wurden und erhöhen den Counter für die bereits geschickten Frames.

Die letzten vier Zeilen deklarieren den Typ der öffentlich verfügbaren
Variablen, die oben im Konstruktor gesetzt werden.
#pagebreak()


= Das GUI

Der letzte größere Teil des Projekts ist das GUI, um den Roboter steuern zu
können. Um das ganze auch für spätere Projekte nutzbar machen zu können ist ein
sehr universelles Design notwendig, das für verschiedenste Anwendungszwecke
geeignet ist. Das Design und die Grundidee sind simpel und gleichzeitig
erstaunlich effizient: Warum ein unveränderbares UI festlegen, was für wenige
Aufgaben ausgelegt ist? Warum sich als Entwickler die Mühe machen, etwas zu
designen, was viele Nutzer vielleicht nur begrenzt oder teilweise verwenden
können? Warum nicht einfach... den Anwender seine Benutzeroberfläche selbst
gestalten lassen? Genau das war die Idee hinter dem Projekt: ein universelles,
komplett veränderbares ROS2 UI, was für so ziemlich alle noch kommenden
Roboter-Projekte, an denen wir oder andere arbeiten, eine Möglichkeit bietet,
schnell zu testen oder einfach drauf los zu fahren! Doch wie soll die Software
nun aussehen? Wie schon gesagt, sehr einfach: ein Fenster, oben eine
Menü-Leiste, ansonsten: leer.

#image("pictures/gui1.png")

== Implementations-Überblick

Die Software selbst ist in C++ programmiert und basiert auf vier grundlegenden "Säulen":
OpenGL @opengl bzw. GLEW @glew, GLFW @glfw, ImGui @imgui und ROS2 @ros2
(RCLCpp), Version "Humble". Hierbei ist GLFW für die Fensterverwaltung und
Context-Bereitstellung zuständig, OpenGL übernimmt das Graphic-Backend, ImGui
generiert das GUI, und ROS2 ist, einfach gesagt, ROS2. Um das Arbeiten mit den
unterschiedlichen Libraries einfacher zu gestalten habe ich einige
Wrapper-Klassen und Funktionen geschrieben, die die Funktionalität der Libraries
etwas abstrahieren. Dazu zählt zum Beispiel die Window-Klasse, die die
Funktionen von GLFW (ursprünglich für C gemacht) nimmt und in eine Klasse
verpackt, mit deren Hilfe man relativ einfach (dafür umsomehr eingeschränkt) ein
Fenster erstellen und verwalten kann.

Ein zusätzliches Tool, dass ich selbst geschrieben habe ist eine Art
Resourcen-Management, das die verschiedenen Resourcen der App, befindlich im "res"
Ordner, installiert im Hauptverzeichnis der App, verwaltet, einliest und so
weiter. Alle Resourcen-Dateien sind im JSON Format verfasst. Dieses System ist
dazu gemacht, die Weiterentwicklung des Programms zu vereinfachen: so sind zum
Beispiel die Menü-Leiste und das Standart Layout nicht fest einprogrammiert,
sondern können schnell, ohne das das Projekt lange neu gebuildet werden muss
angepasst werden:

#sc[```JSON
{
    "type": "viewlayout",
    "id": "default",
    "name": "Default Layout",
    "views": [
        {
            "type": "view",
            "id": "0",
            "name": "Demo",
            "viewtype": "demo"
        },
        {
            "type": "view",
            "id": "1",
            "name": "Help",
            "viewtype": "help"
        }
    ]
}
```] _layout-default.json_

Der oben gezeigte Codeblock enthält die _layout-default.json_. Diese Datei
definiert das Standart Layout, dass beim Start der App geladen wird. Zunächst
definieren wir hier den Typ der Datei ("viewlayout"), dann eine ID ("default")
und einen Anzeigenamen ("Default Layout"). Dannach wird ein Array mit Views
festgelegt: Ein Demo-View mit der ID "0" und dem Namen "Demo", und ein
Help-View, ID "1" und Name "Help". Die in "viewtype" eingetragenen Werte sind
die IDs der jeweiligen Typen von Views, diese werden in anderen Resourcen
definiert. Wenn man das Programm startet dann sieht das Ergebnis ungefähr so
aus:

#image("pictures/gui2.png")

Warum nur ungefähr? Nun ja, die JSON-Datei gibt ja nicht an, wo sich die Views
befinden und wie groß sie sind. Dafür ist die so genannte _imgui.ini_ zuständig:

#sc[```INI
[Window][DockSpaceViewport_11111111]
Pos=0,19
Size=800,581
Collapsed=0

[Window][Debug##Default]
Pos=60,60
Size=400,400
Collapsed=0

[Window][Demo##view@0]
Pos=60,60
Size=100,71
Collapsed=0

[Window][Help##view@1]
Pos=276,131
Size=268,133
Collapsed=0

[Docking][Data]
DockSpace ID=0x8B93E3BD Window=0xA787BDB4 Pos=0,19 Size=800,581 CentralNode=1
```]

Diese Datei wird von ImGui generiert und gespeichert. Das heißt: wenn man die
Views verschiebt/ihre Größe verändert, dann werden die Änderungen in der _imgui.ini_ gespiegelt.

Das Resourcen-Management erlaubt außerdem auch das definieren eigener
View-Types, also was für Views der Nutzer erstellen kann, neuen Menüs in der
Menü-Leiste, verschiedene Standartlayouts die geladen werden könenn und so
weiter. Später kommen auch noch Resourcen-JSONs für Bilder, Schriftarten,
Textdateien und noch einiges mehr dazu, um die Entwicklung noch weiter zu
beschleunigen und zu vereinfachen.

Um die Arbeit an dieser Stelle nicht noch weiter unnötig in die Länge ziehen zu
müssen wird hier kein Source-Code eingefügt, da der Code an sich zu komplex,
viel zu groß und noch mehr viel zu durcheinander ist, um ihn hier sinnvoll und
verständlich darstellen zu können. Das gesamte Software-Projekt ist jedoch auf #link("https://github.com/Scriptor25/jif")[*_GitHub_*] zu
finden. Das Projekt ist folgendermaßen strukturiert (nur die wichtigsten Ordner
und Dateien):

#sc[```
├─include // Header Dateien
│ ├─imgui    // ImGui
│ ├─jif      // JIF
│ ├─kubazip  // Zip-Library
│ ├─nlohmann // JSON-Library
│ └─stb      // stb_image Library
├─res     // Resourcen
│ ├─drawable
│ ├─font
│ ├─layout
│ └─viewtype
├─src     // Source Code
│ ├─imgui
│ ├─jif
│ └─kubazip
├─.gitignore
├─CMakeLists.txt
├─LICENSE.txt
└─package.xml
```]

und läuft unter der "BEERWARE" Lizenz:

#sc[```
"THE BEERWARE LICENSE" (Revision 42):
Felix Schreiber wrote this code. As long as you retain this 
notice, you can do whatever you want with this stuff. If we
meet someday, and you think this stuff is worth it, you can
buy me a beer in return.
```]

== Funktionsweise

Doch was kann man mit der Software bis jetzt als Nutzer alles machen? Die
Palette an Funktionen wächst zwar ständig weiter, aber zum Zeitpunkt dieser
Arbeit sind ein paar grundlegende Dinge möglich:

- Das Laden/Speichern von Layouts und
- Das Hinzufügen/Entfernen/Verwalten von Views

So kann man zum Beispiel schon eine kleine App zusammenbasteln:

#image("pictures/gui3.png")

Um ein View zum Layout hinzuzufügen geht man auf "View => Add". Dann öffnet sich
ein Fenster, dass einen durch den Prozess begleitet: Zunächst gibt man dem neuen
View einen Namen, zum Beispiel "Test". Dann wählt man im nächsten Dialog aus
einem Dropdown-Menü den View-Type aus, in diesem Beispiel "image". Unter dem
Dropdown tauchen dann weitere Felder auf, die man je nach Typ ausfüllt, bei "image"
zum Beispiel den ROS-Topic, aus dem das Bild gezogen werden soll.

Um ein View zu entfernen/wieder sichtbar zu machen (man kann Views minimieren,
wenn man auf das X klickt), geht man auf "View => Manager". Darauf öffnet sich
ein View-Manager Fenster, in dem man die einzelnen Views (de-)aktivieren oder
löschen kann.

#image("pictures/gui4.png")

Beim Speichern eines Layouts wird die _imgui.ini_ und _layout.json_ in eine _.jif_ Datei
gepackt, die eigentlich nur eine ganz normale _.zip_ Datei ist, aber der Name
klingt cooler... Jedenfalls kann diese mit anderen ausgetauscht, geteilt,
verschoben, etc. werden, wenn man sie über "Layout => Load" wieder lädt, wird
das Layout und die ImGui Konfiguration wiederhergestellt.

