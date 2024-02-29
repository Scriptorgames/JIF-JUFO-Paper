#import "@preview/polylux:0.3.1": *

#set page(paper: "presentation-16-9")
#set text(size: 25pt)

#import themes.simple: *
#set text(font: "Inria Sans")
#show: simple-theme.with(
  footer: [Roboter + ControllerGUI],
)

#title-slide[
     = JIF

     == Roboter + ControllerGUI
]
#slide[
    = 1. Roboter
    == 1.1 Fahrgestell und Karrosserie
    - Fahrgestell und Kettenantrieb von _HowToMechatronics_
    - Selbst designte Karrosserie und Adapterstücke
    - Verbau von stärkeren Federn (ca. $10 N$)
    #grid(
      columns:(75%, 25%), rows:(auto),
      [- Fahrgestell wurde aus ABS und PETG gedruckt,\ die Ketten aus PETG und TPU,\ und die Karrosserie aus PETG und PLA],
      image("pictures/JifFusion.png", width: 100%)
    )
]
#slide[
  == 1.2 Hardware
  - 2x 37mm 12V DC Motoren($66 r p m$)
  - Ansteuerung über 2 L298N H-Brücken
  - Ansteuerung dieser über ein ESP32-Developnment-Board
  - Strom: 12V 5.000 mAh Lipo-Akku
  - Abwärtswandler von 12V auf 5V
  #pagebreak()
  #image("./pictures/SchaltplanUpdate.png", height: 100%)
]
#slide[
  == 1.3 MicroROS
  + neuer Workspace für das Flashen des ESPs und den MicroROS agent
  + Klonen der Microros-App in den Firmware Ordner
  + Konfigurieren & Builden der App
  + Konfigurieren & Builden des Microros-Agents
  + Flashen
  + Starten des Micoros Agents
  + Nun sichtbar unter ros_topics cmd_velocity
]
#slide[
  = 2. Containerisierung des ROS2 Environments
  == 2.1 Podman
  - Kein Daemon -> keine Administratorrechte nötig
  - Keine Nutzerrollen nötig
  - Pods zum Gruppieren von Containern und Tests
  - Einfaches speichern des Containerstands
  == 2.2 GUI Übertragung
  - Env Variablen setzen, um Audio und Video zu übertragen
  - Einhängen der Geräte in den Container
  - Einhängen der Systemrelevanten Ordner z.B. Fonts
]
#slide[
  == 2.3 Letzte Schritte
  - Ordner die persistent sein sollten auf der echten Maschine erstellen und einhängen
  - Container aufsetzen und Abhängigkeiten installieren
  - Containerzustand Speichern
]
#slide[
  = 3. Kamera Publisher
  == 3.1 Grundstruktur
  #text(size: 12pt)[
 ```cpp
 // Ein Haufen imports
 using namespace std::chrono_literals;
 class MinimalImagePublisher : public rclcpp::Node {
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
    ```
  ]
]
#slide[
  == 3.2 Die wichtige Funktion
  #text(size: 12pt)[
  ```cpp
 void timer_callback() {
 cv::Mat image;
 cv::VideoCapture cap(0);
 if (!cap.isOpened()){
 std::cout << "cannot open camera";
 return;
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
]
