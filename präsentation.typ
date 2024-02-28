#import "@preview/polylux:0.3.1": *
#import "term.typ": term
#import "@preview/showybox:2.0.1": showybox
#import "@preview/codelst:2.0.0": sourcecode

#let sc = sourcecode.with(
  numbers-style: (lno) => text(size: 10pt, font: "Times New Roman", fill: rgb(0, 0, 0), str(lno)),
  frame: block.with(
    stroke: 1pt + rgb("#a2aabc"),
    radius: 2pt,
    inset: (x: 10pt, y: 5pt),
    fill: rgb("DBDBDB"),
  ),
)


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
