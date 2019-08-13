# Interne Sammlung von FAQ

## Zugriff auf Turtlebot über WLAN-Lehning

Der Zugriff erfolgt über SSH per WLAN. Der Turtlebot holt sich via DHCP (eigentlich) immer dieselbe IP-Adresse.
Deswegen wurde vorerst auf Static-IP verzichtet.
Die IP-Adresse lautet: 192.168.178.45

Mit dem Zugriff über

```bash
ssh -X turtlebot@192.168.178.45
```

kann die Remote-Verbindung zum Roboter aufgebaut werden.


## PC:

    PC-Typ: Ist es der Typ „EC700-BT4051-454-64WT“, den Sie in Ihrer Master-Arbeit erwähnen? -> Ja
    Ist ein Account/Ubuntu eingerichtet? -> Ja
        Annahme: Ubuntu: 18.04 Betriebssystem mit der ROS-Distribution ROS-Melodic -> Richtig
        Wenn „Ja“: Username und Passwort? -> Username: turtlebot, Passwort: turtlebot3
    Kann man den PC mit einem der beigefügten Netzteile betreiben? -> Könnte man theoretisch. Ich würde Ihnen empfehlen den Stecker (roter Pfeil) einzustecken. (Aufgrund der Länge passt er nur in die auf dem Bild rechte Buchse).  Dazu können Sie das Netzteil für die Powerbank in die linke Buchse einstecken. Dann haben Sie einen "quasi Netzbetrieb".

## TiM:

    Eingestellte IP-Adresse? -> Am TiM habe ich nichts konfiguriert. Daher müsste noch die Standard-IP-Adresse konfiguriert sein.

## Spg.-Versorgung:

    Batterie wird mit dem Standardnetzteil aufgeladen? Blauer Pfeil? -> Richtig
    Spg.-Versorgung zum PC erfolgt einfach durch Einstecken des Hohlsteckers in die Batterie? (roter Pfeil) -> Korrekt
     

## Miniatur-Lichtschranken:

    Ich nehme an, dass die Miniatur-Lichtschranken von Typ "GL2S- E1311" sind (vgl. Masterarbeit) -> korrekt
    Sind die Lichtschranken an das OpenCR-Board wie in der Masterarbeit angeschlossen? -> Auf dem "Version 0" Dokument, das Sie haben finden Sie auf Seite 16 die Abbildung 5.1.  Die Sensoren sind wie dort abgebildet am OpenCR-Board angeschlossen. Die Einweglichtschranken (GRSE18S) sind auf Ihrem Turtlebot jedoch nicht vorhanden. Zwischen dem Schaltausgang der Miniatur-Lichtschranken GL2S-E1311 und dem GPIO-Pin befindet sich noch ein Serienwiderstand von 57k, um die Spannung an den GPIO-Pins auf 3,3V anzupassen (Thesis S. 20 Abb. 5.6).

               
## Allgemein:

    Sollte ich beim Laden, Betreiben etc. auf irgendetwas besonders achten? -> Ich habe in dem Git-Repo, in dem ich Sie als Collaborator hinzugefügt habe eine Readme.md erstellt. Mit dieser Datei könnte es Ihnen evtl. etwas leichter fallen, das System -- so wie ich es immer in Betrieb genommen habe -- in Betrieb zu nehmen.
    
## Herunterfahren

```bash
sudo shutdown -h now
# Alternativ:
sudo init 0
```

## Turtlebot: USB stick mounten

```bash
# list all devices:
sudo fdisk -l
# usbstick-device finden , z.B. /dev/sdb1
sudo mkdir /media/usbstick
# usb stick mounten, /dev/sdb1 ggfs ersetzen
sudo mount -t vfat /dev/sdb1 /media/usbstick -o uid=1000,gid=1000,utf8,dmask=027,fmask=137
# usb stick gemounted unter /media/usbstick:
ls /media/usbstick
```

## Turtlebot: Filetransfer mit scp

```bash
# ros logfiles mit scp vom Turtlebot in ein lokales Verzeichnis auf dem Linux-Rechner kopieren:
rostest@ROS-NB: 
mkdir -p ~/tmp
cd ~/tmp
scp -r turtlebot@192.168.178.45:/home/turtlebot/.ros/log scp -r turtlebot@192.168.178.45:/home/turtlebot/catkin_ws/src/sick_line_guidance/turtlebotDemo/test/scripts/log .
# scp -r turtlebot@192.168.178.45:/home/turtlebot/.ros/log .
# scp -r turtlebot@192.168.178.45:/home/turtlebot/catkin_ws/src/sick_line_guidance_demo/test/scripts/log .
```
