# Inbetriebnahme

Die vorliegende Anleitung zeigt die Inbetriebnahme der Hardware für den Turtlebot.

Ausführliche Details findet man unter: 
https://github.com/tsprifl/catkin_src

# Lager
Der Turtlebot und die Unterlagen (neben Ladegerät etc.) lagen im Labor in den Kisten <todo>
# Schritte

1. Powerverbindung mit XTPower-Akku herstellen.
2. Ggf. Ladegerät "XTPower TurtleBot Sick" anschließen.
3. Gerät startet
4. Gerät mit Monitor (HDMI an der Frontseite) und USB über HUB mit Tastatur und Maus verbinden.
5. Bei Sick wurde das Gerät hinter einem Proxy betrieben. Der ROS-Master lief auf einem Remote-PC.
   Möchte man bei SICK eine Demo starten, muss man den Proxy gemäß Proxy Config einstellen.
6. ROS-Master: Bei einer lokalen Entwicklung auf dem System ROS_MASTER und ROS_HOSTNAME gem. u.a. config eingestellt werden.
   Für eine Remote-Verbindung muss die IP-Adresse etc. für Remote-Rechner mit laufendem Ros-Core bekannt sein.
   (siehe u.a. Punkte)
7. Nun der o.a. Anleitung folgen (ACHTUNG: In der Anleitungen befinden sich Fehler. Siehe "Start des Bots". Statt
   "rosrun iam" muss man "roslaunch iam Robot_FSM.launch" angeben (s.u.).
   In Kurzform:
   * Terminator: 4 Terminalfenstser starten
   * Terminal 1: roslaunch turtlebot3_bringup turtlebot3_robot.launch
   * Terminal 2: ols
   * Terminal 3: rosrun gpio_handling gpio_handler
   * Terminal 4: rosparam load ~/catkin_ws/src/iam/yaml/AGC.yam
   * Terminal 4 (erneut): roslaunch iam Robot_FSM.launch
   
8. Spurführungsband verlegen bzw. Robot auf Demo-System setzen.
9. terminator starten

## Proxy Config
* Bei Lehning : keine
* Bei Sick : 
  * Host/Port: cloudproxy-sickag.sickcn.net:10415 
  * Details siehe:
    * Webproxy: https://wiki.ubuntuusers.de/Proxyserver/#Unity-und-GNOME-3
    * apt proxy: https://askubuntu.com/questions/257290/configure-proxy-for-apt
    
## ros config remote/local master
IP Config Bash RC
In der Datei ~/.bashrc muss die IP-Adresse des Ros-Master (Remote-PC), sowie die IP-Adresse des Turtlebots eingetragen werden. (Ändern der letzten beiden Zeilen).
für lokalen master 127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1

## Start des Bots

Für jedes Terminal wie gewohnt:
```console
cd catkin_ws
source devel/setup.bash
```

Bei Punkt 6:
statt
```console
rosrun iam
```
```console
roslaunch iam Robot_FSM.launch
```

siehe Anleitung unter 
https://github.com/tsprifl/catkin_src


