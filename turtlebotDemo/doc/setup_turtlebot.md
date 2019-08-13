Hostname des Turtlebots: turtlebot
Password: turtlebot3


Setup:

Zunächst muss sich der Turtlebot im gleichen Netz wie der Remote-PC befinden. Im Auslieferungszustand wird sich der EC700-BT Rechner in ein WLAN mit der SSID "Ubuntu-Hotspot" mit dynamischer IP einloggen, da dies meine Konfiguration war.
Die Empfehlung ist, bei der ersten Inbetriebnahme Maus und Tastatur am hinteren Ende des AGC, sowie HDMI-Kabel vorne neben dem OLS20 in den Rechner einzustecken. Dann kann manuell ein WLAN-Netzwerk gespeichert werden, in welches sich der Rechner nach dem booten einloggt (vorzugsweise mit fester IP-Adresse).

In der Datei ~/.bashrc muss die IP-Adresse des Ros-Master (Remote-PC), sowie die IP-Adresse des Turtlebots eingetragen werden. (Ändern der letzten beiden Zeilen).

Nachdem sichergestellt ist, dass sich der Turtlebot-PC beim Neustart in das gewünschte Netz einloggt, kann Maus, Tastatur und HDMI-Kabel abgezogen werden und der Turtlebot-PC neu gestartet werden.


Damit im folgenden Abschnitt nicht bei jeder ssh-Verbindung das Password eingegeben werden muss, empfielt es sich, am Remote-PC ein rsa-key zu generieren und diesen auf dem Turtlebot-PC zu speichern. Dazu auf dem Remote-PC folgende Befehle ausführen:
	$ ssh-keygen -t rsa #Den Key ohne Password durch zweimaliges drücken von Enter speichern.
	$ ssh-copy-id -i ~/.ssh/id_rsa.pub turtlebot@Turtlebot-IP-Adresse


Starten der Robot-Fsm:

1. Turtlebot an Powerbank einstecken und ca. 1 min warten, bis sich der Turtlebot-PC in das konfigurierte Netzwerk eingeloggt hat.

2. roscore vom Remote-PC aus starten.

3. Konsolenfenster öffnen und eine ssh-Verbindung zum Turtlebot-PC erstellen. Anschließend zum OpenCR-Board verbinden. Dies geschieht Normalerweise durch den Befehl "roslaunch turtlebot3_bringup turtlebot3_robot.launch". Der Befehl ist jedoch durch den Kurzbefehl "rl" hinterlegt. Somit folgende Befehle ausführen:
	$ ssh turtlebot@IP-Adresse #Danach besteht das Konsolenfenster auf dem Turtlebot-PC
	$ rl # Am Anfang wird ein Fehler auftauchen, da der Standard-Laserscanner nicht verbunden ist. Der 			Fehler kann ignoriert werden.

4. Konsolenfenster öffnen und eine ssh-Verbindung zum Turtlebot-PC erstellen. Anschließend den ols-Treiber starten. Dies geschieht Normalerweise durch den Kurzbefehl "ols". Dieser konfiguriert die can0-Schnittstelle und führt das sick_line_guidance launch-file mit dem ols-yaml-file aus. Somit folgende Befehle ausführen:
	$ ssh turtlebot@IP-Adresse #Danach besteht das Konsolenfenster auf dem Turtlebot-PC
	$ ols

5. Die gpio-handler Node vom Remote-PC aus starten:
	$ rosrun gpio_handling gpio_handler

### ACHTUNG: Beim ausführen der nächsten Befehle wird sich der Turtlebot geradeaus bewegen. ###
6. Das yaml-Parameter-File für die robot-fsm-Node laden und anschließend die Node starten:
	$ rosparam load ~/catkin_ws/src/iam/yaml/AGC.yaml
	$ rosrun iam

7. Durch starten der Teleop-Node kann der Turtlebot gestoppt werden.
	(Achtung hier publishen nun zwei Nodes auf dem gleichen Topic (cmd_vel). Deshalb muss die robot_fsm Node 		kurz nach dem starten der teleop-Node mit strg+c beendet werden):
	$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch # Durch drücken der Taste s auf der Tastatur, 			kann der Turtlebot sofort gestoppt werden.

Der Turtlebot fährt nun geradeaus. Wird eine Linie erkannt erfolgt ein Wechsel in den State follow_line. Der Turtlebot folgt der Linie. Geht die Linie verloren, wird der Turtlebot nach 1 Sekunde stoppen.
