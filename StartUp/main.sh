cd /home/jt/桌面/basler_camera/StartUp

gnome-terminal --geometry=80x25+10+10 -t "roscore" -- bash -c "./roscore.sh;exec bash;" 
# gnome-terminal --geometry=80x25+10+30 -t "cloud" -- bash -c "./cloud.sh;exec bash;" 
gnome-terminal --geometry=80x25+10+50 -t "QT_TCP" -- bash -c "./QT.sh;exec bash;" 

gnome-terminal --geometry=80x25+10+90 -t "basler_ros" -- bash -c "./basler_ros.sh;exec bash;" 
