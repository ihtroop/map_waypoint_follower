import sys
import subprocess
from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_new_ws(object):
    def setupUi(self, new_ws):
        new_ws.setObjectName("new_ws")
        new_ws.resize(835, 532)
        self.centralwidget = QtWidgets.QWidget(new_ws)
        self.centralwidget.setObjectName("centralwidget")
        
        # Buttons and labels setup
        self.Gazebo_Button = QtWidgets.QPushButton(self.centralwidget)
        self.Gazebo_Button.setGeometry(QtCore.QRect(50, 210, 141, 43))
        self.Gazebo_Button.setObjectName("Gazebo_Button")
        
        self.Nav2_Button = QtWidgets.QPushButton(self.centralwidget)
        self.Nav2_Button.setGeometry(QtCore.QRect(250, 210, 141, 43))
        self.Nav2_Button.setObjectName("Nav2_Button")
        
        self.Slam_Button = QtWidgets.QPushButton(self.centralwidget)
        self.Slam_Button.setGeometry(QtCore.QRect(450, 210, 141, 43))
        self.Slam_Button.setObjectName("Slam_Button")
        
        self.Rviz_Button = QtWidgets.QPushButton(self.centralwidget)
        self.Rviz_Button.setGeometry(QtCore.QRect(650, 210, 141, 43))
        self.Rviz_Button.setObjectName("Rviz_Button")
        
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(300, 150, 251, 51))
        font = QtGui.QFont()
        font.setFamily("Monospace")
        font.setPointSize(22)
        self.label.setFont(font)
        self.label.setObjectName("label")
        
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(340, 270, 161, 41))
        font = QtGui.QFont()
        font.setFamily("Monospace")
        font.setPointSize(22)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        
        self.user_ip_button = QtWidgets.QPushButton(self.centralwidget)
        self.user_ip_button.setGeometry(QtCore.QRect(80, 340, 122, 43))
        self.user_ip_button.setObjectName("user_ip_button")
        
        self.intermediate_goals_button = QtWidgets.QPushButton(self.centralwidget)
        self.intermediate_goals_button.setGeometry(QtCore.QRect(220, 340, 122, 43))
        self.intermediate_goals_button.setObjectName("intermediate_goals_button")
        
        self.pause_nav2_button = QtWidgets.QPushButton(self.centralwidget)
        self.pause_nav2_button.setGeometry(QtCore.QRect(360, 340, 122, 43))
        self.pause_nav2_button.setObjectName("pause_nav2_button")
        
        self.aruco_button = QtWidgets.QPushButton(self.centralwidget)
        self.aruco_button.setGeometry(QtCore.QRect(500, 340, 122, 43))
        self.aruco_button.setObjectName("aruco_button")
        
        self.final_goals_button = QtWidgets.QPushButton(self.centralwidget)
        self.final_goals_button.setGeometry(QtCore.QRect(80, 390, 122, 43))
        self.final_goals_button.setObjectName("final_goals_button")
        
        self.pushButton_6 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_6.setGeometry(QtCore.QRect(640, 390, 122, 43))
        self.pushButton_6.setObjectName("pushButton_6")
        
        self.pid_button = QtWidgets.QPushButton(self.centralwidget)
        self.pid_button.setGeometry(QtCore.QRect(640, 340, 122, 43))
        self.pid_button.setObjectName("pid_button")
        
        self.String_pub_button = QtWidgets.QPushButton(self.centralwidget)
        self.String_pub_button.setGeometry(QtCore.QRect(220, 390, 122, 43))
        self.String_pub_button.setObjectName("String_pub_button")
        
        self.pushButton_9 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_9.setGeometry(QtCore.QRect(360, 390, 122, 43))
        self.pushButton_9.setObjectName("pushButton_9")
        
        self.pushButton_10 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_10.setGeometry(QtCore.QRect(500, 390, 122, 43))
        self.pushButton_10.setObjectName("pushButton_10")
        
        self.Build_Button = QtWidgets.QPushButton(self.centralwidget)
        self.Build_Button.setGeometry(QtCore.QRect(50, 80, 741, 61))
        self.Build_Button.setObjectName("Build_Button")
        
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(360, 20, 131, 41))
        font = QtGui.QFont()
        font.setFamily("Monospace")
        font.setPointSize(26)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        
        new_ws.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(new_ws)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 835, 32))
        self.menubar.setObjectName("menubar")
        new_ws.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(new_ws)
        self.statusbar.setObjectName("statusbar")
        new_ws.setStatusBar(self.statusbar)

        self.retranslateUi(new_ws)
        QtCore.QMetaObject.connectSlotsByName(new_ws)

        # Connect buttons to their respective functions
        self.Build_Button.clicked.connect(self.build_button_clicked)
        self.Gazebo_Button.clicked.connect(self.gazebo_button_clicked)
        self.Nav2_Button.clicked.connect(self.nav2_button_clicked)
        self.Slam_Button.clicked.connect(self.slam_button_clicked)
        self.Rviz_Button.clicked.connect(self.rviz_button_clicked)
        self.user_ip_button.clicked.connect(self.user_ip_button_clicked)
        self.intermediate_goals_button.clicked.connect(self.intermediate_goals_button_clicked)
        self.pause_nav2_button.clicked.connect(self.pause_nav2_button_clicked)
        self.aruco_button.clicked.connect(self.aruco_button_clicked)
        self.final_goals_button.clicked.connect(self.final_goals_button_clicked)
        self.pid_button.clicked.connect(self.pid_button_clicked)
        self.String_pub_button.clicked.connect(self.string_pub_button_clicked)
        self.pushButton_6.clicked.connect(self.kill_all)

        # Dictionary to keep track of running processes
        self.running_processes = {}

    def retranslateUi(self, new_ws):
        _translate = QtCore.QCoreApplication.translate
        new_ws.setWindowTitle(_translate("new_ws", "MainWindow"))
        self.Gazebo_Button.setText(_translate("new_ws", "Gazebo"))
        self.Nav2_Button.setText(_translate("new_ws", "Nav2 Stack"))
        self.Slam_Button.setText(_translate("new_ws", "Slam ToolBox"))
        self.Rviz_Button.setText(_translate("new_ws", "Rviz 2"))
        self.label.setText(_translate("new_ws", "Run Simulation"))
        self.label_2.setText(_translate("new_ws", "Run codes"))
        self.user_ip_button.setText(_translate("new_ws", "user_ip"))
        self.intermediate_goals_button.setText(_translate("new_ws", "interm. goal"))
        self.pause_nav2_button.setText(_translate("new_ws", "pause Nav2"))
        self.aruco_button.setText(_translate("new_ws", "aruco"))
        self.final_goals_button.setText(_translate("new_ws", "final goals"))
        self.pushButton_6.setText(_translate("new_ws", "Kill All"))
        self.pid_button.setText(_translate("new_ws", "pid"))
        self.String_pub_button.setText(_translate("new_ws", "String pub"))
        self.pushButton_9.setText(_translate("new_ws", ""))
        self.pushButton_10.setText(_translate("new_ws", ""))
        self.Build_Button.setText(_translate("new_ws", "Colcon Build"))
        self.label_3.setText(_translate("new_ws", "new_ws"))

    def run_command_in_terminal(self, command):
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', command])

    def run_command_with_check(self, command):
        # Check if the command is already running
        if command in self.running_processes:
            # Terminate the existing process
            self.running_processes[command].terminate()
            self.running_processes[command].wait()
        
        # Run the command
        process = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', command], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.running_processes[command] = process

    def build_button_clicked(self):
        self.run_command_in_terminal('cd ~/new_ws && colcon build')
    
    def kill_all(self):
        subprocess.Popen(['pkill', 'gnome-terminal'])

    def gazebo_button_clicked(self):
        self.run_command_in_terminal('ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py')

    def nav2_button_clicked(self):
        self.run_command_in_terminal('ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True')

    def slam_button_clicked(self):
        self.run_command_in_terminal('ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True')

    def rviz_button_clicked(self):
        self.run_command_in_terminal('ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz')

    def user_ip_button_clicked(self):
        command = 'python3 ~/new_ws/src/control/control/user_ip.py'
        self.run_command_with_check(command)

    def intermediate_goals_button_clicked(self):
        command = 'python3 ~/new_ws/src/control/control/intermediate_goals.py'
        self.run_command_with_check(command)

    def pause_nav2_button_clicked(self):
        command = 'python3 ~/new_ws/src/control/control/pause_nav2.py'
        self.run_command_with_check(command)

    def aruco_button_clicked(self):
        command = 'python3 ~/new_ws/src/control/control/aruco.py'
        self.run_command_with_check(command)

    def final_goals_button_clicked(self):
        command = 'python3 ~/new_ws/src/control/control/final_goal_node.py'
        self.run_command_with_check(command)

    def pid_button_clicked(self):
        command = 'python3 ~/new_ws/src/control/control/pid.py'
        self.run_command_with_check(command)

    def string_pub_button_clicked(self):
        command = 'python3 ~/new_ws/src/control/tests/string_pub.py'
        self.run_command_with_check(command)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    new_ws = QtWidgets.QMainWindow()
    ui = Ui_new_ws()
    ui.setupUi(new_ws)
    new_ws.show()
    sys.exit(app.exec_())
