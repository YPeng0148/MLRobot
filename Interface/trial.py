import subprocess
def direction(destination):
    #new_terminal_cmd = 'osascript -e \'tell application "Terminal" to do script ""\'' #Macs terminal
    #new_terminal_cmd = 'wt.exe new-tab' #for windows terminal
    login_cmd = 'sshpass -p temppwd ssh debian@192.168.8.1'
   # directory = 'cd robot_delivery_system && source devel_isolated/setup.bash &&  xport ROS_IP=192.168.8.1 && roscore & roslaunch executive beaglebone.launch destination:=' + destination 
    directory = 'bash -c "cd robot_delivery_system && source devel_isolated/setup.bash && export ROS_IP=192.168.8.1 && roslaunch executive beaglebone.launch destination:=' + destination + '"'
    #run = "roslaunch navigation navigation.launch"
    #subprocess.Popen(new_terminal_cmd, shell=True)
    subprocess.Popen(login_cmd + ' \'' + directory + '\'', shell=True)
    #subprocess.call(login_cmd + ' ' + directory + ' && ' + python_cmd, shell=True)
