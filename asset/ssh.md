# Instruction for SSH  
All robots are connected to the local Wi-Fi network **ECE346** at startup. Each will have a reserved IP address ```192.168.0.1XX```, where ```XX``` is the ID of the robot. For example, if the Jetson on your robot has the label *NX-7*, the IP address is ```192.168.0.107```. 

Before running SSH, first connect your computer to the **ECE346** Wi-Fi with password *ece346sp2022*. <span style="color:red">Due to bandwidth constraints, do not use any stream services on **ECE346** Wi-Fi</span>.


## Connect from Windows PC
If you are using your windows laptop or the lab workstation, you can connect to the SSH through PowerShell.
### SSH with x11 forwarding on Windows 
TODO

## Connect from Linux/Mac PC
TODO
### SSH with x11 forwarding on Linux/Mac
TODO


## Code Remotely with [VS Code](https://code.visualstudio.com/)
If you want to code directly on your laptop but have the code saved in Jetson directly, one solution is to SSH into the Jetson and use vim or nano through the terminal. On the other hand, if you enjoy using modern IDEs,  [VS Code allows you to code remotely through SSH](https://code.visualstudio.com/docs/remote/ssh-tutorial).