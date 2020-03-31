1. I clicked on the top-left arrow and the client stopped.

**Look at the Error cluster on the right corner look if there is any error.**

2. I get this pop-up when I ran the simulator

<div class="img_container">
    <img class='sm_img' src="https://github.com/monoDriveIO/documentation/raw/links_and_giffs/docs/LV_client/quick_start_img/faq.png"/>
</div>

**The simulator is not running. Go to your VehicleAI directory and double click on the VehicleAI.exe, then try again.**

3. I got error 63. 

“TCP Open Connection in monoDrive.lvlib:mono_connect.vi->monoDrive.lvlib:mono_init.vi->monoDrive_closed_loop_example.vi”

**The client couldn’t connect to the server, make sure the simulator is running and the client is connected to port 8999**

4. I got error 56.

This means the client hit a timeout, this happens if the port for any sensor is duplicated. Make sure every sensor has a unique port number in its configuration.


The simulator is not running. Go to your VehicleAI directory and double click on the VehicleAI.exe, then try again.
