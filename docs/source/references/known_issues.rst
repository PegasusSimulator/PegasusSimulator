Known Issues
============

[Fatal] [gpu.foundation.plugin] Out of resource descriptors
-----------------------------------------------------------

If you get the error message ``[Fatal] [gpu.foundation.plugin] Out of resource descriptors`` when running Pegasus Simulator, it likely that you are getting above the default limit on the number of file descriptors allowed per process Ubuntu 22.04's, which is 1024 by default. When using multiple GPUs, Pegasus Simulator on Isaac Sim 4.5.0 uses closer to 1382, leading to Isaac's Vulkan API back end to crash. 
Raising that system wide limit may solve this issue. You can do this by following these steps:

1. Open a terminal and run the command: 

    .. code:: bash
        
        sudo nano /etc/systemd/user.conf

        find, uncomment, and set DefaultLimitNOFILE=65535

2. Again, modify the following system file:
   
    .. code:: bash
        
        add the following two lines just above the "# End of file" at the bottom of the file:
    
        * hard nofile 65535
        * soft nofile 65535

After this you will need to reboot your machine, but after that, Pegasus Simulator should be working fine on Isaac Sim 4.5.0. 

**This guide was provided by** `Austin-Parks <https://github.com/Austin-Parks>`__ **, who encountered this issue while running Pegasus Simulator on Ubuntu 22.04 with Isaac Sim 4.5.0. Check the original issue on** `Github Issues <https://github.com/PegasusSimulator/PegasusSimulator/issues/86>`__.