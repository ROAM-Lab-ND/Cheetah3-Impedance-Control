BUILDING
========


Linux
-----

   * `mkdir build`
   * `cd build`
   * `cmake ..`
   * `make`

Environment Varaible Setting

   * `export LCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=1`


This would set the environment variable for the current shell. To permanantly set the environment variable for the current (not root) user, add the above commands to ~/.bashrc

Run
-----
   * `cd build/test/linux/simple_test`
   * `sudo -E ./simple_test em1`

The option -E has to be used since it allows the environment variables to be passed to the sudo environment. This is becauase sudo will create a [new root environment](https://www.petefreitag.com/item/877.cfm), and the environment variables defined for the current user are not visible to the sudo environment.
