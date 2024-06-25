
### Data grabber for Unitree z1 manipulator


#### Installation no-ros sdk

1. Make a new directory and clone controller and sdk.

    ```bash
    mkdir ~/grab_data_itmo && cd ~/grab_data_itmo
    git clone git@github.com:unitreerobotics/z1_controller.git
    git clone git@github.com:composable-learning-manipulation/z1_sdk.git
    ```

2. Build controller.

    ```bash
    cd ~/grab_data_itmo/z1_controller
    mkdir build && cd build
    cmake .. 
    make
    ```

3. Build sdk.

    ```bash
    cd ~/grab_data_itmo/z1_sdk
    mkdir build && cd build
    cmake ..
    make
    ```

    If eigen don't found by pybind
    - install eigen `sudo apt-get install libeigen3-dev` 
    - use in building process `cmake ../ -D CMAKE_CXX_FLAGS="-I /usr/include/eigen3"`

4. Setup IP address in `~/grab_data_itmo/z1_controller/config/config.xml`.


#### Start grab data

1. Start controller.

    ```bash
    cd ~/grab_data_itmo/z1_controller/build && ./z1_ctrl
    ```

2. Start data grabber_1 and wait.

    ```bash
    cd ~/grab_data_itmo/z1_sdk/ && ./build/data_grabber_pos_itmo
    ```

3. Start data grabber_2 and wait.

    ```bash
    cd ~/grab_data_itmo/z1_sdk/ && ./build/data_grabber_torque_itmo
    ```

4. Archive data.

    ```bash
    cd ~/grab_data_itmo/z1_sdk/
    tar -cf data_z1_for_itmo.tar ./*.txt
    ```

### Experiments description

ТТХ робота:
tau_max = 33
tau_min = 0.2
dq_max = M_PI


1. trapezoid
- q \pm M_PI/2
- dq_max = M_PI/2
- ddq_max = M_PI 

2. sin
- A = 0.2, w = 0.2
- A = 0.1, w = 2.0

3. torque sin
- A = 0.5, w = 2.0
- A = 1.0, w = 6.0
