# ResQTank Project
![ResQTank Logo](images/logo.jpg)
## Description
ResQTank is an advanced Lidar-based emergency response system designed for rapid and efficient disaster response in indoor environments. This project utilizes cutting-edge technology to detect people and navigate through complex environments effectively.

## Key Features
- **Real-Time 3D Mapping**: Leverages RTAB-Map for real-time 3D mapping.
![ResQTank Logo](images/logo.jpg)
- **AI People Detection**: Utilizes advanced AI algorithms for accurate detection and tracking.
- **Smart Guidance System**: Path assistance for safe navigation in light and dark environments.
- **Responsive User Interface**: Feature-rich control panel GUI for easy monitoring and control.


## Installation

### Prerequisites
Before you begin, ensure you have the following installed:
- Python 3.x
- NVIDIA JetPack (for Jetson modules)
- Arduino IDE (for Arduino code)


### Setting Up Python Environment
1. Clone the repository:
   ```
   git clone https://github.com/odxxt/resqtank.git
   ```
2. Navigate to the project directory:
   ```
   cd resqtank
   ```
3. Install Python dependencies:
   ```
   pip install -r requirements.txt
   ```

### Additional Dependencies
Some components used in this project are not available through pip and need to be set up separately:


#### NVIDIA Jetson Inference
- The Jetson Inference library is not part of the NVIDIA JetPack SDK. Instead, it can be installed from the `dusty-nv/jetson-inference` GitHub repository.
- To install and configure Jetson Inference, follow the guide for building the project from source:
  [Building Jetson Inference from Source](https://github.com/dusty-nv/jetson-inference/blob/master/docs/building-repo-2.md#building-the-project-from-source).
- Ensure you follow the steps specifically tailored to your Jetson model. This method is recommended as we did not use any container for this project.


#### Intel RealSense SDK
- The Intel RealSense SDK can be installed on NVIDIA Jetson devices, including the Jetson Orin Nano, following the instructions outlined in the `installation_jetson.md`. This guide provides comprehensive steps for setting up on various Jetson models.
- For our project, we specifically installed the SDK by building it from source using the RSUSB Backend. This method is particularly well-suited for environments like the Jetson Orin Nano, ensuring optimal compatibility and performance.
- The installation guide can be found here: [Intel RealSense SDK Installation on NVIDIA Jetson Devices](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md#nvidia-jetson-devices).


#### RTAB-Map for Intel RealSense L515
- RTAB-Map installation details can be found on the [official RTAB-Map GitHub page](https://github.com/introlab/rtabmap).
- Ensure to configure it specifically for Intel RealSense L515 as described in their setup. 
- For our project, we have included a custom configuration file `config.ini` located in the `rtabmap_config` folder. This configuration has been optimized for the best performance in controlled indoor environments using the Intel RealSense L515. We recommend using this configuration to replicate our results.

## Arduino Setup
The Arduino is programmed to interface with the [Cytron 20Amp 6V-30V DC Motor Driver](https://www.cytron.io/p-20amp-6v-30v-dc-motor-driver), controlling the movement of the motors based on serial commands. The specific Arduino code can be found in the repository.

##### Motor Driver Configuration
The provided Arduino script is configured for the Cytron 20Amp 6V-30V DC Motor Driver. This setup controls various movements through serial commands as detailed in the table below. If you are using a different motor driver, the code can be easily adapted by modifying the motor control commands. The structure of the code is straightforward and simple, allowing for easy modifications.

##### Control Commands
Here is a table of serial commands and their corresponding movements, along with the keyboard bindings used in our control interface:

| Char | Keyboard Bind    | Command      |
|------|------------------|--------------|
| 'f'  | W or arrow-up    | Forward      |
| 'b'  | S or arrow-down  | Backward     |
| 'r'  | D or arrow-right | Right        |
| 'l'  | A or arrow-left  | Left         |
| 'e'  | E                | Spin Right   |
| 'q'  | Q                | Spin Left    |

To modify or customize the Arduino code for different motor drivers, adjust the motor control functions within the script to match the input specifications and control methods of the new driver. This customization allows the ResQTank to be adaptable to a variety of hardware configurations.

## Usage
Explain how to run the project after installation, any command line arguments they might need, and how to navigate the GUI.

## Contributing
We welcome contributions! If you would like to help make ResQTank better, please feel free to fork the repository and submit a pull request.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Authors
- Abdulaziz AlOdat
- Adnan Kazi
- Alexandre Meulien
- Amer Ammar

## Acknowledgments
- Thanks to all the contributors who have invested their time into making ResQTank a reality.
- Special thanks to NVIDIA and Intel for their powerful hardware and software solutions.
