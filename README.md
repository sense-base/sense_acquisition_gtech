<div style="text-align: center;" align="center">
  <img src="docs/figs/sense_aquisition_gtec.svg" alt="" width="800"/>
  <h1> sense_aquisition_gtec </h1>
</div>

# Sense Acquistion Gtech
## :eyeglasses: Overview

`sense_aquisition_gtec` is a ROS 2 (Humble) package that provides a C++ implementation of an EEG data acquisition node for a g.tec amplifier. The package publishes g.tec EEG data to ROS 2 topics using the custom EEGBlock message defined in eeg_msgs.

## How to Use

Clone the `sense-base/base` and create the directory `workspace/src`. Clone `sense_aquisition_gtec` package into the `workspace/src`:


```bash
mkdir sense-base && cd sense-base
git clone git@github.com:sense-base/base.git
cd base
mkdir workspace/src
# Clone this sense_aquisition_gtec repo
git clone git@github.com:sense-base/sense_aquisition_gtec.git
# Clone the sense_msgs package
git clone git@github.com:sense-base/sense_msgs.git
```

In order to build this node you need a personal access token for the PRIVATE_gUSBamp-Linux-Driver-C-API repository.
You also need to be running docker as root in order to access the usb port. I tried setting plugdev permissions for a standard user, but that did not work for me.

Open the sense-base/base folder in VS Code and Reopen in Container when prompted. In the `.devcontainer`, build the workspace:

```bash
export GTEC_TOKEN=XXXXXXXXXXXXX
colcon build --packages-select eeg_msgs gtec_eeg_publisher
source install/setup.bash
ros2 launch gtec_eeg_publisher gtec_eeg_publisher_launch.py
```

## Requesting access to the g.tec API
To run or develop this code you will need access to the gtec API (header file and library). g.tec have given us permission to distribute these files but have asked that
we keep track of who is using them and for what. We are currently managing access to these files using a private GitHub repository and personal access tokens. If you would 
like a personal access token please [create a new issue](https://github.com/sense-base/sense_acquisition_gtech/issues) using the `Request access for gtec API` template.

## CI/CD workflow
The [ci.yml](.github/workflows/ci.yml) workflow installs libraries, builds ROS 2 packages, and runs tests for:
* Humble Hawksbill (May 2022 – May 2027) on Ubuntu Jammy (22.04)
* Jazzy Jalisco (May 2024 – May 2029) on Ubuntu Noble (24.04)
* Kilted Kaiju (May 2025 – November 2026) on Ubuntu Noble (24.04)

You can view the CI/CD logs on [github-actions](https://github.com/sense-base/sense_acquisition_gtech/actions).
If you'd like to test a new or previous version, simply edit ci.yml and update the list above in line with our [GitHub workflow contribution guidelines](https://github.com/sense-base/base/blob/main/CONTRIBUTING.md).

Note: You might want to run [`ament_uncrustify --reformat <path_to_source_folders or file>`](https://docs.ros.org/en/rolling/p/ament_cmake_uncrustify/doc/index.html) to sort out any code style inconsistencies.