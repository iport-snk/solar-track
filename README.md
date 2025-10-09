# Solar Tracker Project

A C++ application for solar panel tracking with MQTT communication capabilities.

## Prerequisites

This project requires several system dependencies to be installed before building.

### System Requirements

- **Operating System**: Linux (tested on Raspberry Pi OS/Debian)
- **Compiler**: GCC with C++20 support
- **Build System**: CMake 3.10 or higher

### Required Dependencies

#### 1. Development Tools
```bash
sudo apt update
sudo apt install build-essential cmake git
```

#### 2. MQTT Client Library
Install the Eclipse Paho MQTT C client library:
```bash
sudo apt install libpaho-mqtt1.3 libpaho-mqtt-dev
```

This provides:
- `libpaho-mqtt3c` - Asynchronous MQTT client
- `libpaho-mqtt3cs` - Synchronous MQTT client (used by this project)
- `libpaho-mqtt3a` - Asynchronous MQTT client with automatic reconnection
- `libpaho-mqtt3as` - Synchronous MQTT client with automatic reconnection

#### 3. OpenSSL Development Headers
Required for secure MQTT connections and CMake's OpenSSL integration:
```bash
sudo apt install libssl-dev
```

#### 4. Arduino CLI
Required for compiling and uploading the Arduino relay controller code:
```bash
# Download and install arduino-cli
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Move to system path (optional, or add to PATH)
sudo mv bin/arduino-cli /usr/local/bin/

# Initialize configuration
arduino-cli config init

# Update core index
arduino-cli core update-index

# Install Arduino megaAVR core (for Arduino Nano Every)
arduino-cli core install arduino:megaavr
```

#### 5. Arduino Device Binding (Optional but Recommended)
To ensure your Arduino always appears as `/dev/ttyACM0` across reboots:

1. **Connect your Arduino** and identify its hardware details:
   ```bash
   lsusb | grep -i arduino
   udevadm info -a -n /dev/ttyACM0 | grep -E "(idVendor|idProduct|serial)" | head -3
   ```

2. **Create udev rule** (replace the serial number with your Arduino's):
   ```bash
   sudo tee /etc/udev/rules.d/99-arduino-nano-every.rules << 'EOF'
   # Arduino Nano Every - bind to /dev/ttyACM0
   SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0058", ATTRS{serial}=="YOUR_SERIAL_HERE", SYMLINK+="ttyACM0", MODE="0666", GROUP="dialout"
   EOF
   ```

3. **Activate the rule**:
   ```bash
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

This ensures the Arduino is always accessible at `/dev/ttyACM0` regardless of boot order or USB port changes.

#### 6. Time Synchronization Setup
Critical for accurate solar position calculations:

1. **Verify timezone is correct** (replace with your location):
   ```bash
   timedatectl status
   # If timezone is wrong, set it:
   sudo timedatectl set-timezone Europe/Kyiv  # Example for Ukraine
   ```

2. **Ensure NTP synchronization is enabled**:
   ```bash
   sudo timedatectl set-ntp true
   systemctl status systemd-timesyncd
   ```

3. **Verify time synchronization**:
   ```bash
   timedatectl status | grep "System clock synchronized"
   # Should show: System clock synchronized: yes
   ```

**Important**: Solar tracking requires precise time for calculating sun position. Ensure your system maintains accurate time via NTP.

### Dependency Information

- **libpaho-mqtt3cs**: Provides synchronous MQTT client functionality with SSL/TLS support
- **OpenSSL**: Used for secure connections (dynamically linked with libpaho-mqtt3cs)
- **Version compatibility**: 
  - Paho MQTT C: 1.3.12-1
  - OpenSSL: 3.0.17 or compatible

## Building the Project

### Main C++ Application

1. **Clone the repository** (if not already done):
   ```bash
   git clone <repository-url>
   cd solar-track
   ```

2. **Create build directory**:
   ```bash
   mkdir build
   cd build
   ```

3. **Configure with CMake**:
   ```bash
   cmake ..
   ```

4. **Build the project**:
   ```bash
   make
   ```

5. **Run the executable**:
   ```bash
   ./main
   ```

### Arduino Relay Controller

The project includes an Arduino-based relay controller for motor control:

1. **Navigate to Arduino directory**:
   ```bash
   cd ardu-relays
   ```

2. **Connect your Arduino Nano Every** to `/dev/ttyACM0` (or update `PORT` in Makefile)

3. **Compile and upload**:
   ```bash
   make all
   ```
   
   Or separately:
   ```bash
   make compile  # Compile only
   make upload   # Upload to Arduino
   ```

#### Arduino Hardware Requirements
- **Board**: Arduino Nano Every (or compatible megaAVR board)
- **Connection**: USB connection to `/dev/ttyACM0`
- **Permissions**: User must be in `dialout` group for serial access

## Project Structure

```
solar-track/
├── CMakeLists.txt          # CMake build configuration
├── README.md               # This file
├── ardu-relays/           # Arduino relay control code
│   ├── ardu-relays.ino
│   └── Makefile
├── include/               # Header files
│   ├── Config.hpp
│   ├── IntervalRunner.hpp
│   ├── Motors.hpp
│   ├── Mqtt.hpp
│   ├── SensorController.h
│   ├── SerialWorker.h
│   ├── spa.h
│   ├── State.hpp
│   └── Sun.hpp
├── libs/                  # Third-party libraries
│   └── json.hpp
├── src/                   # Source files
│   ├── main.cpp
│   ├── SensorController.cpp
│   ├── SerialWorker.cpp
│   └── spa.c
├── keys/                  # SSL certificates
│   ├── client.csr
│   └── client.key
└── build/                 # Build output directory
```

## Features

- **Solar Position Calculation**: Uses SPA (Solar Position Algorithm) for accurate sun tracking
- **MQTT Communication**: Secure MQTT client for remote monitoring and control
- **Motor Control**: Interface for controlling solar panel positioning motors
- **Sensor Integration**: Temperature and environmental sensor support
- **Serial Communication**: Arduino integration for relay control
- **SSL/TLS Security**: Encrypted MQTT connections

## Configuration

The project uses header-based configuration. Edit the files in the `include/` directory to customize:

- `Config.hpp` - General project configuration
- `Mqtt.hpp` - MQTT broker settings and credentials
- `Motors.hpp` - Motor control parameters
- `State.hpp` - System state management

## Troubleshooting

### Common Issues

1. **CMake can't find OpenSSL**:
   ```bash
   sudo apt install libssl-dev
   ```

2. **Paho MQTT library not found**:
   ```bash
   sudo apt install libpaho-mqtt1.3 libpaho-mqtt-dev
   ```

3. **Compiler errors about C++20**:
   Ensure you have a recent version of GCC:
   ```bash
   gcc --version  # Should be 8.0 or higher for C++20
   ```

4. **Permission issues with serial devices**:
   Add your user to the dialout group:
   ```bash
   sudo usermod -a -G dialout $USER
   ```
   Then log out and back in.

5. **Arduino compilation errors**:
   Ensure you have the correct core installed:
   ```bash
   arduino-cli core install arduino:megaavr
   ```

6. **Arduino upload fails**:
   Check the correct port:
   ```bash
   arduino-cli board list
   # Update PORT in ardu-relays/Makefile if different
   ```

7. **Arduino not consistently at /dev/ttyACM0**:
   Set up the udev rule as described in the Arduino Device Binding section above.

8. **Permission denied accessing Arduino**:
   Ensure you're in the dialout group:
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in for changes to take effect
   ```

9. **Incorrect solar calculations**:
   Verify time and timezone are correct:
   ```bash
   timedatectl status
   date && date -u  # Check local and UTC time
   ```
   
   If timezone is wrong:
   ```bash
   sudo timedatectl set-timezone Your/Timezone
   ```

### Build Clean

To clean the build directory:
```bash
cd build
make clean
# Or remove and recreate build directory
cd ..
rm -rf build
mkdir build
cd build
cmake ..
make
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test the build process
5. Submit a pull request

## License

[Add your license information here]

## Hardware Requirements

- Raspberry Pi or compatible Linux SBC
- Solar panel positioning motors
- Environmental sensors (temperature, light, etc.)
- Arduino (for relay control)
- MQTT broker (local or cloud-based)

## Network Configuration

Ensure your system can connect to:
- MQTT broker (configure in `Mqtt.hpp`)
- Internet (for NTP time synchronization)
- Local Arduino devices (via serial/USB)

## Cheat sheet
- arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200