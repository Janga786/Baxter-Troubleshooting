# 🤖 Baxter Station - Windows Ready

A complete Docker-based development environment for Baxter robot programming, fully compatible with Windows.

## 🚀 Quick Start

### Prerequisites
- **Windows 10/11** with Docker Desktop
- **Docker Desktop** with Linux containers enabled
- **Git** for cloning the repository
- **Network access** to your Baxter robot (default IP: 192.168.42.2)

### Installation

1. **Clone the repository:**
   ```cmd
   git clone https://github.com/Janga786/Baxter_Station.git
   cd Baxter_Station
   ```

2. **Run setup:**
   ```cmd
   setup.bat
   ```

3. **Test your robot:**
   ```cmd
   test_baxter.bat
   ```

## 🏗️ Architecture

This Docker environment includes three containers:

1. **baxter_ros1** - ROS Indigo environment with complete Baxter SDK
2. **moveit_ros2** - ROS2 Humble with MoveIt2 motion planning
3. **ros_bridge** - ROS1↔ROS2 communication bridge

## 📁 Directory Structure

```
Baxter_Station_Windows/
├── setup.bat               # Main setup script
├── test_baxter.bat          # Robot connection test
├── start_moveit.bat         # Launch MoveIt interface
├── docker-compose.yml       # Container orchestration
├── baxter_ros1/
│   └── Dockerfile          # ROS1 Baxter environment
├── moveit_ros2/
│   └── Dockerfile          # ROS2 MoveIt environment
├── bridge/
│   └── Dockerfile          # ROS1-ROS2 bridge
├── shared_ws/              # Shared workspace
│   └── simple_baxter_control.py
└── documentation/          # Guides and references
```

## 🎮 Usage

### Initial Setup
```cmd
setup.bat
```
This will:
- Check Docker installation
- Build all containers (15-20 minutes first time)
- Start all services
- Display status and quick commands

### Test Robot Connection
```cmd
test_baxter.bat
```
This will:
- Enable the robot
- Run a simple movement test
- Verify everything is working

### Start MoveIt Planning
```cmd
start_moveit.bat
```
This opens an interactive shell in the MoveIt container for motion planning.

### Manual Commands

#### Enable Robot
```cmd
docker exec -it baxter_ros1_container bash -c "source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && rosrun baxter_tools enable_robot.py -e"
```

#### Disable Robot
```cmd
docker exec -it baxter_ros1_container bash -c "source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && rosrun baxter_tools enable_robot.py -d"
```

#### Interactive Shell Access
```cmd
# ROS1 Baxter environment
docker exec -it baxter_ros1_container bash

# ROS2 MoveIt environment  
docker exec -it moveit_ros2_container bash
```

#### Run Custom Python Scripts
```cmd
docker exec -it baxter_ros1_container bash -c "source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /shared_ws/simple_baxter_control.py"
```

## 🔧 Configuration

### Baxter IP Address
Edit these files if your Baxter has a different IP:
- `docker-compose.yml` (ROS_MASTER_URI values)
- Default IP: `192.168.42.2`

### Network Configuration
If you can't connect to Baxter:
1. Verify Baxter's IP: `ping 192.168.42.2`
2. Check Windows Firewall settings
3. Ensure you're on the same network as Baxter
4. Verify Docker Desktop is using the correct network adapter

## 🛠️ Development

### Adding Custom Scripts
1. Place Python scripts in `shared_ws/` 
2. They'll be accessible from all containers at `/shared_ws/`
3. Source the appropriate ROS environment before running

### Example Custom Script
```python
#!/usr/bin/env python
import rospy
import baxter_interface

def my_baxter_program():
    rospy.init_node('my_program')
    rs = baxter_interface.RobotEnable()
    rs.enable()
    
    left_arm = baxter_interface.Limb('left')
    # Your code here
    
if __name__ == '__main__':
    my_baxter_program()
```

### Rebuilding Containers
```cmd
docker-compose down
docker-compose build --no-cache
docker-compose up -d
```

## 📚 Available Baxter Examples

Once inside the baxter_ros1_container:
```bash
# Joint control examples
rosrun baxter_examples joint_velocity_wobbler.py
rosrun baxter_examples joint_position_keyboard.py

# Gripper control
rosrun baxter_examples gripper_keyboard.py

# Head control
rosrun baxter_examples head_wobbler.py

# Pick and place demo
rosrun baxter_examples pick_and_place.py
```

## 🐛 Troubleshooting

### Docker Issues
- **Container won't start**: Check Docker Desktop is running with Linux containers
- **Build fails**: Ensure you have enough disk space (need ~10GB)
- **Permission errors**: Run Command Prompt as Administrator

### Robot Connection Issues
- **Can't connect to Baxter**: 
  - Verify IP with `ping 192.168.42.2`
  - Check network connectivity
  - Ensure Baxter is powered on and enabled
- **Robot won't enable**: Check E-stop is not pressed

### Performance Issues
- **Slow builds**: This is normal on first run (15-20 minutes)
- **High CPU usage**: Expected during container builds
- **Memory usage**: Ensure you have at least 8GB RAM available

## 🔍 Monitoring

### View Container Status
```cmd
docker-compose ps
```

### View Container Logs
```cmd
# All containers
docker-compose logs

# Specific container
docker-compose logs baxter_ros1
docker-compose logs moveit_ros2
docker-compose logs ros_bridge
```

### Check Resource Usage
```cmd
docker stats
```

## 🛑 Stopping Everything

```cmd
docker-compose down
```

This stops and removes all containers but preserves the images for faster restart.

## 📞 Support

### Common Solutions:
1. **Robot won't move**: Check if robot is enabled and calibrated
2. **Container communication issues**: Verify network_mode: host in docker-compose.yml
3. **Windows path issues**: Use Command Prompt, not PowerShell or Git Bash
4. **Firewall blocking**: Temporarily disable Windows Firewall for testing

### Debug Commands:
```cmd
# Check Docker installation
docker --version
docker-compose --version

# Test network connectivity
ping 192.168.42.2

# Check container health
docker exec -it baxter_ros1_container rosnode list
```

## 🏷️ Version Information

- **ROS1**: Indigo (compatible with Baxter)
- **ROS2**: Humble with MoveIt2
- **Docker**: Compose v3.3 format
- **OS**: Windows 10/11 with Docker Desktop
- **Robot**: Baxter Research Robot

---

**Note**: This Windows-ready version removes X11 forwarding for GUI applications. For visualization, use web-based interfaces or VNC if needed.