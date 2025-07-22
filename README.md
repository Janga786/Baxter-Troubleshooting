# Baxter Troubleshooting Tips

Troubleshooting tips for keeping Baxter robots operational in 2025 and beyond.  

This guide consolidates common fixes for boot issues, networking problems, ROS/Intera SDK errors, and hardware troubleshooting.

---

## 📖 Documentation
Official docs are no longer maintained, but you can access archived versions here:  
[Archived SDK Wiki](https://web.archive.org/web/20220312130557/https://sdk.rethinkrobotics.com/wiki/Home)

---

## ⚡ Startup & Boot Issues
**1. Baxter not powering on / unresponsive screen**  
- Power cycle at least 3 times, waiting 30 sec each time.  
- If it still doesn’t work, go into FSD (Field Service Menu):  
  **Press `Alt + F`** → reboot from there.

**2. BIOS password lockout**  
- Use [BIOS Password Recovery](https://bios-pw.org/)  
- Enter system number shown when opening BIOS.  
- Generated password is admin → confirm with `Ctrl+Enter`.

**3. Real-time clock showing wrong date (e.g., 2016)**  
- Sync Baxter’s time with your computer.  
- Set in Baxter FSM or use NTP from your computer via command line.

---

## 🌐 Networking & Communication
**4. IP mismatch between Baxter and workstation**  
- Set Baxter to Manual IP in FSM.

**5. Static IP configuration on Linux (example: 192.168.42.1)**  
- First 3 numbers must match between workstation and Baxter.  
- Ensure Baxter knows your IP in `intera.sh`.

**6. Ping test: can't reach baxter.local**  
- Make sure Baxter’s hostname is set correctly in FSM.  
- Disable firewall on your computer.  
- Try pinging Baxter’s static IP.

**7. ROS Master URI not resolving**  
```bash
export ROS_MASTER_URI=http://baxter.local:11311
```

**8. SSH into Baxter fails**  
- Verify SSH installed, firewall off, IP correct.

---

## 🛠 ROS & Intera SDK Issues
**9. Wrong catkin workspace sourcing**  
```bash
source ~/intera_ws/devel/setup.bash
```

**10. enable_robot.py or joint_trajectory_action_server.py missing**  
- Run `catkin_make` or `catkin_build` after troubleshooting.

**11. intera.sh script error**  
- Ensure file is in root of catkin workspace:  
  `~/intera_ws/intera.sh`

**12. MoveIt integration not working**  
- Ensure robot is enabled and joint trajectory server is active in a second terminal.

---

## 🤖 Hardware & Motion Problems
**13. Arms not enabled or unresponsive**  
```bash
rosrun intera_interface enable_robot.py -e
```
- Test by gripping cuffs (zero-g mode should enable).

**14. Joint calibration errors**  
- Restart robot. Happens if you hit `CTRL+Z` mid-script.

---

## 🔍 Testing, Debugging, & Logging
**15. Check robot state:**  
```bash
rostopic echo /robot/state
```

**16. Helpful debug commands:**  
```bash
rostopic list
rosnode list
rosservice list
```

**17. Reading logs:**  
- Robot: `~/.ros/log/latest/`  
- Workstation: `/var/log/roslaunch.log`

**18. Confirm joint angles:**  
```bash
rostopic echo /robot/joint_states
```

---

## ✅ Contribute
If you have more tips or fixes, open a Pull Request or share an Issue!

---
