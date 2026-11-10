# Baxter Troubleshooting Tips

Troubleshooting tips for keeping Baxter robots operational in 2025 and beyond.  

This guide collects fixes for boot issues, networking problems, ROS/SDK errors, and hardware problems.

I wrote it in summer 2025 while reviving two Rethink robots (Baxter and Sawyer) during my research assistantship with Dr. Kevin Wedeward at Fort Lewis College. The Docker workstation I used with them is in [Baxter-Sawyer_Work_Station_Windows](https://github.com/Janga786/Baxter-Sawyer_Work_Station_Windows).

Note: commands that use `intera.sh`, `intera_ws` or `intera_interface` are for Sawyer's Intera SDK. On Baxter the equivalents are `baxter.sh` and `baxter_tools` (for example `rosrun baxter_tools enable_robot.py -e`).

---

## Documentation
Official docs are no longer maintained, but you can access archived versions here:  
[Archived SDK Wiki](https://web.archive.org/web/20220312130557/https://sdk.rethinkrobotics.com/wiki/Home)

---

## Startup & Boot Issues
**1. Baxter not powering on / unresponsive screen**  
- Power cycle at least 3 times, waiting 30 sec each time.  
- If it still doesn’t work, go into the FSM (Field Service Menu):  
  **Press `Alt + F`** → reboot from there.

**2. BIOS password lockout**  
- Use [BIOS Password Recovery](https://bios-pw.org/)  
- Enter system number shown when opening BIOS.  
- Generated password is admin → confirm with `Ctrl+Enter`.

**3. Real-time clock showing wrong date (e.g., 2016)**  
- Sync Baxter’s time with your computer.  
- Set in Baxter FSM or use NTP from your computer via command line.

---

## Networking & Communication
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

## ROS & Intera SDK Issues
**9. Wrong catkin workspace sourcing**  
```bash
source ~/intera_ws/devel/setup.bash
```

**10. enable_robot.py or joint_trajectory_action_server.py missing**  
- Run `catkin_make` or `catkin build` after troubleshooting.

**11. intera.sh script error**  
- Ensure file is in root of catkin workspace:  
  `~/intera_ws/intera.sh`

**12. MoveIt integration not working**  
- Ensure robot is enabled and joint trajectory server is active in a second terminal.

---

## Hardware & Motion Problems
**13. Arms not enabled or unresponsive**  
```bash
rosrun intera_interface enable_robot.py -e
```
- Test by gripping cuffs (zero-g mode should enable).

**14. Joint calibration errors**  
- Restart robot. Happens if you hit `CTRL+Z` mid-script.

---

## Testing, Debugging, & Logging
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

## Contribute
If you have more tips or fixes, open a pull request or an issue.

## License

MIT. See [LICENSE](LICENSE).
