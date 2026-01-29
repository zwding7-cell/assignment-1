# AAE5303 Environment Setup Report — Template for Students

> **Important:** Follow this structure exactly in your submission README.  
> Your goal is to demonstrate **evidence, process, problem-solving, and reflection** — not only screenshots.

---

## 1. System Information

**Laptop model:**  
_[LAPTOP-V2JF7ABJ.]_

**CPU / RAM:**  
_[Intel(R) Core(TM) Ultra 9 275HX (2.70 GHz)，16GB RAM]_

**Host OS:**  
_[Windows 11]_

**Linux/ROS environment type:**  
_[Choose one:]_
- [ ] Dual-boot Ubuntu
- [√ ] WSL2 Ubuntu
- [ ] Ubuntu in VM (UTM/VirtualBox/VMware/Parallels)
- [ ] Docker container
- [ ] Lab PC
- [ ] Remote Linux server

---

## 2. Python Environment Check

### 2.1 Steps Taken

Describe briefly how you created/activated your Python environment:

**Tool used:**  
_[venv]_

**Key commands you ran:**
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

**Any deviations from the default instructions:**  

The repository does not provide a requirements.txt file.
Problem:
pip install -r requirements.txt failed because the file does not exist.

Cause:
The repository only contains a README.md file and does not include a
requirements.txt file for dependency installation.

Solution:
Instead of installing dependencies, the Python environment was verified by
checking Python and pip versions directly.

Verification:
The virtual environment activated successfully and Python version checks passed.

### 2.2 Test Results

Run these commands and paste the actual terminal output (not just screenshots):

```bash
python scripts/test_python_env.py
```

**Output:**
```
(.venv) yourusername@LAPTOP-V2JF7ABJ:~/PolyU-AAE5303-env-smork-test$ cd ~/PolyU-AAE5303-env-smork-test
source .venv/bin/activate
source /opt/ros/humble/setup.bash
command -v ros2
python3 scripts/test_python_env.py
/opt/ros/humble/bin/ros2
========================================
AAE5303 Environment Check (Python + ROS)
Goal: help you verify your environment and understand what each check means.
========================================

Step 1: Environment snapshot
  Why: We capture platform/Python/ROS variables to diagnose common setup mistakes (especially mixed ROS env).
Step 2: Python version
  Why: The course assumes Python 3.10+; older versions often break package wheels.
Step 3: Python imports (required/optional)
  Why: Imports verify packages are installed and compatible with your Python version.
Step 4: NumPy sanity checks
  Why: We run a small linear algebra operation so success means more than just `import numpy`.
Step 5: SciPy sanity checks
  Why: We run a small FFT to confirm SciPy is functional (not just installed).
Step 6: Matplotlib backend check
  Why: We generate a tiny plot image (headless) to confirm plotting works on your system.
Step 7: OpenCV PNG decoding (subprocess)
  Why: PNG decoding uses native code; we isolate it so corruption/codec issues cannot crash the whole report.
Step 8: Open3D basic geometry + I/O (subprocess)
  Why: Open3D is a native extension; ABI mismatches can segfault. Subprocess isolation turns crashes into readable failures.
Step 9: ROS toolchain checks
  Why: The course requires ROS tooling. This check passes if ROS 2 OR ROS 1 is available (either one is acceptable).
  Action: building ROS 2 workspace package `env_check_pkg` (this may take 1-3 minutes on first run)...
  Action: running ROS 2 talker/listener for a few seconds to verify messages flow...
Step 10: Basic CLI availability
  Why: We confirm core commands exist on PATH so students can run the same commands as in the labs.

=== Summary ===
✅ Environment: {
  "platform": "Linux-6.6.87.2-microsoft-standard-WSL2-x86_64-with-glibc2.35",
  "python": "3.10.12",
  "executable": "/home/yourusername/PolyU-AAE5303-env-smork-test/.venv/bin/python3",
  "cwd": "/home/yourusername/PolyU-AAE5303-env-smork-test",
  "ros": {
    "ROS_VERSION": "2",
    "ROS_DISTRO": "humble",
    "ROS_ROOT": null,
    "ROS_PACKAGE_PATH": null,
    "AMENT_PREFIX_PATH": "/opt/ros/humble",
    "CMAKE_PREFIX_PATH": null
  }
}
✅ Python version OK: 3.10.12
✅ Module 'numpy' found (v2.2.6).
✅ Module 'scipy' found (v1.15.3).
✅ Module 'matplotlib' found (v3.10.8).
✅ Module 'cv2' found (v4.13.0).
✅ Module 'rclpy' found (vunknown).
✅ numpy matrix multiply OK.
✅ numpy version 2.2.6 detected.
✅ scipy FFT OK.
✅ scipy version 1.15.3 detected.
✅ matplotlib backend OK (Agg), version 3.10.8.
✅ OpenCV OK (v4.13.0), decoded sample image 128x128.
✅ Open3D OK (v0.19.0), NumPy 2.2.6.
✅ Open3D loaded sample PCD with 8 pts and completed round-trip I/O.
✅ ROS 2 CLI OK: /opt/ros/humble/bin/ros2
✅ ROS 1 tools not found (acceptable if ROS 2 is installed).
✅ colcon found: /usr/bin/colcon
✅ ROS 2 workspace build OK (env_check_pkg).
✅ ROS 2 runtime OK: talker and listener exchanged messages.
✅ Binary 'python3' found at /home/yourusername/PolyU-AAE5303-env-smork-test/.venv/bin/python3

All checks passed. You are ready for AAE5303 🚀
```

```bash
python scripts/test_open3d_pointcloud.py
```

**Output:**
```
(.venv) yourusername@LAPTOP-V2JF7ABJ:~/PolyU-AAE5303-env-smork-test$ python scripts/test_open3d_pointcloud.py
ℹ️ Loading /home/yourusername/PolyU-AAE5303-env-smork-test/data/sample_pointcloud.pcd ...
✅ Loaded 8 points.
   • Centroid: [0.025 0.025 0.025]
   • Axis-aligned bounds: min=[0. 0. 0.], max=[0.05 0.05 0.05]
✅ Filtered point cloud kept 7 points.
✅ Wrote filtered copy with 7 points to /home/yourusername/PolyU-AAE5303-env-smork-test/data/sample_pointcloud_copy.pcd
   • AABB extents: [0.05 0.05 0.05]
   • OBB  extents: [0.08164966 0.07071068 0.05773503], max dim 0.0816 m
🎉 Open3D point cloud pipeline looks good.
```

**Screenshot:**  
_[Include one screenshot showing both tests passing]_
<img width="1281" height="969" alt="image" src="https://github.com/user-attachments/assets/f56c129d-ddc5-4566-ac40-51f39057693d" />
<img width="1399" height="263" alt="image" src="https://github.com/user-attachments/assets/b94d4f2c-2ddf-4740-84a2-19e3d9487a62" />


---

## 3. ROS 2 Workspace Check

### 3.1 Build the workspace

Paste the build output summary (final lines only):

```bash
source /opt/ros/humble/setup.bash
colcon build
```

**Expected output:**
```
Summary: 1 package finished [x.xx s]
```

**Your actual output:**
```
[colcon build
Starting >>> env_check_pkg
Finished <<< env_check_pkg [6.97s]                     

Summary: 1 package finished [7.24s]]
```

### 3.2 Run talker and listener

Show both source commands:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

**Then run talker:**
```bash
ros2 run env_check_pkg talker.py
```

**Output (3–4 lines):**
```
[[INFO] [1769689108.890873868] [env_check_pkg_talker]: AAE5303 talker ready (publishing at 2 Hz).
[INFO] [1769689109.414385206] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #0'
[INFO] [1769689109.937667815] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #1'
[INFO] [1769689110.461002431] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #2'
[INFO] [1769689110.984384991] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #3']
```

**Run listener:**
```bash
ros2 run env_check_pkg listener.py
```

**Output (3–4 lines):**
```
[[INFO] [1769689095.287726640] [env_check_pkg_listener]: AAE5303 listener awaiting messages.
[INFO] [1769689109.414533182] [env_check_pkg_listener]: I heard: 'AAE5303 hello #0'
[INFO] [1769689109.937933721] [env_check_pkg_listener]: I heard: 'AAE5303 hello #1'
[INFO] [1769689110.461294981] [env_check_pkg_listener]: I heard: 'AAE5303 hello #2'
[INFO] [1769689110.984589435] [env_check_pkg_listener]: I heard: 'AAE5303 hello #3']
```

**Alternative (using launch file):**
```bash
ros2 launch env_check_pkg env_check.launch.py
```

**Screenshot:**  
_[Include one screenshot showing talker + listener running]_

<img width="1412" height="1071" alt="image" src="https://github.com/user-attachments/assets/cc315eb9-6ac0-40dd-b64a-6c994239bbf8" />


---

## 4. Problems Encountered and How I Solved Them

> **Note:** Write 2–3 issues, even if small. This section is crucial — it demonstrates understanding and problem-solving.

### Issue 1: [ROS requirement not satisfied during Python environment test.]

**Cause / diagnosis:**  
_[The Python test script was executed inside a virtual environment without
sourcing the ROS 2 setup file. Therefore, ROS-related commands were not
available in that terminal session.]_

**Fix:**  
_[Opened a new terminal and manually sourced the ROS 2 environment before running
any ROS-related commands.]_

```bash
[source /opt/ros/humble/setup.bash]
```

**Reference:**  
_[Terminal output, official ROS 2 documentation, and AI assistant]_

---

### Issue 2: [failed due to missing file]

**Cause / diagnosis:**  
_[The repository did not provide a requirements.txt file. Attempting to follow
generic Python setup instructions resulted in a file-not-found error.]_

**Fix:**  
_[Instead of installing dependencies blindly, I inspected the repository structure
and README.md. I verified the Python environment by running the provided test
scripts directly and checking Python and pip versions inside the virtual
environment.]_

```bash
[python --version
pip --version
]
```

**Reference:**  
_[Repository README.md and AI assistant]_

---

### Issue 3 (Optional): [Confusion between Python virtual environment and ROS environment]

**Cause / diagnosis:**  
_[At first, I assumed activating the Python virtual environment would also make
ROS tools available. I later realized that Python virtual environments and ROS
environments are independent and must be sourced separately.]_

**Fix:**  
_[I learned to clearly separate responsibilities:

Use .venv for Python package isolation

Use source /opt/ros/humble/setup.bash for ROS tooling]_

```bash
[source .venv/bin/activate
source /opt/ros/humble/setup.bash]
```

**Reference:**  
_[ROS 2 documentation and AI assistant explanation.]_

---

## 5. Use of Generative AI (Required)

Choose one of the issues above and document how you used AI to solve it.

> **Goal:** Show critical use of AI, not blind copying.

### 5.1 Exact prompt you asked

**Your prompt:**
```
[My Python environment test says:
"ROS requirement not satisfied: neither ROS 2 nor ROS 1 appears to be installed/working".
But I already installed ROS 2. Why does this happen and how do I fix it?]
```

### 5.2 Key helpful part of the AI's answer

**AI's response (relevant part only):**
```
[This usually happens when ROS is installed but the setup.bash file has not been
sourced in the current terminal. You need to run:
source /opt/ros/humble/setup.bash
before running any ROS-related commands.]
```

### 5.3 What you changed or ignored and why

Explain briefly:
- Did the AI recommend something unsafe?
- Did you modify its solution?
- Did you double-check with official docs?

**Your explanation:**  
_[The AI suggested sourcing the ROS environment, which aligned with the official
ROS documentation. I verified the solution by checking environment variables
after sourcing. No unsafe or unnecessary steps were suggested, so the solution
was applied directly.]_

### 5.4 Final solution you applied

Show the exact command or file edit that fixed the problem:

```bash
[source /opt/ros/humble/setup.bash]
```

**Why this worked:**  
_[Sourcing the ROS setup file correctly configured the environment variables and
PATH entries required for ROS tools, allowing the Python test script to detect
the ROS installation successfully.]_

---

## 6. Reflection (3–5 sentences)

Short but thoughtful:

- What did you learn about configuring robotics environments?
- What surprised you?
- What would you do differently next time (backup, partitioning, reading error logs, asking better AI questions)?
- How confident do you feel about debugging ROS/Python issues now?

**Your reflection:**

_[This setup process helped me understand that robotics environments are composed
of multiple independent layers, such as Python virtual environments and ROS
toolchains. I was surprised by how many issues were caused simply by environment
variables not being sourced correctly. Through debugging, I learned to rely on
terminal outputs and error messages instead of guessing. Next time, I would read
the repository structure more carefully before running installation commands.
Overall, I feel more confident in diagnosing and fixing ROS and Python
environment issues.]_

---

## 7. Declaration

✅ **I confirm that I performed this setup myself and all screenshots/logs reflect my own environment.**

**Name:**  
_[Ding Zhongwei]_

**Student ID:**  
_[25059382G]_

**Date:**  
_[2026/01/29]_

---

## Submission Checklist

Before submitting, ensure you have:

- [ √] Filled in all system information
- [√ ] Included actual terminal outputs (not just screenshots)
- [√ ] Provided at least 2 screenshots (Python tests + ROS talker/listener)
- [√ ] Documented 2–3 real problems with solutions
- [√ ] Completed the AI usage section with exact prompts
- [√ ] Written a thoughtful reflection (3–5 sentences)
- [√ ] Signed the declaration

---

**End of Report**
