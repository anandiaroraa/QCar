# MPC Hardware vs Simulation Code Comparison Analysis

## Overview
You are **heading in the right direction**! The `MPC_Hardware` folder represents a well-considered adaptation of your simulation code for real hardware. Here's a detailed breakdown:

---

## 1. `mpcspeed_steercontrol.py` Comparison

### ✅ Good Changes (Hardware Folder)

| Change | Reason | Status |
|--------|--------|--------|
| **Relative imports** (`from .qcar_params` vs `from qcar_params`) | Package structure - MPC_Hardware is a proper package | ✓ Correct |
| **Matplotlib disabled** (`show_animation = False`) | Hardware doesn't need visualization | ✓ Correct |
| **plot_car() commented out** | Matplotlib unavailable on hardware | ✓ Correct |
| **MPC failure handling** | New check for `None` values after MPC solve | ✓ Good defensive coding |
| **Tuned parameters** | `Q=[1.0, 1.0, 0.2, 0.8]`, `Rd=[0.1, 1.0]` | ✓ Hardware-optimized |

### ⚠️ Issues to Address

1. **Missing `calc_ref_trajectory` and other functions**
   - The hardware file appears incomplete in the shown range
   - Need to verify all functions from simulation are present

2. **Parameter tuning rationale**
   - `Q` matrix changes: velocity weight reduced (0.5→0.2), yaw weight increased (0.5→0.8)
   - `Rd` change: acceleration penalty increased (0.01→0.1)
   - These make sense but consider documenting why

---

## 2. `shared.py` Comparison

### Major Differences (Hardware > Simulation)

#### **A. Coordinate System Transformation** ⚠️ CRITICAL
```python
# SIMULATION (mpc/shared.py)
self.car1.quat = [orientation.w, orientation.x, orientation.y, orientation.z]
self.car1.theta = quat2euler([...])
self.car1.x = pose.x
self.car1.y = pose.y

# HARDWARE (MPC_Hardware/shared.py)
self.car1.quat = [orientation.x, orientation.y, orientation.z, orientation.w]  # Different order!
self.car1.theta = quat2euler([...])[2]  # Extract only yaw
self.car1.x = pose.z  # ← Mapped from pose.z (NOT pose.x)!
self.car1.y = -pose.x  # ← Negated and mapped from pose.x!
```

**Key Insight:** The hardware uses a different OptiTrack coordinate frame mapping:
- Simulation assumes: `x → x, y → y`
- Hardware maps: `z → x, -x → y` (90° rotation + negation)

#### **B. Circle Center Calculation** 
```python
# SIMULATION
center_x = float(cfg.get("center_x", 0.0))  # Config-based
center_y = float(cfg.get("center_y", 0.0))

# HARDWARE
yaw0 = data.car1.theta
center_x = data.car1.x - radius * np.sin(yaw0)  # Auto-calculated!
center_y = data.car1.y + radius * np.cos(yaw0)
```

**Benefit:** Hardware auto-calculates center based on car's initial pose and heading. Smart for real-world deployment where you want the circle tangent to starting direction.

#### **C. Path Discretization**
```python
# SIMULATION
dl = float(cfg.get("ds", 0.05))  # Finer: 5cm waypoints

# HARDWARE
dl = float(cfg.get("ds", 0.5))   # Coarser: 50cm waypoints
```

**Reason:** Fewer waypoints = fewer MPC calculations = faster on actual hardware.

#### **D. Target Speed**
```python
# SIMULATION: 0.6 m/s
# HARDWARE: 0.18 m/s
```

**Reason:** Hardware runs slower for safety/stability.

#### **E. Debug Output & Diagnostics** 
Hardware has extensive logging:
```python
# Print every 10 pose updates
if self.car1._print_count % 10 == 0:
    print(f"x={self.car1.x:.3f} y={self.car1.y:.3f} yaw={np.rad2deg(self.car1.theta):.1f}deg")

# Circle geometry verification
print(f"Car start: x={data.car1.x:.3f}, y={data.car1.y:.3f}")
print(f"Circle center: x={center_x:.3f}, y={center_y:.3f}")
print(f"target_ind at start: {target_ind}")

# MPC output logging
print(f"steer1={np.rad2deg(steer1):.2f}deg, speed1={speed1:.3f}m/s")
```

#### **F. Error Handling**
```python
# HARDWARE: Try-except wrapper around MPC
try:
    result = iterative_linear_mpc_control(xref, x0, dref, oa, odelta)
    oa_new, odelta_new = result[0], result[1]
except Exception as e:
    print(f"MPC exception: {e}")
    oa_new, odelta_new = None, None

if oa_new is None or odelta_new is None:
    steer1 = 0.0
    speed1 = 0.0  # Safe fallback
```

#### **G. Waypoint Search Algorithm**
```python
# SIMULATION
target_ind, _ = calc_nearest_index(...)

# HARDWARE: Full circle search for best starting point
target_ind = 0
min_dist = float('inf')
for i in range(len(cx)):
    d = math.hypot(data.car1.x - cx[i], data.car1.y - cy[i])
    if d < min_dist:
        min_dist = d
        target_ind = i
```

Better for real hardware where initial position might not be exactly on the path.

---

## 3. Key Architectural Decisions

### What's Working Well ✅
1. **Modular structure** - Separate `mpc/` and `MPC_Hardware/` folders
2. **Relative imports** - Makes the hardware folder a proper Python package
3. **Safety-first approach** - Multiple checks for `None` returns, fallback speeds
4. **Real-world tuning** - Parameters adjusted for hardware constraints
5. **Comprehensive logging** - Helps debug OptiTrack/MPC issues

### What Needs Attention ⚠️

1. **Coordinate Frame Documentation**
   - Add comments explaining the `x → z, y → -x` transformation
   - Include OptiTrack → robot coordinate frame mapping diagram

2. **Parameter Justification**
   - Document why `Q=[1.0, 1.0, 0.2, 0.8]` works better than `[1.0, 1.0, 0.5, 0.5]`
   - Test different tunings systematically

3. **Yaw Angle Handling**
   - Hardware version extracts `[2]` from quat2euler (yaw only)
   - Verify this is correct for your OptiTrack setup
   - Consider edge cases near ±π discontinuities

4. **Speed Command Integration**
   - Hardware uses: `speed1 = np.clip(last_speed_cmd + a_cmd * DT, ...)`
   - Simulation uses: `speed1 = np.clip(state.v + a_cmd * DT, ...)`
   - **Question:** Which is more accurate? Does your robot integrate velocity correctly?

5. **Circle Center Calculation Verification**
   - Does auto-calculating center from initial pose work well in practice?
   - Consider adding visualization of expected vs actual path

---

## 4. What Needs to Be Done Next

### Priority 1: Validation ⚠️
- [ ] **Verify coordinate frame transformation** - Run a test with known positions
- [ ] **Confirm quat2euler function** - Does it match OptiTrack's convention?
- [ ] **Test MPC robustness** - Run multiple hardware trials, collect data
- [ ] **Validate circle tracking** - Compare expected path vs actual trajectory

### Priority 2: Documentation
- [ ] Add coordinate frame mapping diagram to README
- [ ] Document all parameter tuning decisions
- [ ] Create troubleshooting guide for common hardware issues
- [ ] Add comments explaining `update_car1_pose()` transformations

### Priority 3: Code Synchronization
- [ ] Check if `mpc/` has all functions that `MPC_Hardware/` imports
- [ ] Create a shared utility module for common functions
- [ ] Add unit tests for coordinate transformations

### Priority 4: Hardware-Specific Features
- [ ] Add data logging to `.npz` format (already in `shared.py`)
- [ ] Create replay/visualization tool for recorded trajectories
- [ ] Add emergency stop button handling
- [ ] Implement timeout/watchdog for stuck MPC

### Priority 5: Optimization
- [ ] Profile MPC solve time on actual hardware
- [ ] Consider reducing `T` (horizon length) if too slow
- [ ] Benchmark different solvers (CLARABEL vs ECOS vs SCS)

---

## 5. Summary

**Your approach is sound:** You've created a separate hardware-adapted version with:
- ✅ Proper package structure (relative imports)
- ✅ Defensive error handling
- ✅ Real-world tuning (slower speed, coarser paths)
- ✅ Better initial waypoint selection
- ✅ Extensive diagnostics

**Critical next step:** Verify the coordinate frame transformations are correct for YOUR OptiTrack setup. The `x → z, y → -x` mapping is specific to your hardware configuration and needs validation.

**Test systematically:** Run trials, collect data in `hardware_results_test1/`, and iterate on parameters.
