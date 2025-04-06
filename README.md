# hive_robot

This repo contains **3 branches**(robot1, robot2, and robot3) for each robot.
---

### ⚠️ License & Patent Notice

> This repository is provided **for non-commercial academic and research use only**.
> **Commercial use is strictly prohibited** without a separate license agreement.

- This project includes technology that is **patent-pending in South Korea**
  (Korean Patent Application No. **10-2025-0008102**)
- If you use this code or its ideas in your research or projects,
  **you must provide proper attribution** to the original author.
- **Use of this software or its ideas for commercial purposes is not allowed.**

For commercial licensing inquiries, please contact:
**[xogns2079@gmail.com]**

---


---
# Motor Control

I used linear regression to control a PWM motor by RPM. 
By collecting data on the RPM values for each PWM, I performed linear regression to control the RPM.

Additionally, I used the desired omega and v from the controller to perform RPM control through the kinematics of a differential wheel robot.

![image](https://github.com/BEYOND-thelimit/hive_robot/assets/110722569/61238562-7a1a-45f7-9871-fb40becf58b9)

## Input Velocity
Values received through the callback:

- `v`: Linear velocity (m/s)
- `ω`: Angular velocity (rad/s)

## Velocity for Each Wheel
The velocity for each wheel is calculated as:

- `v_right = v - (ω * (L / 2))`
- `v_left = v + (ω * (L / 2))`

where:
- `v_right`: Velocity of the right wheel
- `v_left`: Velocity of the left wheel
- `L`: Wheelbase (distance between the two wheels)

## Velocity to RPM for Each Wheel
Convert the velocity of each wheel to RPM:

- `RPM_right = (v_right / (2 * π * r)) * 60`
- `RPM_left = (v_left / (2 * π * r)) * 60`

where:
- `RPM_right`: RPM of the right wheel
- `RPM_left`: RPM of the left wheel
- `r`: Radius of the wheel


