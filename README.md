# Brief

A floating-point C implementation of a parallel PID controller discretized using the bilinear (Tustin) transform.

**Full derivation can be found [here](https://www.mujoseph.com/projects/robust-pid).**

# Directory

```source``` folder contains the driver files. Implement platform specific types in ```platform_types.h```.

```sim``` folder contains [QSPICE](https://www.qorvo.com/design-hub/calculators-simulation/qspice) schematic(s) for [validating](https://www.mujoseph.com/projects/robust-pid#validation) the *robust_pid* driver.

# Usage

Fill a `robust_pid_cfg_t` and pass it to `robust_pid_initCtx()`.

```c
#include "robust_pid.h"

// PID controller context
static robust_pid_t pid;

robust_pid_cfg_t cfg = 
{
    .T          = 0.001f,   // PID loop invocation interval, sec 
    .LPF_3dB_Hz = 50.0f,    // Derivative branch LPF cutoff, Hz
    .Kp         = 1.2f,     // Proportional gain
    .Ki         = 0.4f,     // Integral gain
    .Kd         = 0.02f,    // Derivative gain
};


if (!robust_pid_initCtx(&pid, &cfg))
{
    // handle invalid configuration
}
```

Call `robust_pid_run()` once per sample period `robust_pid_cfg_t.T`.

```c
// Run PID controller
float u = robust_pid_run(&pid, setpoint, measurement);
```

Change gains with `robust_pid_updateGains()` and the LPF cutoff with `robust_pid_updateLPFCutoffFrequency()`.

```c
robust_pid_updateGains(&pid, 1.5f, 0.4f, 0.02f);
robust_pid_updateLPFCutoffFrequency(&pid, 80.0f); 
```
