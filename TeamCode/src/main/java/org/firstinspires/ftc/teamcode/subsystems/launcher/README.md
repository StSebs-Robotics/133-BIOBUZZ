# Launcher MPC

The launcher's flywheel speed control: MPC with a Kalman load observer, on its own 100 Hz thread.

Full documentation (setup, tuning recipe, API, how it works, parameters, tools) is in **[docs/launcher.md](../../../../../../../../../../docs/launcher.md)**.

Quick start: set motor names and directions in `LauncherMPC.defaultConfig()`, run **Launcher SysId** and then **Launcher MPC Tuner** (tuning group), and in code:

```java
LauncherMPC launcher = new LauncherMPC(this);
launcher.setTargetVelocity(1800);   // ticks/s
if (launcher.isReady()) feed();
```
