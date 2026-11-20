# Launcher MPC

The launcher's flywheel speed control: MPC with a Kalman load observer, on its own 100 Hz thread.

Documentation:
- **[docs/launcher.md](../../../../../../../../../../docs/launcher.md)**: setup, tuning (SysId, Auto-Tune, by hand), API, `notifyShot()`, parameters, tools, troubleshooting.
- **[docs/launcher-internals.md](../../../../../../../../../../docs/launcher-internals.md)**: how it works and why (model, Kalman filter, MPC/QP, shot detection, feed-forward, threading, SysId, scoring, Auto-Tune, verification).

Quick start: set motor names and directions in `LauncherMPC.defaultConfig()`, run **Launcher SysId** and then **Launcher Auto-Tune** (tuning group; it saves the best values itself), and in code:

```java
LauncherMPC launcher = new LauncherMPC(this);
launcher.setTargetVelocity(1800);   // ticks/s
if (launcher.isReady()) feed();
```
