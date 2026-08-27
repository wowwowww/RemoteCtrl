# SR AMR Interfaces

> ROS2 Distro: Humble

StandardRobots AMR ROS2 interfaces

This package contains the custom (non ROS2 builtin ones) messages, services and actions definitions for StandardRobots AMR platform(Also known as SROS, Standard Robots Operating System).

## Messages

- `BatteryState`: Battery status reported by the AMR.
- `Path`: Path segment definition for movement actions.
- `SystemState`: AMR system, localization, and movement status.

```bash
colcon build --packages-select sr_amr_interfaces
```

You should source the install/setup.bash after building the package to use the defined interfaces in other packages.

```bash
source install/setup.zsh
```

## License

This project is licensed under the BSD-3-Clause License - see the [LICENSE](../LICENSE) file for details.
