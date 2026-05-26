# Changelog

This changelog summarizes major feature additions and important fixes from the initial project history through `56d6b3d`.

## Highlights (chronological)

- **Initial platform baseline (2024-03-11):** Core Project AirSim codebase was introduced.
- **MSVC compatibility fix (2025-07-15):** Fixed `__has_feature` macro issue for Windows toolchains.
- **Linux setup/package fixes (2026-01):** Fixed missing dependency handling and Linux dev tool installation issues.
- **Depth capture correctness (2026-01-22):** Fixed depth copy behavior from Unreal.
- **Local file spec handling (2026-02-10):** Fixed `make_base_specs()` behavior for local files.
- **Configuration/runtime robustness (2026-05):** Improved `UE_ROOT` relaxed Open3D constraints, and fixed namespace/unit-test integration issues.
- **UE 5.7 support (2026-02-24):** Upgraded project compatibility to Unreal Engine 5.7.
- **Build metadata API (2026-03-18):** Added build commit hash service and client helper.
- **Simulation clock mode expansion (2026-05):** Added and finalized engine-driven/external clock support, including schema and demo updates.
- **Depth LiDAR support (2026-04-27 to 2026-05-12):** Implemented DepthLiDAR and added UE 5.7 compatibility updates.