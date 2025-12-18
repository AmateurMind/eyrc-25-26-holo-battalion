# Welcome to eYRC Holo Battalion Theme 2025-26

---
This is a public repository containing the required packages for the theme.


Theme devs: 
* Srivenkateshwar
* Bhavik Jain
* Sahil Shinde

---

## Changes Made

### Mesh File Format Update
- **File Changed:** `hb_description/models/holonomic_bot/hb_bot.xacro`
- **Change:** Updated the base_link mesh from `.dae` (COLLADA) format to `.stl` format
  - Changed `bot.dae` → `base.stl` for both visual and collision geometry
- **Reason:** STL format is more widely supported for 3D printing and robotics simulations

