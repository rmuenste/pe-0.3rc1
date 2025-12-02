# Adding New JSON-Configurable Parameters

This guide shows how to make a simulation parameter configurable via the JSON file so that it is loaded at startup. Follow these steps for each new parameter.

## 1) Add storage, defaults, and accessors
- File: `pe/config/SimulationConfig.h`
  - Add a member variable with a sensible default type (e.g., `real newParam_;`).
  - Add getter/setter methods (`getNewParam()`, `setNewParam(...)`).
- File: `src/config/SimulationConfig.cpp`
  - Initialize the new member in the `SimulationConfig` constructor with a default value.

## 2) Parse from JSON
- File: `src/config/SimulationConfig.cpp` inside `SimulationConfig::loadFromFile(...)`
  - Add a check for the JSON key and call the setter, for example:
    ```cpp
    if (j.contains("newParam_"))
        config.setNewParam(j["newParam_"].get<real>());
    ```
  - Keep key names consistent with the member variables already used (e.g., `"stepsize_"`).
  - JSON parsing requires builds with `HAVE_JSON` enabled; otherwise the loader is a no-op.

## 3) Apply the parameter in setup (if needed)
- If the parameter affects runtime behavior, apply it after loading the config in your setup routine (e.g., `pe/interface/sim_setup_serial.h`):
  ```cpp
  auto& config = SimulationConfig::getInstance();
  // after SimulationConfig::loadFromFile(...)
  someTarget->setNewParam( config.getNewParam() );
  ```
- Use the appropriate target object (world, collision system, solver, etc.).

## 4) Provide an example JSON entry
- File: `pe/interface/example.json` (or your scenario-specific config)
  - Add a key/value with the expected units, e.g.:
    ```json
    "newParam_": 0.001
    ```
  - Keep comments in documentation, not inside JSON.

## 5) Verify and document defaults
- Ensure the constructor default matches the intended “out-of-the-box” behavior.
- Mention units and typical ranges in code comments near the setter/getter or in related docs.

## Checklist
- [ ] Member variable + getter/setter in `SimulationConfig.h`
- [ ] Default initialized in `SimulationConfig` constructor
- [ ] JSON parsing added in `loadFromFile`
- [ ] Applied where used (setup/initialization)
- [ ] Example JSON updated
- [ ] Units and meaning documented
