# chili_crane_bringup

Launch files compose profiles under a per-crane namespace.

Phase 0 launch files start only the fail-safe control skeleton and optional mock
adapter. They do not start an NDT mapper/localizer or real hardware driver.
`mapping` and `localization` currently establish mutually exclusive map
lifecycle parameters for later nodes.

The launch-file default `config_root` resolves the repository-level `config`
directory in a source workspace. Installed deployments must pass
`config_root:=<absolute-linux-path>`. Every public entry launch accepts this
argument. Phase 0 intentionally does not claim that repository-level configs
are installed into package share; that packaging decision remains for Phase 1.
