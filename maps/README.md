# Map Artifacts

Do not commit large production PCDs by default.

Expected versioned layout:

```text
maps/track_01/
  localization_map.pcd
  display_map.pcd
  semantic_map.yaml
  calibration.yaml
  manifest.json
```

`manifest.json` must identify the map version, source run/bag, source Git SHA,
creation time, frame, semantic-map version, LiDAR extrinsic version, and servo
calibration version. It also records SHA-256 checksums for all four artifacts;
paths alone are not sufficient map identity.
