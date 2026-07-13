# D-LIO tuning profiles

Use a profile to run a reproducible estimator-side parameter experiment against
the same raw bag:

```bash
bash scripts/dlio/reconstruct_raw.sh --tuning-profile baseline bags/<bag>
```

A profile is stored in `config/dlio/tuning/profiles/<name>.yaml` and may have
only these keys:

```yaml
name: descriptive-profile-name
description: What single estimator-side change this profile evaluates.
dlio:
  adaptive: false
params:
  odom/preprocessing/voxelFilter/res: 0.03
```

`dlio` overrides parameters from the selected D-LIO calibration config.
`params` overrides parameters from the selected D-LIO runtime config. Start
from `baseline`, change one estimator-side parameter family at a time, and do
not mix RViz/output-only changes into a profile.

Each profile run writes generated effective configuration files and a
`manifest.yaml` to `.dlio-tuning-runs/`. The manifest records the bag, expected
input message counts, replay arguments, and SHA-256 digests of the effective
configuration files.
