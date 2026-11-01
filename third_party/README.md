# Person detection and tracking sources

The team keeps upstream-linked forks as pinned Git submodules for later evaluation.

| Source | Team fork | Original upstream | License |
| --- | --- | --- | --- |
| YOLO26 and Ultralytics tracking | [HARP/ultralytics](https://github.com/Humanoid-Athletic-Robotics-Project-HARP/ultralytics) | [ultralytics/ultralytics](https://github.com/ultralytics/ultralytics) | AGPL-3.0; see `ultralytics/LICENSE` |
| BoT-SORT | [HARP/BoT-SORT](https://github.com/Humanoid-Athletic-Robotics-Project-HARP/BoT-SORT) | [NirAharon/BoT-SORT](https://github.com/NirAharon/BoT-SORT) | MIT; see `BoT-SORT/LICENSE` |

## Get the source

From the social-interaction repository root:

```bash
git submodule update --init --recursive third_party/ultralytics third_party/BoT-SORT
```

The parent repository records exact revisions. Inspect them with:

```bash
git submodule status
```

Update a fork and advance its submodule revision deliberately, then commit the
new pointer in this repository. Updating a fork alone does not change our pin.

## Starting points

- `ultralytics/ultralytics/cfg/models/26/yolo26.yaml`: YOLO26 architecture.
- `ultralytics/ultralytics/cfg/trackers/botsort.yaml`: integrated tracker settings.
- `BoT-SORT/README.md` and `BoT-SORT/tracker/bot_sort.py`: standalone tracker reference.

Ultralytics already provides a BoT-SORT integration. Evaluate that path before
adding the standalone tracker as another runtime dependency.

## Integration boundary

These are source references for evaluating person detection and association.
Our submission build copies its selected application files, so these sources
are outside the image and startup path. Model packages and weights have not
been installed or evaluated by adding the submodules.

Walking up to a person also needs valid depth, observation-time transforms,
spatial track freshness/uncertainty, target selection and an approach-goal
policy. Confirm the target camera, compute, Python/JetPack and inference
runtime before choosing a deployment. Keep upstream license and notice files;
check the terms of any additional model weights when selecting them.
