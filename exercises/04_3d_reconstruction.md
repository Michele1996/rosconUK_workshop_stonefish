# Exercise 4 — Reconstruct What You Surveyed

1. Record the RGB camera topic in a ROS 2 bag.
2. Extract frames:

```bash
python3 reconstruction/scripts/extract_images_from_bag.py   <bag_directory> <rgb_topic> my_survey/images
```

3. Run COLMAP:

```bash
./reconstruction/scripts/run_colmap.sh my_survey/images
```

or GLOMAP:

```bash
./reconstruction/scripts/run_glomap.sh my_survey/images
```

4. Inspect:
- registered images;
- estimated camera trajectory;
- coverage;
- holes/disconnected components;
- whether adjacent survey lines connect.

If your own acquisition fails, use the known-good reference image sequence that
will be generated from the final workshop environment.
