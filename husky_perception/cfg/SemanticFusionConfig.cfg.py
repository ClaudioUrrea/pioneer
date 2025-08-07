#!/usr/bin/env python3
PACKAGE = "husky_perception"  # Use your package name

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()

gen.add("min_score_threshold", double_t, 0, "Below this, detections are discarded", 0.4, 0.0, 1.0)
gen.add("raw_downsample", int_t, 0, "Stride for raw depth downsampling", 3, 1, 10)
gen.add("spread_factor", double_t, 0, "Scales ray coverage", 1.15, 1.0, 3.0)
gen.add("priority_rays_high", int_t, 0, "Rays for high-priority objects", 15, 1, 50)
gen.add("priority_rays_medium", int_t, 0, "Rays for medium-priority objects", 9, 1, 50)
gen.add("priority_rays_low", int_t, 0, "Rays for low-priority objects", 5, 1, 50)
gen.add("priority_coverage_high", double_t, 0, "Coverage diameter for high priority (m)", 2.0, 0.1, 5.0)
gen.add("priority_coverage_medium", double_t, 0, "Coverage diameter for medium priority (m)", 1.2, 0.1, 5.0)
gen.add("priority_coverage_low", double_t, 0, "Coverage diameter for low priority (m)", 0.5, 0.1, 5.0)

exit(gen.generate(PACKAGE, "your_node_namespace", "SemanticFusionConfig"))
