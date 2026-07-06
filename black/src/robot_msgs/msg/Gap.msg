float32 xs                    # smaller x where base_link x-axis intersects gap boundary (a->b)
float32 xe                    # larger x where base_link x-axis intersects gap boundary (a->c)
float32 ymin                  # lateral min along base_link y axis (m)
float32 ymax                  # lateral max along base_link y axis (m)
float32 confidence            # confidence [0, 1]

bool has_bridge_dir_b         # whether bridge_dir_b is valid
float32[2] bridge_dir_b       # unit direction of bridge in base_link XY
