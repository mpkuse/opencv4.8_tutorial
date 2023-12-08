# Common RGB colors as Python lists
white_rgb = [255, 255, 255]
black_rgb = [0, 0, 0]
red_rgb = [255, 0, 0]
green_rgb = [0, 255, 0]
blue_rgb = [0, 0, 255]
yellow_rgb = [255, 255, 0]
cyan_rgb = [0, 255, 255]
magenta_rgb = [255, 0, 255]
gray_rgb = [128, 128, 128]
purple_rgb = [128, 0, 128]
orange_rgb = [255, 165, 0]
brown_rgb = [165, 42, 42]
pink_rgb = [255, 192, 203]
teal_rgb = [0, 128, 128]
olive_rgb = [128, 128, 0]

# Example: Using the colors in Open3D point cloud colors
colors = [red_rgb, green_rgb, blue_rgb, yellow_rgb, cyan_rgb, magenta_rgb]

# Example: Creating a color map
color_map_u = {
    'red': red_rgb,
    'green': green_rgb,
    'blue': blue_rgb,
    'yellow': yellow_rgb,
    'cyan': cyan_rgb,
    'magenta': magenta_rgb,
    'gray': gray_rgb,
    'purple': purple_rgb,
    'orange': orange_rgb,
    'brown': brown_rgb,
    'pink': pink_rgb,
    'teal': teal_rgb,
    'olive': olive_rgb,
}


# Common RGB colors as Python lists in the range [0.0, 1.0]
white_rgb_normalized = [1.0, 1.0, 1.0]
black_rgb_normalized = [0.0, 0.0, 0.0]
red_rgb_normalized = [1.0, 0.0, 0.0]
green_rgb_normalized = [0.0, 1.0, 0.0]
blue_rgb_normalized = [0.0, 0.0, 1.0]
yellow_rgb_normalized = [1.0, 1.0, 0.0]
cyan_rgb_normalized = [0.0, 1.0, 1.0]
magenta_rgb_normalized = [1.0, 0.0, 1.0]
gray_rgb_normalized = [0.5, 0.5, 0.5]
purple_rgb_normalized = [0.5, 0.0, 0.5]
orange_rgb_normalized = [1.0, 0.65, 0.0]
brown_rgb_normalized = [0.65, 0.16, 0.16]
pink_rgb_normalized = [1.0, 0.75, 0.79]
teal_rgb_normalized = [0.0, 0.5, 0.5]
olive_rgb_normalized = [0.5, 0.5, 0.0]

# Example: Using the normalized colors in Open3D point cloud colors
colors_normalized = [red_rgb_normalized, green_rgb_normalized, blue_rgb_normalized, yellow_rgb_normalized, cyan_rgb_normalized, magenta_rgb_normalized]

# Example: Creating a color map with normalized colors
color_map_normalized = {
    'red': red_rgb_normalized,
    'green': green_rgb_normalized,
    'blue': blue_rgb_normalized,
    'yellow': yellow_rgb_normalized,
    'cyan': cyan_rgb_normalized,
    'magenta': magenta_rgb_normalized,
    'gray': gray_rgb_normalized,
    'purple': purple_rgb_normalized,
    'orange': orange_rgb_normalized,
    'brown': brown_rgb_normalized,
    'pink': pink_rgb_normalized,
    'teal': teal_rgb_normalized,
    'olive': olive_rgb_normalized,
}