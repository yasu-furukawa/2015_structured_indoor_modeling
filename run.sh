#! /bin/bash
# Textures and images for viewer.
./main_process/texture/generate_texture_floorplan_cli $1
./main_process/texture/generate_texture_indoor_polygon_cli $1
./main_process/texture/generate_thumbnail_cli $1

# Objects.
./main_process/object_segmentation/object_segmentation_cli $1
./main_process/object_hole_filling/Object_hole_filling $1
./main_process/object_detection/generate_object_icons_cli $1

# Misc.
./post_process/collada/indoor_polygon_to_dae_cli $1
./post_process/evaluation/evaluate_cli $1
