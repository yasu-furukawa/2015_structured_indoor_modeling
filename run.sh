#! /bin/bash
./main_process/texture/generate_texture_floorplan_cli $1
./main_process/texture/generate_texture_indoor_polygon_cli $1
./main_process/texture/generate_thumbnail_cli $1
./main_process/object_segmentation/object_segmentation_cli $1

# Hang's pipeline.

# After detection.
./main_process/object_detection/generate_object_icons_cli $1
