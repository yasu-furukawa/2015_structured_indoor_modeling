#! /bin/bash
./texture/generate_texture_floorplan_cli $1
./texture/generate_texture_indoor_polygon_cli $1
./texture/generate_thumbnail_cli $1
./object_segmentation/object_segmentation_cli $1

# After detection.
./object_detection/generate_object_icons_cli $1
