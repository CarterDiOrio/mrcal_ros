# mrcal_ros
## This package is under active development and I will remove this message when the first version becomes finalized.

Portfolio Post: www.cdiorio.dev/projects/camera-calibration/#mrcal

MrCal (https://mrcal.secretsauce.net/) a rich Splined-Stereographic model that can model not only standard lenses, but narrow, and extremely wide FoV lenses well. However ROS2 has a limited number of camera models it supports. The intent is for this package to provide a bridge based on the MrCal recipe [reprojecting to a lean model](https://mrcal.secretsauce.net/recipes). 

Currently contains one component node that takes in a rich and lean (from and to) MrCal camera model files. 

What operations the node performs depends on the files passed:

- If your rich model is directly supported by ROS2, you do not need to pass a lean model. The Node will republish the image with a corresponding intrinsics topic.

- If your rich model is not directly supported by ROS2, you need to pass a lean model. The node then reprojects from the rich to the lean model that is supported by ROS2 and publishes an image with the lean model intrinsics.

Models Supported By ROS2:
- plumb_bob <-> MrCal LENSMODEL_OPENCV5
- rational_polynomial <-> MrCal LENSMODEL_OPENCV8

## Non-Rosdep Dependencies:

- MrCal: Not currently in rosdep, can be installed with apt following https://mrcal.secretsauce.net/install.html

