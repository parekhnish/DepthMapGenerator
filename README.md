# Depth Map Generator

## In a Nutshell

This repository contains a script to batch-process a number of ShapeNet-style model files and create depth maps for each of them, from different positions along a circular path around the object.

The repository is intended not as a standalone application to use, but more as an learning tool to understand the whole process, including camera parameter concepts and how the Blender tool works.


## Requirements

- Install [Blender](https://www.blender.org/).
- Use the `Pipfile` to download and install the required libraries. I recommend using [Pipenv](https://docs.pipenv.org/) for this.


## Running the script

Run the following line of code:
```
blender -b -P render_images.py > /dev/null
```
*(The `/dev/null` part is to redirect Blender's output logs away from `STDOUT`.The script's own logs are output to `STDERR`.)*


## Brief Explanation

### `render_images.py`
This file contains the main script. Essentially, it loops over every model contained in the `ShapeNetMeshes` folder, and creates depth maps for each object from multiple viewpoints which are defined on a circular path around the object. These depth maps are stored in the `RenderedImages` folder.

### `render_config.yaml`
This file contains settings and parameters for the rendering process. It includes camera parameters and rendering parameters.

### `shape_data.yaml`
This file contains settings for each type of ShapeNet model file present in the `ShapeNetMeshes` directory.

### `ShapeNetMeshes/`
This folder contains all the model files which are to be processed. They are stored in a format compatible with [ShapeNet](https://www.shapenet.org/)'s format.

### `RenderedImages/`
This folder will contain the rendered depth map images. The directory-tree structure is be akin to the `ShapeNetMeshes/` structure.