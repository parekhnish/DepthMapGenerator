import bpy
import numpy as np
import time
import os
import yaml
import sys

# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------

def setupScene(x_dim,y_dim):
    """
    Setting up the scene for the rendering.

    Inputs
    ----------
    x_dim, y_dim: integers
     X and Y Dimension sizes for the render.

    Outputs
    ----------
    None

    """

    for scene in bpy.data.scenes:
        scene.render.resolution_x = x_dim
        scene.render.resolution_y = y_dim
        scene.render.resolution_percentage = 100
        scene.world.horizon_color = (0.0 , 0.0 , 0.0)


# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------

def setupLamps(l_energy,l_color,l_type):
    """
    Setting up the lamps to be used to illuminate the scene
    Locations of the lamps are hardcoded.

    Inputs
    ----------
    l_energy: float
     Energy of the lamp. 1 indicates full power, but it can be more or less.
     See the Blender Lighting details for more information

    l_color: list of length 3, comprised of floats
     List containing RGB values of the color of the lamp

    l_type: string
     Type of lamp i.e. "HEMI", "SUN" etc.
     See Blender Lighting details for more information
    
    Outputs
    ----------
    None

    """

    lamp_energy = l_energy
    lamp_color = l_color
    lamp_type = l_type

    lamp_list = []


    # ADD LAMP_1
    lamp_data_1 = bpy.data.lamps.new(name="Lamp_1", type=lamp_type)
    lamp_data_1.energy = lamp_energy
    lamp_data_1.color = lamp_color

    lamp_object_1 = bpy.data.objects.new(name="Lamp_1", object_data=lamp_data_1)
    lamp_object_1.location = (2,-2,2)

    bpy.context.scene.objects.link(lamp_object_1)
    lamp_object_1.select = True
    bpy.context.scene.objects.active = lamp_object_1

    lamp_list.append(lamp_object_1)



    # ADD LAMP_2
    lamp_data_2 = bpy.data.lamps.new(name="Lamp_2", type=lamp_type)
    lamp_data_2.energy = lamp_energy
    lamp_data_2.color = lamp_color

    lamp_object_2 = bpy.data.objects.new(name="Lamp_2", object_data=lamp_data_1)
    lamp_object_2.location = (-2,2,2)

    bpy.context.scene.objects.link(lamp_object_2)
    lamp_object_2.select = True
    bpy.context.scene.objects.active = lamp_object_2

    lamp_list.append(lamp_object_2)



    return lamp_list


# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------

def setupCamera():
    """
    Setting up the camera for rendering the scene

    Inputs
    ----------
    None

    Outputs
    ----------
    camera: object
     Blender Camera object, initialised for the scene
    
    """

    bpy.ops.object.camera_add()
    camera = bpy.data.objects["Camera"]
    bpy.context.scene.camera = camera

    return camera


# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------

def makeCameraPath(camera,fov_degrees,theta_step_degrees,phi_degrees):
    """
    Defining the various positions for the camera to be placed, for creating
    rendering from different angles.

    For this code, the Y-axis is the axis of revolution for the camera.

    Inputs
    ----------
    camera: object
     The camera object that is to be moved for each render

    fov_degrees: number
     The Field-of-view parameter for the camera (in degrees)

    theta_step_degrees:
     Step parameter for defining the camera positions angles around the axis of
     revolution, like [0,360,step]

    phi_degrees: number
     The 'elevation' angle of the camera

    Outputs
    ----------
    x_cam, z_cam: List of numbers
     X- and Z-coordinate for each position of the camera

    y_cam: number
     Y-coordinate for the camera. Note that this is a single number, because
     this is the coordinate parallel to the axis of revolution, and hence 
     remains constant.

    thetas: list of numbers
     Angular coordinate for the camera at each position, measured as the angle 
     between the line connecting the camera to the origin (projected onto the
     X-Z plane), and the positive X-axis.

    distance_camera: number
     Calculated distance of the camera from the origin. Note that this is also 
     a constant, irrespective of the camera's position around the model.
     
    """

    # Define Camera Properties
    fov = fov_degrees * np.pi / 180
    camera.data.angle = fov
    distance_camera = 1.0 / (2 * np.sin(fov/2.0))

    # Define all locations of the camera when doing a sweep
    thetas = np.arange(0, 360, theta_step_degrees) * np.pi / 180.0
    phi = phi_degrees * np.pi / 180.0

    # (I.e. trasnforming spherical coordinates to Cartsian coordinates)
    x_cam = distance_camera * np.cos(thetas) * np.sin(phi)
    y_cam = distance_camera * np.cos(phi)
    z_cam = distance_camera * np.sin(thetas) * np.sin(phi)


    return (x_cam , y_cam , z_cam , thetas , distance_camera)


# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------

def setupNodes(dist_camera):
    """
    Setting up the node pipeline, which will convert the color render to a 
    depth map.

    Inputs
    ----------
    dist_camera: number
     Distance of the camera from the origin

    Outputs
    ----------
    None

    """

    bpy.context.scene.use_nodes = True
    node_tree = bpy.context.scene.node_tree

    for node in node_tree.nodes:
        node_tree.nodes.remove(node)

    # Input Node
    input_node = node_tree.nodes.new(type="CompositorNodeRLayers")

    # Color Inverter Node. Captures depth-value instead of RGB
    color_invert_node = node_tree.nodes.new(type="CompositorNodeInvert")
    color_invert_node.invert_rgb = True

    # Linear shift node. Needed to adjust the Z values so that they are between 
    # 0 and 1
    linear_shift_node = node_tree.nodes.new(type="CompositorNodeMath")
    linear_shift_node.operation = "ADD"
    linear_shift_node.inputs[0].default_value = dist_camera - 0.5

    # Output node
    output_node = node_tree.nodes.new(type="CompositorNodeComposite")

    tree_links = node_tree.links

    # Connect the nodes in a pipeline fashion
    input_color_link = tree_links.new(input_node.outputs[2]             , color_invert_node.inputs[1])
    color_shift_link =  tree_links.new(color_invert_node.outputs[0]     , linear_shift_node.inputs[1])
    shift_output_link = tree_links.new(linear_shift_node.outputs[0]     , output_node.inputs[0])


# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------

def addTrackingConstraints(camera,lamp_list,origin_obj):
    """
    Constrain the camera and the lamps to alwasy point to the object being 
    rendered.

    Inputs
    ----------
    camera: object
     The Camera object that is used for rendering

    lamp_list: list of Lamp objects
     The Lamps that are part of the scene

    origin_obj: Model Object
     The model towards which the camera and lamps are to be pointed at

    Outputs
    ----------
    None
    """ 

    # Constrain the camera
    bpy.context.scene.objects.active = camera
    bpy.ops.object.constraint_add(type='TRACK_TO')
    camera.constraints['Track To'].target = origin_obj
    camera.constraints['Track To'].track_axis = 'TRACK_NEGATIVE_Z'
    camera.constraints['Track To'].up_axis = 'UP_Y'

    # Constrain all lamps
    for lamp in lamp_list:

        bpy.context.scene.objects.active = lamp
        bpy.ops.object.constraint_add(type='TRACK_TO')
        lamp.constraints['Track To'].target = origin_obj
        lamp.constraints['Track To'].track_axis = 'TRACK_NEGATIVE_Z'
        lamp.constraints['Track To'].up_axis = 'UP_Y'


# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------

def scaleShape(shape_object):
    """
    Scale the object so that it fits inside a sphere of radius 0.5
    This is done to eliminate certain variables from other distance calculations

    Inputs
    ----------
    shape_object: Object
     The object that is to be scaled

    Outputs
    ----------
    None

    """
    farthest_vertex_distance = max([vert.co.magnitude
                                    for vert in shape_object.data.vertices])
    
    scale = 1.0 / (2 * farthest_vertex_distance)
    shape_object.scale *= scale


# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------

def orientShape(deg_rot_tuple , shape_obj):
    """
    Bring the object to the correct orientation, as defined in relation with the
    camera and the lamps.

    Throughout this code, we are taking the positive Y-axis to be the 
    UP direction. The camera's UP direction is also such that if it was
    pointing in a direction parallel to the X-Z plane, it would be the positive 
    Y-Axis. 

    Using these settings, the object must be oriented accordingly before any 
    renders.

    Inputs
    ----------
    deg_rot_tuple: Tuple of 3 numbers
     Tuple indicating the angle of rotation around the 3 (X,Y,Z) axes for the 
     model

    shape_obj; Model object
     The model that is to be oriented
    """

    x_rot = deg_rot_tuple[0] * np.pi/180.0
    y_rot = deg_rot_tuple[1] * np.pi/180.0
    z_rot = deg_rot_tuple[2] * np.pi/180.0
    
    # Arbitrary rotation is not a linear operation! To make life simpler, there are 
    # some assumptions made. These assumptions can be exploited if the order in 
    # which these rotations are applied match your expection of the final
    # output. This is where the "rotation order" argument comes in. Have a look 
    # online on this topic!
    shape_obj.rotation_mode = "XYZ"
    shape_obj.rotation_euler = (x_rot , y_rot , z_rot)

# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------


if __name__ == "__main__":

    # Initialize the config and metadata filenames
    render_config_filename = "render_config.yaml"
    shape_data_filename =    "shape_data.yaml"

    # Read the Config and MetaData Files
    with open(render_config_filename,"r") as fp:
        config_dict = yaml.safe_load(fp)

    with open(shape_data_filename,"r") as fp:
        all_shape_dicts = yaml.safe_load(fp)


    # --- STEP 1 --- Setup the Scene
    setupScene(config_dict['Scene']['x_size'] , 
                            config_dict['Scene']['y_size'])
    
    # --- STEP 2 --- Setup the Lamps
    lamp_list = setupLamps(             
        config_dict['Lamp']['lamp_energy'] ,
        tuple(config_dict['Lamp']['lamp_color']) ,
        config_dict['Lamp']['lamp_type']
    )

    # --- STEP 3 --- Setup the Camera
    camera = setupCamera()

    
    # --- STEP 4 --- Define the path the camera will take in its revolution
    (x_cam , y_cam , z_cam , thetas, dist_camera) = makeCameraPath(  
        camera , 
        config_dict['Camera']['cam_fov'] , 
        config_dict['Camera']['cam_theta_step'] , 
        config_dict['Camera']['cam_phi'] ,
    )
    
    # --- STEP 5 --- Setup the Pipeline nodes for converting a color render to
    #                A depth map
    setupNodes(dist_camera)

    # --- MAIN LOOP --- Loop over every input model type ...
    for shape_dict in all_shape_dicts['shape_data']:

        datatype_start_time = time.time()
        print("\n\n----------------------------------------" , file=sys.stderr)
        print("----------------------------------------" , file=sys.stderr)
        print("PROCESSING DATATYPE:\t" + shape_dict['folder_name'] + "\n\n" , file=sys.stderr)


        # Create and populate the folders (models) in the current model_type folder
        input_folder  = config_dict['Data']['input_root_folder']  + shape_dict['folder_name'] + '/'
        output_folder = config_dict['Data']['output_root_folder'] + shape_dict['folder_name'] + '/'

        if(not (os.path.isdir(output_folder))):
            os.mkdir(output_folder)

        FOLDER_PATHS = []

        for dirname in os.listdir(input_folder):
            FOLDER_PATHS += [os.path.join(input_folder , dirname)]


        # --- MAIN SUB-LOOP --- Loop over every model in the current model_type
        shape_counter = 0
        for obj_folder_path in FOLDER_PATHS:

            shape_time_start = time.time()

            # --- STEP 6 --- Import the model into Blender
            shape_name = obj_folder_path.split("/")[-1]
            bpy.ops.import_scene.obj(filepath=os.path.join(obj_folder_path , "model.obj"),
                                     use_smooth_groups=False)

            # Set the model as the active object. 
            # --- This is a hack; I am sure there exist better ways to do this.
            flag = False
            for o in bpy.data.objects:
                if(o.type=="MESH"):
                    if(not(flag)):
                        bpy.context.scene.objects.active = o
                        flag = True
                    
                    o.select = True
                else:
                    o.select = False

            temp_name = bpy.ops.object.join()
            shape_object = bpy.context.scene.objects.active


            shape_counter += 1
            print("RENDERING " + shape_dict['folder_name'] + " " + str(shape_counter) + "/" + str(len(FOLDER_PATHS)) + "\t" + shape_name + " ...\t" , end= "" , file=sys.stderr)

            shape_object.select = True

            # --- STEP 7 --- Orient and scale the model to conform to the
            #                coordinate system
            orientShape(tuple(shape_dict['orientation']) , shape_object)
            scaleShape(shape_object)

            # --- STEP 8 --- Constrain the cameras and lamps to point to the 
            #                model at all times
            addTrackingConstraints(camera,lamp_list,shape_object)

            num_faces = len(shape_object.data.polygons)

            # Create output folder for Model's renderings
            model_output_folder = os.path.join(output_folder, shape_name)
            if(not (os.path.isdir(model_output_folder))):
                os.mkdir(model_output_folder)
            
            # LOOPING OVER ALL THE CAMERA POSITIONS
            for i, t in enumerate(thetas):

                # --- STEP 9 --- Place the camera at its current rendering
                #                position
                camera.location = (x_cam[i], z_cam[i], y_cam)

                # --- STEP 10 --- Render the scene, save to file
                bpy.data.scenes['Scene'].render.filepath = os.path.join(model_output_folder, shape_name + "_" + str(int(i*config_dict['Camera']['cam_theta_step'])).zfill(3) + "_depthMap.png")
                bpy.ops.render.render(write_still=True)

            
            
            # --- STEP 11 --- Cleanup (for next model to be rendered)
            bpy.data.objects.remove(shape_object)

            shape_time_end = time.time()
            print("Done in " + "{0:.3f}".format(shape_time_end-shape_time_start) + " seconds", file=sys.stderr)
            sys.stderr.flush()

        # Log and exit
        datatype_end_time = time.time()
        print("\n " + shape_dict['folder_name'] + " done", file=sys.stderr)
        sys.stderr.flush()