# CraftUI
A touch interface for shopping windows and similiar glass surfaces. A Kinect is used to sense touch events on the surface. In its core, CraftUI features a special and easy to use auto calibration process where the UI is defined by colored paper templates which are taped to the glass surface. Designing and setting up an installation is very fast, straight forward and first and foremost user friendly.

A simple test installation can be seen in this [Video](http://devlol.soup.io/post/539538698/CraftUI-in-Action?sessid=a90e3c28f8a15868ba9f9264b2b3117f). Calibration and configuration was done in about 5 Minutes without any hassle.

#### Tools and Executables
* craftui - The main application.
* craftui_calib_ui - Define and calibrate UI elements from paper templates.
* craftui_calib_template - Calibrates the color descriptor for a element type.
* craftui_init_config - Creates the default configuration XML file.
* craftui_name_element - Tool to select and name UI elements (assign an ID). 

#### IPC
Craftui broadcasts UI events on a ZeroMQ socket. The event protcol is defined via protobuf. In the ./clients/ directory there are simple python clients which process these events, execute actions or forward the events to other channels like MQTT.

#### Calibration Workflow and Setup.
1. Use **craftui_init_config** to create a default config. Now edit the size of your calibration sheet, default is A4.
2. (optional) Calibrate a color descriptor for each ElementType (slider, button, calibrationtemplate) to identify the the in the subsequent calibration step. This is done with **craftui_calib_template**. Tape a colored sheet to the glass surface. The border of the sheet must have a distance of about 10cm to any other visible object or window frame. Press [c] to capture a frame and select the ElementType to assign the color to. :w
3. Cut out the shape of your UI elements (buttons) and tape them to the glass. As before, tape the calibration sheet to the surface. It is used to define the plane and coordinate frame of the window. The Kinec must be placed in a way that the sheet and all elements are visible. Facing the Kinect at an angle to the glass reduces problems with reflection. Make sure that the Kinect is mounted and stable since geometric calibration is done in the next step.
4. Use **craftui_calib_ui** to to define UI elements. Again, press [c] to capture a frame to calibrate from. After the window is closed [Esc] all elements are stored to the config file.
5. Now draw the UI with whiteboard markers, window colors or even other paper cutouts and prints. Mark the outlines of the buttons. It is adviced, tho not necessary, to encircle the paper templates at a short distance (1cm). Remove all paper templates.
5. All elements are currently unnamed. To assign an ID, use the tool **craftui_name_element**. Start the tool and hit an UI element. The tool prints a query for a new ID. Repeat this until all elements are named. This IDs are part of the events which are fired by the main application.
6. Run the main application **craftui**. Keep the glass surface free (don't touch it) in the first seconds. CraftUI does an autocalibration at this point. After a few seconds the UI is ready to use.
7. Attach a client to craftui. Sample clients written in python are included in the ./clients/ directory. Run **craftui_print_event.py** and test the UI. MQTT examples are also included.
8. Fine tune the distance and threshold settings for each element. This must be done manually in the config file at this point. However, the default settings usually work just fine.
9. Enjoy! Attach hardware, sofware and whatnot to CraftUI.
