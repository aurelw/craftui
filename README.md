# CraftUI
A touch interface for shopping windows and similiar glass surfaces. A Kinect is used to sense touch events on the surface. In its core, CraftUI features a special and easy to use auto calibration process where the UI is defined by colored paper templates which are taped to the glass surface. Designing and setting up an installation is very fast, straight forward and first and foremost user friendl.

A simple test installation can be seen in this [Video](http://devlol.soup.io/post/539538698/CraftUI-in-Action?sessid=a90e3c28f8a15868ba9f9264b2b3117f). Calibration and configuration was done in about 5 Minutes without any hassle.

#### Tools and Executables
* craftui - The main application.
* craftui_calib_ui - Define and calibrate UI elements from paper templates.
* craftui_calib_template - Calibrates the color descriptor for a element type.
* craftui_init_config - Creates the default configuration XML file.

#### IPC
Craftui broadcasts UI events on a ZeroMQ socket. The event protcol is defined via protobuf. In the ./clients/ directory there are simple python clients which process these events, execute actions or forward the events to other channels like MQTT. 
