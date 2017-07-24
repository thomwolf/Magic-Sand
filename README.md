# Magic Sand Fire
## The study project Magic Sand Fire is based on the code forked from Thomas Wolf https://github.com/thomwolf/Magic-Sand 

The aim of the project is to provide a framework visualizing the behavior of fire depending on the topography, as well as wind direction and speed. The Magic Sand Fire is addressing fire fighter, who might use the model to get specialized in recognizing risks connected to these dependencies. The Sand box allows interaction and shows how the change of wind direction or wind speed affects fire spreading. 
Particular challenges of this study project were working based on existing code, however without knowing the person who wrote the code and learning a new programming language. Further a prototype meeting the idea of the course goal had to be rapidly developed. 

Magic Sand Fire is a software which may be operated on an augmented reality sandbox like the [Augmented Reality Sandbox] (https://arsandbox.ucdavis.edu/).

## Main Features

To run the software the computer needs to be connected to a Kinect sensor and to a projector.
On the input side the Kinect sensor measures the height sand level. On the output side the projector draws colors as a function of the input values, i.e. the sand level. Thus the sandbox is being transformed into in a colorful topographical map.

A fire simulation is included with factors such as vegetation, slope, wind and general topography influencing the spread of fire. The fire is being visualized in the sandbox and leaves burnt areas on its trail.

The user can control the wind direction and its speed as well as change the topography by shaping rivers and lakes or forming mountains within the sandbox. Risk zones for fire ignition areas with south aspects and slopes of at least 10 degrees can be rendered as an overlay after the calculation. These risk zones are used to use an automated risk detection which enables to initially spawn fires. 
Additionally fires may be placed manually anywhere within the extent of the sandbox. A wind arrow indicates the wind as it is dynamically rotated and scaled according to the settings of wind direction and speed. The simulation can be paused and resumed at any time needed.

Specified information about the dependencies and a quick start for editing the source code can be received from the [readme file] (https://github.com/thomwolf/Magic-Sand/blob/master/README.md) by Thomas Wolf.
Detailed description about the basic code can be found there as well.