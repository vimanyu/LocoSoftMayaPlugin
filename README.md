LocoSoftMayaPlugin
==================
LocoSoft Read Me

-------------------------------------------------------------------------------
PROJECT OVERVIEW
-------------------------------------------------------------------------------
The goal of our project is to create a Maya plug-in that will facilitate the process of creating animation sequences of deformable objects. 
We believe that our project will allow artists to quickly generate physically plausible and styled animation while being able to maintain 
some control over the overall shape of the deformable character.

Features of LocoSoft include live real time tetrahedralization for arbitrary meshes, achieved by using Tetgen. By using the VegaFEM library,
we provide FEM simulation of soft bodies with numerous parameters such Young's modulus, material density, poisson ratio, friction, restitution,
and many others for the user to tweak. There is also a choice of time integration techniques: implicit euler, implicit newmark, central differences,
symplectic euler. In order to achieve locomotion and deformations, we have provided the users ways to fix vertices through constraints and add periodic
user defined external forces to get interesting locomotion behaviors.

-------------------------------------------------------------------------------
INSTRUCTIONS FOR RUNNING
-------------------------------------------------------------------------------
If you are interested in trying out the plugin, we have provided a precompiled release version of our plugin for Maya 2013 along with all the 
related mel scripts in the "MayaDeliverables" folder. 
AELSSolverNodeTemplate.mel will need to be put into the Maya project's script folder 
for it to modify the attribute editor.

Our project has been tested with Visual Studio 2010/2012. If you would like to build our project, please locate the Maya 2013 include directory.
See 

    Project Properties -> C/C++ -> General -> Additional Include Directories, for the sample include path.
                                -> Linker -> General -> Additional Library Directories, for the sample library path.

Please set your visual studio solution configurations to Debug / Release x64.

The LocoSoft.mll (release) and LocoSoftd.mll can be found in the "mayaDeliverables" folder after a successful build.

Note that since Alembic support only available in Maya 2013. This means that Bake Geometry and Load Geometry will only work in Maya 2013.

-------------------------------------------------------------------------------
VIDEOS
-------------------------------------------------------------------------------
For a sample workflow video,
[![ScreenShot](https://raw.github.com/vimanyu/LocoSoftMayaPlugin/master/images/videoLink2.png)](http://youtu.be/_MVScjAs3-o)

For a general overview of the project,
[![ScreenShot](https://raw.github.com/vimanyu/LocoSoftMayaPlugin/master/images/videoLink3.png)](http://youtu.be/9maF-S3pmAI)

For results,
[![ScreenShot](https://raw.github.com/vimanyu/LocoSoftMayaPlugin/master/images/videoLink1.png)](http://youtu.be/jITIfyKzRcc)

-------------------------------------------------------------------------------
THIRD PARTY LIBRARIES
-------------------------------------------------------------------------------
VegaFEM: http://run.usc.edu/vega/

Tetgen: http://wias-berlin.de/software/tetgen/

GLM: http://glm.g-truc.net/0.9.4/index.html



