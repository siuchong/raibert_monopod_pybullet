# raibert_monopod_pybullet

An implementation of Marc Raiberts Hopping control algorithm for a One legged hopping robot on pyBullet.

hopper.urdf was generated from hopper.urdf.xacro using ROS xacro tools

The hopper is modeled to have two actuators at the hip. One swivels the inner frame(red) relative to the outer frame(white), the other swivels the leg(yellow) relative to the inner frame(red). The leg is modeled to be similar to a spring which provides thrust when leg tip is in contact with the ground.

The simulation can be seen here https://youtu.be/ePpoTIP6aXw
