import mujoco
from mujoco.viewer import launch    



# model with right arm
# model = mujoco.MjModel.from_xml_path("myo_sim/arm/myoarm.xml")



# model with muscles, will be the final build, don't update until truetest is finished
# model = mujoco.MjModel.from_xml_path("myo_sim/arm/leftarm.xml") 


# model with both arms, use for testing, doesn't have muscles
model = mujoco.MjModel.from_xml_path("myo_sim/arm/truetest.xml") 


data = mujoco.MjData(model)

    

# Simulation duration
# Launch the viewer
with launch(model, data) as viewer:
    print(".")
    