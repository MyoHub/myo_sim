import mujoco
from mujoco.viewer import launch

# Load the MuJoCo model
# model = mujoco.MjModel.from_xml_path("myo_sim/torso/myotorso_rigid.xml")
# model = mujoco.MjModel.from_xml_path("myo_sim/leg/myolegs_abdomen.xml")
# model = mujoco.MjModel.from_xml_path("myo_sim/arm/leftarm.xml") 
#model = mujoco.MjModel.from_xml_path("arm/leftarm.xml")
 




#model = mujoco.MjModel.from_xml_path("arm/myoarm.xml")
model = mujoco.MjModel.from_xml_path("arm/myoarm.xml")
#model = mujoco.MjModel.from_xml_path("arm/assets/myoarm_body.xml")


#model = mujoco.MjModel.from_xml_path("arm/leftarmtest.xml") 

data = mujoco.MjData(model)

    

# Simulation duration
# Launch the viewer
with launch(model, data) as viewer:
    print(".")
    