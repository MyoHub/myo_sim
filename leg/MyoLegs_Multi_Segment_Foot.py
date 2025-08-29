# -*- coding: utf-8 -*-
"""
Created on Mon May 26 09:04:23 2025

@author: radsi
"""

import mujoco
from mujoco.viewer import launch

model = mujoco.MjModel.from_xml_string("""

<mujoco model="MyoSuite's MyoLeg Model">
<!--  =================================================
    Copyright 2020 Vikash Kumar, Vittorio Caggiano, Huawei Wang
    Model   :: Myo Hand (MuJoCoV2.0)
    Author  :: Vikash Kumar (vikashplus@gmail.com), Vittorio Caggiano, Huawei Wang
    source  :: https://github.com/vikashplus
    License :: Under Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.
======================================================  -->
<include file="../scene/myosuite_scene_noPedestal.xml"/>
<include file="../torso/assets/myotorso_rigid_assets.xml"/>
<include file="../leg/assets/myolegs_assets.xml"/>
<include file="../leg/assets/myolegs_tendon.xml"/>
<include file="../leg/assets/myolegs_muscle.xml"/>
<compiler angle="radian" meshdir=".." texturedir=".."/>
<asset>
<hfield name="terrain" size="7 7 1 0.001" nrow="100" ncol="100"/>
</asset>
<worldbody>
<geom name="terrain" type="hfield" hfield="terrain" pos="0 0 -0.005" material="matfloor" conaffinity="1" contype="1" rgba="1 1 1 0"/>
<site name="pelvis_target" size="0.02" pos="0 0 .92" group="4"/>
<body name="root" pos="0 0 1" euler="0 0 -1.57">
<freejoint name="root"/>
<inertial mass="1.0" pos="0 0 0" diaginertia="0.01 0.01 0.01"/>
<body name="test">
<joint name="pelvis_tx" range="-5 5" limited="false" user="0.369999999988462" ref="0" axis="1 0 0" type="slide" damping="0" stiffness="0"/>
<joint name="pelvis_ty" range="-1 2" limited="false" user="0.9999999999157" ref="0" axis="0 1 0" type="slide" damping="0" stiffness="0"/>
<joint name="pelvis_tz" range="-5 5" limited="false" user="0.555999999984879" ref="0" axis="0 0 1" type="slide" damping="0" stiffness="0"/>
<joint name="pelvis_rotation" range="-0.2618 6.283" limited="true" user="0.0" ref="0" axis="0 1 0" type="hinge" damping="0" stiffness="0"/>
<joint name="pelvis_tilt" range="-3.14 3.14" limited="true" user="0.0" ref="0" axis="0 0 1" type="hinge" damping="0" stiffness="0"/>
<joint name="pelvis_list" range="-3.14 3.14" limited="true" user="0.0" ref="0" axis="1 0 0" type="hinge" damping="0" stiffness="0"/>
                
<include file="../torso/assets/myotorso_rigid_chain.xml"/>   
<include file="../leg/assets/myolegs_chain.xml"/>





</body>
</body>
</worldbody>
<!-- 
<keyframe>

<key qpos="0 0 .92 0.707388 0 0 -0.706825 0.161153 -0.0279385 -0.041886 0.00247908 0.00101098 0.461137 0.0275069 0.136817 0.334 -0.00117055 -0.000125295 -0.0302192 0.0395202 -0.194029 0.161153 -0.0279385 -0.041886 0.00247908 0.00101098 0.461137 0.0275069 0.136817 0.334 -0.00117055 -0.000125295 -0.0302192 0.0395202 -0.194029"/>
<key qpos="0 0 .9 0.707388 0 0 -0.706825 0.405648 -0.020957 -0.118677 0.0039054 0.00122326 0.7329 0.0102961 0.215496 0.40143 -0.006982 -0.02618 -0.03738 0.0080579 -0.87272 0.405648 -0.020957 -0.118677 0.0039054 0.00122326 0.7329 0.0102961 0.215496 0.40143 -0.006982 -0.02618 -0.03738 0.0080579 -0.87272"/>
<key qpos="0 0 1.0 0.707388 0 0 -0.706825 -0.2326 -0.0279385 -0.041886 0.00247908 0.00101098 1.227 0.0275069 0.136817 0.1672 -0.00117055 -0.000125295 -0.0302192 0.0395202 -0.194029 -0.1652 -0.0279385 -0.041886 0.00247908 0.0010198 0.0888 0.0275069 0.136817 -0.019 -0.00117055 -0.000125295 -0.0302192 0.0395202 -0.194029" qvel="0 -1.5 0 0 0 0 4.9066 0 0 0 0 -3.597 0 0 0.633 0 0 0 0 0 0.175 0 0 0 0 0.175 0 0 0.988 0 0 0 0 0"/>
<key qpos="0 0 1.0 0.707388 0 0 -0.706825 -0.1652 -0.0279385 -0.041886 0.00247908 0.00101098 0.0888 0.0275069 0.136817 -0.019 -0.00117055 -0.000125295 -0.0302192 0.0395202 -0.194029 -0.2326 -0.0279385 -0.041886 0.00247908 0.0010198 1.227 0.0275069 0.136817 0.1672 -0.00117055 -0.000125295 -0.0302192 0.0395202 -0.194029" qvel="0 -1.5 0 0 0 0 -0.576 0 0 0 0 0.175 0 0 0.988 0 0 0 0 0 4.9066 0 0 0 0 -3.597 0 0 0.633 0 0 0 0 0"/>
</keyframe>
-->
</mujoco>

""")




launch(model)
