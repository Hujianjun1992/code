<?xml version="1.0"?>
<robot name="smartcar">
 <link name="base_link">
   <visual>
     <geometry>
       <box size="0.36 .47 .07"/>
     </geometry>
     <origin rpy="0 0 -1.57075" xyz="0 0 0"/>
     <material name="blue">
        <color rgba="0 0 .8 1"/>
     </material>
   </visual>
 </link>


 <link name="right_front_wheel">
    <visual>
      <geometry>
        <cylinder length=".04" radius="0.0762"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="right_front_wheel_joint" type="continuous">
    <axis xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="right_front_wheel"/>
    <origin rpy="0 1.57075 0" xyz="0.255 0.18 .0"/>
    <limit effort="100" velocity="100"/>
    <joint_properties damping="0.0" friction="0.0"/>
  </joint>

  <link name="right_back_wheel">
    <visual>
      <geometry>
        <cylinder length=".04" radius="0.0762"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="right_back_wheel_joint" type="continuous">
    <axis xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="right_back_wheel"/>
    <origin rpy="0 1.57075 0" xyz="0.255 -0.18 -0.0"/>
    <limit effort="100" velocity="100"/>
    <joint_properties damping="0.0" friction="0.0"/>
 </joint>

 <link name="left_front_wheel">
    <visual>
      <geometry>
        <cylinder length=".04" radius="0.0762"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="left_front_wheel_joint" type="continuous">
    <axis xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="left_front_wheel"/>
    <origin rpy="0 1.57075 0" xyz="-0.255 0.18 -0.0"/>
    <limit effort="100" velocity="100"/>
    <joint_properties damping="0.0" friction="0.0"/>
  </joint>

  <link name="left_back_wheel">
    <visual>
      <geometry>
        <cylinder length=".04" radius="0.0762"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="left_back_wheel_joint" type="continuous">
    <axis xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="left_back_wheel"/>
    <origin rpy="0 1.57075 0" xyz="-0.255 -0.18 -0.0"/>
    <limit effort="100" velocity="100"/>
    <joint_properties damping="0.0" friction="0.0"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <box size=".08 .08 .057"/>
      </geometry>
      <material name="white">
          <color rgba="1 1 1 1"/>
      </material>
    </visual>
  </link>

  <joint name="tobox" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0.14 0.061"/>
  </joint>
</robot>



