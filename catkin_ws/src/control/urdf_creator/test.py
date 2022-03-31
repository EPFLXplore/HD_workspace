from compiler import *
import sys


class Test(RobotURDF):
    def __init__(self):
        super().__init__("bonk")
        self.comment("="*10 + "CONSANTS" + "="*10)
        self.box_link_dim("base_link", (0.3,0.3,0.1))
        self.box_link_origin("base_link",
                             btm_shift=(0,0,0),
                             top_shift=(0,0,self.base_link_dim_z/2))

        self.box_link_dim("link1", (0.083,0.156,0.140))
        self.box_link_origin("link1",
                             btm_shift=(0,0,self.link1_dim_z/2),
                             top_shift=(0,0,self.link1_dim_z/2-0.05))

        self.box_link_dim("link2", (0.096,0.062,0.425))
        self.box_link_origin("link2",
                             btm_shift=(0,0,-(0.048-self.link2_dim_z/2)),
                             top_shift=(0.03224-self.link2_dim_x/2,0,self.link2_dim_z/2-0.032224))

        self.box_link_dim("link3", (0.092, 0.112, 0.334))
        self.box_link_origin("link3",
                             btm_shift=(0.01615-self.link3_dim_x/2,0,-(0.096328-self.link3_dim_z/2)),
                             top_shift=(-(0.0415-self.link3_dim_x/2),0,self.link3_dim_z/2))

        self.box_link_dim("link4", (0.080, 0.173, 0.120))
        self.box_link_origin("link4",
                             btm_shift=(0,0,self.link4_dim_z/2),
                             top_shift=(0,0,self.link4_dim_z/2-0.029))
        
        self.box_link_dim("link5", (0.047, 0.056, 0.08625))
        self.box_link_origin("link5",
                             btm_shift=(0,0,self.link5_dim_z/2-0.0235),
                             top_shift=(0,0,self.link5_dim_z/2))

        self.box_link_dim("gripper_base", (0.1196, 0.1105, 0.062))
        self.box_link_origin("gripper_base",
                             btm_shift=(self.gripper_base_dim_x/2-0.03,0,self.gripper_base_dim_z/2),
                             top_shift=(0,0,0))
        names = ["base_link", "link1", "link2", "link3", "link4", "link5", "gripper_base"]

        self.blank_lines(1)
        self.separation()
        self.blank_lines(4)
        self.comment("="*10 + "LINKS" + "="*10)

        for name in names:
            self.classic_link(name)

        self.blank_lines(1)
        self.separation()
        self.blank_lines(4)
        self.comment("="*10 + "JOINTS" + "="*10)

        self.classic_revolute_joint("joint1", parent="base_link", child="link1", axis="0 0 1")
        self.classic_revolute_joint("joint2", parent="link1", child="link2", axis="0 1 0")
        self.classic_revolute_joint("joint3", parent="link2", child="link3", axis="0 1 0")
        self.classic_revolute_joint("joint4", parent="link3", child="link4", axis="0 0 1")
        self.classic_revolute_joint("joint5", parent="link4", child="link5", axis="0 1 0")
        self.classic_revolute_joint("joint6", parent="link5", child="gripper_base", axis="0 0 1")

    def box_link_dim(self, name, dim):
        self.blank_lines(3)
        self.comment(name + " constants")
        self.def_constant(name+"_dim_x", dim[0])
        self.def_constant(name+"_dim_y", dim[1])
        self.def_constant(name+"_dim_z", dim[2])
        self.def_constant(name+"_dim", Arithmetic.conc(dim))

    def box_link_origin(self, name, btm_shift, top_shift):

        self.def_constant(name+"_btm_shift_x", btm_shift[0])
        self.def_constant(name+"_btm_shift_y", btm_shift[1])
        self.def_constant(name+"_btm_shift_z", btm_shift[2])

        self.def_constant(name+"_top_shift_x", top_shift[0])
        self.def_constant(name+"_top_shift_y", top_shift[1])
        self.def_constant(name+"_top_shift_z", top_shift[2])

        self.def_constant(name+"_origin", Arithmetic.conc(btm_shift))
        self.def_constant(name+"_attach_point", Arithmetic.conc([top_shift[i]+btm_shift[i] for i in range(3)]))

    def classic_link(self, name):
        self.blank_lines(2)
        self.comment(name)
        geometry = Box(getattr(self, name+"_dim"))
        origin = Origin(getattr(self, name+"_origin"))
        visual = LinkVisual(geometry, origin)
        collision = LinkCollision(geometry, origin)
        link = Link(name, visual, collision)
        self.add_link(link)

    def classic_revolute_joint(self, name, parent, child, axis, limit=None):
        self.blank_lines(2)
        self.comment(name)
        parent_s = str(parent)
        origin = Origin(xyz=self.names[parent_s+"_attach_point"])
        if limit is None:
            limit = Limit(1000, 0.5, lower=-PI, upper=PI)
        joint = RevoluteJoint(name, parent, child, origin, axis, limit)
        self.add_joint(joint)


if __name__ == "__main__":
    urdf = Test()
    urdf.compile(sys.argv[1])
    #roslaunch urdf_tutorial display.launch model:=test.urdf.xacro
