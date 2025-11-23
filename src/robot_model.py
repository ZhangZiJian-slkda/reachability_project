import numpy as np
import mujoco
import mujoco.viewer

class RobotModel:
    def __init__(self,model_path,tcp_site_name='tcp'):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.tcp_site = tcp_site_name

        # #joint info
        # # self.joint_names = [name for name in self.model.joint_names]
        # self.joint_names = [self.model.joint(i).name for i in range(self.model.njnt)]
        # self.nq = self.model.nq
        # self.nu = self.model.nu

        # print(f"关节名称: {self.joint_names}")
        # print(f"关节数量 (njnt): {self.model.njnt}")
        # print(f"广义坐标数量 (nq): {self.nq}")
        # print(f"控制输入数量 (nu): {self.nu}")


        #######
                # 正确获取关节信息
        self.joint_names = []
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                self.joint_names.append(name)
        
        self.nq = self.model.nq
        self.nu = self.model.nu
        
        print(f"加载的关节: {self.joint_names}")
        print(f"关节数量: {self.model.njnt}")
        print(f"广义坐标数量: {self.nq}")
        print(f"控制输入数量: {self.nu}")
        
        # 打印所有站点信息用于调试
        print("模型中的站点:")
        for i in range(self.model.nsite):
            site_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SITE, i)
            if site_name:
                print(f"  站点 {i}: {site_name}")

    def fk(self,q):
        """
        q: ndarray with size == model.nq (generalized positions)
        returns: (pos(3,), rot_matrix(3,3))
        """
        if len(q) != self.model.nq:
            raise ValueError("q length mismatch")
        
        #set positions
        self.data.qpos[:] = q
        mujoco.mj_step(self.model,self.data,0)# forward kinematics update (0 steps)
        # get tcp site pos and orientation (as rotation matrix)
        try:
            sid = self.model.site(self.tcp_site).id
        except Exception:
            sid = 0

        pos = self.data.site_xpos[sid].copy()
        mat = self.data.site_xmat[sid].reshape(3,3).copy()
        return pos,mat
    
    def random_joint_sample(self):
        """sample random q within joint limits (use qpos0 +/- range from model)"""
        # For simplicity, sample within model.jnt_range if available, else small range
        q = np.zeros(self.model.nq)
        #If joint limits exist (depends on model),use them:
        for i ,joint in enumerate(self.model.joint_names):
            jnt_id = self.model.joint_name2id(joint)
            # model.jnt_range stores [lower, upper] pairs if available
            if self.model.jnt_range is not None and self.model.jnt_range.shape[0] > jnt_id:
                low,high = self.model.jnt_range[jnt_id]
                q[i] = np.random.uniform(low=low,high=high)
            else:
                q[i] = np.random.uniform(-1.0,1.0)
        return q