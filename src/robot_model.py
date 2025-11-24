import numpy as np
import mujoco
import mujoco.viewer

class RobotModel:
    def __init__(self,model_path,tcp_site_name='attachment_site'):
        """
        tcp_site_name: 请指定你 XML 中真正的 TCP site 名称
        你的模型末端 site 是 attachment_site 或 force_sensor_site
        """
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.tcp_site = tcp_site_name
        # 正确获取关节信息
        self.joint_names = []
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                self.joint_names.append(name)
        
        self.nq = self.model.nq
        self.nu = self.model.nu
        
        print(f"[INFO] 加载的关节: {self.joint_names}")
        print(f"[INFO] 关节数量: {self.model.njnt}")
        print(f"[INFO] nq = {self.nq}, nu = {self.nu}")
        # 打印所有 site，用于检查 TCP 命名
        print("\n[INFO] 模型包含的 Site:")
        for i in range(self.model.nsite):
            print("   ", mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SITE, i))

        # 预取 TCP site ID（提前报错，不要悄悄 fallback）
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, tcp_site_name)
        if sid < 0:
            raise ValueError(f"给定的 tcp_site_name='{tcp_site_name}' 不存在于模型中！")
        self.tcp_sid = sid

    def fk(self,q):
        """返回末端位置 pos(3,) 和旋转矩阵 rot(3,3)"""
        if len(q) != self.model.nq:
            raise ValueError(f"配置 q 长度错误: {len(q)} != {self.model.nq}")

        self.data.qpos[:] = q
        mujoco.mj_forward(self.model, self.data)  # 正确的 FK 更新

        pos = self.data.site_xpos[self.tcp_sid].copy()
        mat = self.data.site_xmat[self.tcp_sid].reshape(3, 3).copy()
        return pos, mat
    
    def random_joint_sample(self):
        """sample random q within joint limits (use qpos0 +/- range from model)"""
        q = np.zeros(self.model.nq)
    
        # 使用已经存储的关节名称列表 self.joint_names
        for i, joint_name in enumerate(self.joint_names):
            jnt_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            
            # 检查关节限制是否可用
            if self.model.jnt_range is not None and self.model.jnt_range.shape[0] > jnt_id:
                low, high = self.model.jnt_range[jnt_id]
                q[i] = np.random.uniform(low=low, high=high)
            else:
                # 如果没有关节限制，使用安全范围
                q[i] = np.random.uniform(-1.0, 1.0)
        
        return q