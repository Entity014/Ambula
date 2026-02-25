#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

class AmbulaLegIK:
    def __init__(self, model):
        """
        รับ Pinocchio model เข้ามาเพื่อดึงค่า Joint Limits โดยอัตโนมัติ
        """
        self.model = model
        
        # ค่าคงที่ Offset ขาของ Ambula
        self.x1, self.z1 = -0.015, 0.201  
        self.z2 = 0.235              
        self.L1 = np.sqrt(self.x1**2 + self.z1**2)
        self.L2 = self.z2
        self.beta = np.arctan2(-self.x1, self.z1) 

        # ดึง Limits จาก URDF อัตโนมัติ
        hip_id = self.model.getJointId("left_hip_joint")
        knee_id = self.model.getJointId("left_knee_joint")
        
        # ป้องกันกรณีหา Joint ไม่เจอ
        if hip_id >= len(self.model.joints) or knee_id >= len(self.model.joints):
            raise ValueError("Could not find left_hip_joint or left_knee_joint in URDF!")
            
        self.hip_lower = self.model.lowerPositionLimit[self.model.joints[hip_id].idx_q]
        self.hip_upper = self.model.upperPositionLimit[self.model.joints[hip_id].idx_q]
        self.knee_lower = self.model.lowerPositionLimit[self.model.joints[knee_id].idx_q]  
        self.knee_upper = self.model.upperPositionLimit[self.model.joints[knee_id].idx_q]  

        # คำนวณ Workspace Bound ล่วงหน้า
        self.L_MAX_REACHABLE = self._get_length_from_knee(self.knee_lower) 
        self.L_MIN_REACHABLE = self._get_length_from_knee(self.knee_upper) 

    def _get_length_from_knee(self, q_knee):
        """ Helper function สำหรับหาความยาว L จากมุมเข่า """
        C = np.cos(q_knee - self.beta)
        L_sq = self.L1**2 + self.L2**2 + 2 * self.L1 * self.L2 * C
        return np.sqrt(L_sq)

    def compute(self, x_des, z_des):
        """
        คำนวณหา q_hip, q_knee จากพิกัด x, z 
        (ส่งคืนมุมที่ปลอดภัยและอยู่ในขีดจำกัดเสมอ)
        """
        # 1. หาระยะกระจัดรวม (L_req)
        L_req = np.sqrt(x_des**2 + z_des**2)
        if L_req < 1e-5: L_req = 1e-5
            
        # 2. BOUND INPUT (Vector Scaling)
        L_safe = np.clip(L_req, self.L_MIN_REACHABLE, self.L_MAX_REACHABLE)
        x_safe = x_des * (L_safe / L_req)
        z_safe = z_des * (L_safe / L_req)
        
        # 3. หามุม Knee (กฎของโคไซน์)
        C_knee = (L_safe**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        C_knee = np.clip(C_knee, -1.0, 1.0)
        q_knee_math = self.beta + np.arccos(C_knee)
        q_knee = np.clip(q_knee_math, self.knee_lower, self.knee_upper)
        
        # 4. หามุม Hip (Robust Trigonometry)
        X_w = self.x1 - self.L2 * np.sin(q_knee)
        Z_w = self.z1 + self.L2 * np.cos(q_knee)
        
        denominator = X_w**2 + Z_w**2
        cos_q_hip = (X_w * x_safe + Z_w * z_safe) / denominator
        sin_q_hip = (X_w * z_safe - Z_w * x_safe) / denominator
        
        q_hip_math = np.arctan2(sin_q_hip, cos_q_hip)
        q_hip = np.clip(q_hip_math, self.hip_lower, self.hip_upper)
        
        return q_hip, q_knee