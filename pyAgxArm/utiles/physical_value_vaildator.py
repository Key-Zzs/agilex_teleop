import math

class PhysicalValueValidator:
    REF_MAX_ANGLE = 2 * math.pi
    REF_MIN_ANGLE = -REF_MAX_ANGLE

    REF_MAX_POS = 2.0      # typical arm length (m)
    REF_MIN_POS = -REF_MAX_POS

    REF_MAX_ANG_VEL = 10.0
    REF_MIN_ANG_VEL = -REF_MAX_ANG_VEL

    REF_MAX_ANG_ACC = 50.0
    REF_MIN_ANG_ACC = -REF_MAX_ANG_ACC

    @staticmethod
    def _validate_numeric(value: float) -> None:
        if not isinstance(value, (int, float)):
            raise TypeError(f"Value must be int or float, got {type(value)}")
        if math.isnan(value) or math.isinf(value):
            raise ValueError("Value cannot be NaN or Inf")

    @staticmethod
    def _validate_min_max(min_val: float, max_val: float) -> None:
        if min_val > max_val:
            raise ValueError(
                f"Invalid limit range: min_val ({min_val}) > max_val ({max_val})"
            )

    @staticmethod
    def _validate_limits_structure(
        values: list,
        limits: list,
    ) -> None:
        if not isinstance(limits, list):
            raise TypeError("limits must be list")

        if len(values) != len(limits):
            raise ValueError(
                f"Length mismatch: values({len(values)}) != limits({len(limits)})"
            )

        for i, lim in enumerate(limits):
            if not isinstance(lim, (list, tuple)):
                raise TypeError(f"limits[{i}] must be list or tuple")

            if len(lim) != 2:
                raise ValueError(
                    f"limits[{i}] length must be 2, got {len(lim)}"
                )

            min_val, max_val = lim
            PhysicalValueValidator._validate_min_max(min_val, max_val)

    @staticmethod
    def clamp_value(
        value: float,
        min_val: float,
        max_val: float,
    ) -> float:
        PhysicalValueValidator._validate_numeric(value)
        PhysicalValueValidator._validate_min_max(min_val, max_val)
        return max(min(value, max_val), min_val)

    @staticmethod
    def is_within_limit(
        value: float,
        min_val: float,
        max_val: float,
        tolerance: float = 0.0,
    ) -> bool:
        PhysicalValueValidator._validate_numeric(value)
        PhysicalValueValidator._validate_min_max(min_val, max_val)
        return (min_val - tolerance) <= value <= (max_val + tolerance)

    @staticmethod
    def clamp_joints(joints:list, joints_limit:list=[]):
        # 判断 limit 是否为空
        if not joints_limit:
            min_val = PhysicalValueValidator.REF_MIN_ANGLE
            max_val = PhysicalValueValidator.REF_MAX_ANGLE
            return [
                PhysicalValueValidator.clamp_value(j, min_val, max_val)
                for j in joints
            ]

        # 校验 limit 结构
        PhysicalValueValidator._validate_limits_structure(joints, joints_limit)

        # clamp
        clamped = []
        for j, (min_val, max_val) in zip(joints, joints_limit):
            clamped.append(
                PhysicalValueValidator.clamp_value(j, min_val, max_val)
            )

        return clamped

    @staticmethod
    def clamp_6pose(pose:list, pose_limit:list=[]):
        if not isinstance(pose, list):
            raise TypeError("pose must be list")

        if len(pose) != 6:
            raise ValueError(
                f"pose length must be 6 [x,y,z,r,p,y], got {len(pose)}"
            )
        x, y, z, roll, pitch, yaw = pose
        if not pose_limit:
            roll_min, roll_max = -math.pi, math.pi
            yaw_min, yaw_max = -math.pi, math.pi
            pitch_min, pitch_max = -math.pi / 2.0, math.pi / 2.0
            roll = PhysicalValueValidator.clamp_value(roll, roll_min, roll_max)
            pitch = PhysicalValueValidator.clamp_value(pitch, pitch_min, pitch_max)
            yaw = PhysicalValueValidator.clamp_value(yaw, yaw_min, yaw_max)

        # 5. 返回新 pose（前三位不动）
        return [x, y, z, roll, pitch, yaw]

    @staticmethod
    def is_joints_vaild(joints:list) -> bool:
        min_val = PhysicalValueValidator.REF_MIN_ANGLE
        max_val = PhysicalValueValidator.REF_MAX_ANGLE
        for i, j in enumerate(joints):
            if not PhysicalValueValidator.is_within_limit(
                j, min_val, max_val, 0.0
            ):
                return False
        return True

    @staticmethod
    def is_6pose_vaild(pose:list):
        if not isinstance(pose, list):
            raise TypeError("pose must be list")

        if len(pose) != 6:
            raise ValueError(
                f"pose length must be 6 [x,y,z,r,p,y], got {len(pose)}"
            )
        x, y, z, roll, pitch, yaw = pose
        roll_min, roll_max = -math.pi, math.pi
        pitch_min, pitch_max = -math.pi / 2.0, math.pi / 2.0
        yaw_min, yaw_max = -math.pi, math.pi
        
        if not PhysicalValueValidator.is_within_limit(
                roll, roll_min, roll_max, 0.0
            ):
            return False
        if not PhysicalValueValidator.is_within_limit(
                pitch, pitch_min, pitch_max, 0.0
            ):
            return False
        if not PhysicalValueValidator.is_within_limit(
                yaw, yaw_min, yaw_max, 0.0
            ):
            return False
        return True
        