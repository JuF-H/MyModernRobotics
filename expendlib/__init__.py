"""扩展数学库：雅可比矩阵转换、可操作度指标、齐次变换工具。"""

from .core import (
    is_within_limit,
    clip_q,
    wrap_to_limit,
    Js2Jg,
    Jb2Jg,
    Js2Jb,
    Jb2Js,
    A_of_r,
    A_of_l,
    Jb2Ja_r,
    Js2Ja_r,
    mu_1_axis_ratio,
    mu_2_condition_number,
    mu_3_volume,
)
from .compute import (
    translation_matrix,
    rotation_matrix_axis,
    rotate_about_point,
)

__all__ = [
    # 关节限位
    "is_within_limit",
    "clip_q",
    "wrap_to_limit",
    # 雅可比转换
    "Js2Jg",
    "Jb2Jg",
    "Js2Jb",
    "Jb2Js",
    "A_of_r",
    "A_of_l",
    "Jb2Ja_r",
    "Js2Ja_r",
    # 可操作度
    "mu_1_axis_ratio",
    "mu_2_condition_number",
    "mu_3_volume",
    # 齐次变换
    "translation_matrix",
    "rotation_matrix_axis",
    "rotate_about_point",
]
