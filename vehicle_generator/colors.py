"""Gazebo 차량 10대용 고대비 원색 지정. 명칭·순서는 고정입니다."""
from __future__ import annotations
import re

AI_COLORS = {
    "AI-01": ("노랑", (1.00, 0.88, 0.03)),
    "AI-02": ("빨강", (1.00, 0.09, 0.08)),
    "AI-03": ("파랑", (0.12, 0.35, 1.00)),
    "AI-04": ("초록", (0.05, 0.95, 0.16)),
    "AI-05": ("주황", (1.00, 0.46, 0.02)),
    "AI-06": ("자홍", (1.00, 0.06, 0.82)),
    "AI-07": ("청록", (0.04, 0.91, 1.00)),
    "AI-08": ("보라", (0.70, 0.24, 1.00)),
    "AI-09": ("연두", (0.68, 1.00, 0.03)),
    "AI-10": ("분홍", (1.00, 0.35, 0.58)),
}
IDENTIFIER = re.compile(r"^urrc_ai_(0[1-9]|10)$")

def color_for_vehicle(name):
    """비-AI 실험차는 기존 검정색을 유지하고 10대만 식별색 적용."""
    if IDENTIFIER.fullmatch(name):
        return AI_COLORS["AI-"+name.rsplit("_",1)[-1]][1]
    return (0.028, 0.028, 0.033)

def name_for_vehicle(name):
    if IDENTIFIER.fullmatch(name):
        return AI_COLORS["AI-"+name.rsplit("_",1)[-1]][0]
    return "검정"

def material_rgb(rgb):
    return " ".join(f"{x:.3f}" for x in rgb)+" 1"
