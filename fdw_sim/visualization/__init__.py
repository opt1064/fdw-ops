"""USD 시각화 모듈 — Isaac Sim Level 2 통합.

PoC-1의 추상 셀(MaterialCell, WeldingCell, InspectionCell)을 Isaac Sim의
USD 스테이지 위에 시각적으로 표현한다. 각 셀은 다음을 가진다:

    * 작업대(workbench)            : 회색 박스
    * 셀 라벨                      : 텍스트 (선택)
    * 입력 버퍼/출력 버퍼 마커     : 컬러 슬랫
    * 셀 고유 시각요소             : 로봇팔, 카메라, AMR, 랙 등

이 모듈은 mode="isaac" 일 때만 활성화되며, mode="discrete"에서는
import조차 일어나지 않는다(SimulationManager에서 lazy import).
"""
from fdw_sim.visualization.scene_builder import SceneBuilder, SceneConfig

__all__ = ["SceneBuilder", "SceneConfig"]
