"""
步态加载器
"""
import toml
from typing import Optional, Dict, Any


class GaitLoader:
    """步态配置加载器"""

    def __init__(self, config_path: str = "./toml/usergait.toml"):
        self._config_path = config_path
        self._steps: Dict[int, Dict[str, Any]] = {}
        self._load()

    def _load(self) -> None:
        """加载步态配置"""
        try:
            data = toml.load(self._config_path)
            self._steps = {int(k): v for k, v in data.get("step", {}).items()}
            print(f">>> [步态] 加载成功，共 {len(self._steps)} 个步态")
        except Exception as e:
            print(f">>> [步态] 加载失败: {e}")
            self._steps = {}

    def get(self, gait_idx: int) -> Optional[Dict[str, Any]]:
        """获取步态配置"""
        return self._steps.get(int(gait_idx))

    def reload(self) -> None:
        """重新加载配置"""
        self._load()

    def __getitem__(self, gait_idx: int) -> Optional[Dict[str, Any]]:
        return self.get(gait_idx)

    def __contains__(self, gait_idx: int) -> bool:
        return int(gait_idx) in self._steps
