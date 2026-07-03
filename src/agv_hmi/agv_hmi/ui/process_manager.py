"""
process_manager.py — Managed ROS2 launch helper v4.2.6

Fix v4.2.3:
  - force_cleanup dùng pkill -x (exact process name) thay vì pkill -f
    để tránh kill nhầm Gazebo/gz_sim hoặc simulator đang chạy song song.

Fix v4.2.4:
  - start(extra_args=...) cho phép truyền launch arguments động,
    ví dụ map:=/path/to/map.yaml.
  - current_extra_args lưu tham số của lần start thành công gần nhất.

Fix v4.2.6:
  - Chuẩn hoá so sánh extra_args theo list[str].
  - stop() luôn clear current_extra_args.
  - Giữ API tương thích với MainWindow._ensure_nav_running().
"""
import os
import signal
import subprocess
from typing import Optional


class ManagedLaunch:
    """
    Start/stop ros2 launch bằng process group riêng.

    cmd:
        Lệnh cơ bản, ví dụ:
        ['ros2', 'launch', 'mec_mobile_navigation', 'navigation.launch.py']

    extra_args:
        Launch arguments nối vào cuối cmd khi start(), ví dụ:
        ['map:=/home/hiep0247/maps/phong_hop.yaml']

    kill_patterns:
        Tên executable chính xác để cleanup bằng pkill -x.
    """

    def __init__(self, name: str, cmd: list[str],
                 kill_patterns: list[str] | None = None):
        self.name = name
        self.cmd = list(cmd)
        self.kill_patterns = list(kill_patterns or [])
        self.proc: Optional[subprocess.Popen] = None
        self.current_extra_args: list[str] = []

    def is_running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def start(self, extra_args: list[str] | None = None) -> tuple[bool, str]:
        """
        Start launch process.

        Nếu đang chạy cùng extra_args thì không start lại.
        Nếu đang chạy khác extra_args thì trả False để caller tự stop/start.
        """
        extra_args = [str(x) for x in (extra_args or [])]

        if self.is_running():
            if extra_args == self.current_extra_args:
                return True, f"{self.name} đang chạy"
            return False, (
                f"{self.name} đang chạy với tham số khác — "
                "cần stop() trước khi start() lại"
            )

        self.force_cleanup()
        full_cmd = list(self.cmd) + list(extra_args)

        try:
            self.proc = subprocess.Popen(
                full_cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            self.current_extra_args = list(extra_args)
            return True, f"{self.name} đã chạy"
        except Exception as e:
            self.proc = None
            self.current_extra_args = []
            return False, f"Lỗi start {self.name}: {e}"

    def stop(self) -> tuple[bool, str]:
        """Stop process group mà HMI đã tạo, rồi cleanup node còn sót."""
        if self.proc is not None:
            try:
                if self.proc.poll() is None:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
                    try:
                        self.proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                        self.proc.wait(timeout=2)
            except Exception:
                pass
            self.proc = None

        self.current_extra_args = []
        self.force_cleanup()
        return True, f"{self.name} đã dừng"

    def force_cleanup(self):
        """
        Cleanup node cũ còn sót bằng exact process name.

        Không dùng pkill -f vì -f match toàn command line và dễ kill nhầm
        Gazebo/gz_sim/ros_gz_sim khi simulator đang chạy song song.
        """
        for pat in self.kill_patterns:
            try:
                subprocess.run(
                    ["pkill", "-x", str(pat)],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=2,
                )
            except Exception:
                pass
