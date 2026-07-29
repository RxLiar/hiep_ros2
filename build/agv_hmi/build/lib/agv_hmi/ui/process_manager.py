"""
process_manager.py — Managed ROS2 launch helper v4.2.7

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

Fix v4.2.7:
  - pkill -x chỉ match đúng exact process name (map_server, amcl,...),
    nhưng tiến trình launch cha (ros2 launch ... navigation.launch.py)
    có tên process thực tế là "ros2" hoặc "python3" — pkill -x KHÔNG bắt
    được, nên launch cũ từ lần crash trước có thể vẫn còn sống, gây
    xung đột port/node khi start() lần mới.
    -> Thêm bước dọn dẹp bổ sung bằng `pgrep -af <launch signature>`,
       chỉ kill đúng process có command line CHỨA NGUYÊN VẸN cụm lệnh
       launch (package + file), KHÔNG dùng pkill -f tự do (dễ kill nhầm
       Gazebo/ros_gz_sim đang chạy song song).
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
        # [NEW v4.2.7] Cụm command line đầy đủ (không gồm extra_args vì
        # extra_args thay đổi theo map) để match chính xác tiến trình
        # "ros2 launch <package> <file>" cũ còn sót lại.
        self._launch_signature = " ".join(self.cmd)

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
        Cleanup node cũ còn sót bằng exact process name (pkill -x).

        Không dùng pkill -f cho toàn bộ vì -f match toàn command line và
        dễ kill nhầm Gazebo/gz_sim/ros_gz_sim khi simulator đang chạy
        song song.

        [NEW v4.2.7] Bổ sung: dọn thêm chính tiến trình `ros2 launch`
        cha cũ (tên process là "ros2"/"python3", pkill -x không bắt
        được) bằng cách kiểm tra command line CHỨA ĐÚNG cụm launch
        signature (package + file), rồi mới kill theo process group.
        Không đụng tới process con hiện tại (self.proc) đang được quản
        lý bình thường.
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

        self._cleanup_stale_launch_process()

    def _cleanup_stale_launch_process(self):
        """Dọn tiến trình `ros2 launch <pkg> <file>` cũ bị sót lại."""
        try:
            out = subprocess.run(
                ["pgrep", "-af", self._launch_signature],
                capture_output=True, text=True, timeout=2,
            )
        except Exception:
            return

        current_pid = self.proc.pid if self.proc is not None else None

        for line in out.stdout.strip().splitlines():
            line = line.strip()
            if not line:
                continue
            parts = line.split(" ", 1)
            pid_str = parts[0]

            # Không kill process con đang được quản lý bình thường.
            if current_pid is not None and pid_str == str(current_pid):
                continue

            try:
                pid = int(pid_str)
            except ValueError:
                continue

            try:
                pgid = os.getpgid(pid)
                os.killpg(pgid, signal.SIGTERM)
            except Exception:
                # Fallback: kill trực tiếp theo pid nếu không lấy được pgid
                try:
                    subprocess.run(
                        ["kill", "-9", pid_str],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        timeout=1,
                    )
                except Exception:
                    pass
