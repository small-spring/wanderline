import cv2
import os

class VideoRecorder:
    """
    Simple video recorder using OpenCV VideoWriter.
    Usage:
        recorder = VideoRecorder(output_path, frame_size, fps)
        recorder.record(frame)
        recorder.release()
    """
    def __init__(self, output_path: str, frame_size: tuple, fps: int = 5, is_color: bool = True):
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.writer = cv2.VideoWriter(output_path, fourcc, fps, frame_size, is_color)

    def record(self, frame):
        """Record a single frame to the video."""
        # frame should match the initialized frame_size
        self.writer.write(frame)

    def release(self):
        """Finish writing and release the video writer."""
        self.writer.release()
