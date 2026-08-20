"""
ESP32-CAM Timelapse -> MP4

Works on Windows and macOS.

Install once:
    python -m pip install pillow imageio imageio-ffmpeg numpy

Run:
    python esp32_timelapse_to_mp4.py
"""

from pathlib import Path
import re
import sys
import tkinter as tk
from tkinter import filedialog, messagebox

import imageio.v2 as imageio
import numpy as np
from PIL import Image


# =============================================================================
# USER SETTINGS
# =============================================================================

# Frames per second in the final MP4.
FPS = 10

# Output filename. The video is saved in the selected image folder.
OUTPUT_FILENAME = "timelapse.mp4"

# JPEG extensions to include.
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".JPG", ".JPEG"}

# Quality for H.264 encoding.
# Lower = higher quality / larger file.
VIDEO_QUALITY = 8

# Codec pixel format. yuv420p gives broad compatibility with players/editors.
VIDEO_PIXEL_FORMAT = "yuv420p"

# Resize images that do not match the first image.
RESIZE_MISMATCHED_IMAGES = True

# Print progress every N frames.
PROGRESS_EVERY = 50

# Open the folder containing the finished MP4 when done.
OPEN_OUTPUT_FOLDER = True


# =============================================================================
# FOLDER SELECTION
# =============================================================================

def choose_folder() -> Path | None:
    root = tk.Tk()
    root.withdraw()
    root.update()

    selected = filedialog.askdirectory(
        title="Select the folder containing the ESP32-CAM images"
    )

    root.destroy()

    return Path(selected) if selected else None


# =============================================================================
# IMAGE DISCOVERY / SORTING
# =============================================================================

def image_sort_key(path: Path):
    """
    Numeric sorting for ESP32-CAM names such as:
        IMG_000001.jpg
        IMG_000002.jpg
        IMG_000010.jpg
    """
    match = re.search(r"(\d+)", path.stem)

    if match:
        return (0, int(match.group(1)), path.name.lower())

    return (1, float("inf"), path.name.lower())


def find_images(folder: Path) -> list[Path]:
    images = [
        p for p in folder.iterdir()
        if p.is_file() and p.suffix in IMAGE_EXTENSIONS
    ]

    images.sort(key=image_sort_key)
    return images


# =============================================================================
# IMAGE LOADING
# =============================================================================

def load_rgb(path: Path) -> np.ndarray:
    with Image.open(path) as image:
        return np.asarray(image.convert("RGB"))


def prepare_frame(path: Path, width: int, height: int) -> np.ndarray:
    with Image.open(path) as image:
        image = image.convert("RGB")

        if image.size != (width, height):
            if not RESIZE_MISMATCHED_IMAGES:
                raise ValueError(
                    f"{path.name} has size {image.size}; "
                    f"expected {(width, height)}."
                )

            image = image.resize(
                (width, height),
                Image.Resampling.LANCZOS,
            )

        return np.asarray(image)


# =============================================================================
# MP4 CREATION
# =============================================================================

def create_video(images: list[Path], output_path: Path) -> None:
    if not images:
        raise ValueError("No JPG/JPEG images found.")

    first = load_rgb(images[0])
    height, width = first.shape[:2]

    # H.264/yuv420p requires even dimensions.
    if width % 2 or height % 2:
        width -= width % 2
        height -= height % 2

        first = (
            Image.fromarray(first)
            .resize((width, height), Image.Resampling.LANCZOS)
        )
        first = np.asarray(first)

    duration = len(images) / FPS

    print()
    print("Video settings")
    print("---------------------------")
    print(f"Images:      {len(images)}")
    print(f"FPS:         {FPS}")
    print(f"Resolution:  {width} x {height}")
    print(f"Duration:    {duration:.2f} s")
    print(f"Output:      {output_path}")
    print()

    writer = imageio.get_writer(
        str(output_path),
        fps=FPS,
        codec="libx264",
        quality=VIDEO_QUALITY,
        pixelformat=VIDEO_PIXEL_FORMAT,
    )

    try:
        for i, image_path in enumerate(images, start=1):
            frame = prepare_frame(image_path, width, height)
            writer.append_data(frame)

            if i == 1 or i == len(images) or i % PROGRESS_EVERY == 0:
                percent = i / len(images) * 100
                print(
                    f"\rEncoding: {i}/{len(images)} "
                    f"({percent:6.2f}%)",
                    end="",
                    flush=True,
                )

        print()
    finally:
        writer.close()


# =============================================================================
# OPEN OUTPUT FOLDER
# =============================================================================

def open_folder(folder: Path) -> None:
    try:
        if sys.platform.startswith("win"):
            import os
            os.startfile(folder)  # type: ignore[attr-defined]

        elif sys.platform == "darwin":
            import subprocess
            subprocess.run(["open", str(folder)], check=False)

        else:
            import subprocess
            subprocess.run(["xdg-open", str(folder)], check=False)

    except Exception as exc:
        print(f"Could not open output folder: {exc}")


# =============================================================================
# MAIN
# =============================================================================

def main() -> None:
    print("=" * 60)
    print("ESP32-CAM TIMELAPSE -> MP4")
    print("=" * 60)
    print()

    folder = choose_folder()

    if folder is None:
        print("No folder selected. Exiting.")
        return

    if not folder.is_dir():
        raise ValueError(f"Not a valid folder: {folder}")

    images = find_images(folder)

    if not images:
        message = "No JPG/JPEG images were found in the selected folder."
        print(message)

        root = tk.Tk()
        root.withdraw()
        messagebox.showerror("No images found", message)
        root.destroy()
        return

    print(f"Selected folder: {folder}")
    print(f"Found {len(images)} images.")

    output_path = folder / OUTPUT_FILENAME

    try:
        create_video(images, output_path)
    except Exception as exc:
        print()
        print("Video creation failed:")
        print(exc)

        root = tk.Tk()
        root.withdraw()
        messagebox.showerror(
            "Video creation failed",
            str(exc),
        )
        root.destroy()
        raise

    print()
    print("=" * 60)
    print("DONE")
    print("=" * 60)
    print(f"MP4: {output_path}")

    if OPEN_OUTPUT_FOLDER:
        open_folder(folder)


if __name__ == "__main__":
    main()
