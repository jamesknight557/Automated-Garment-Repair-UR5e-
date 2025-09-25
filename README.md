# Automated-Garment-Repair-UR5e-
Robotic Felting Repair System: Python GUI for automated detection and repair of holes in felt using computer vision and a UR robot arm. Features camera calibration, adjustable vision parameters, DXF export for large holes, selectable repair shapes (circle, square, cross), and live feedback during robotic repair.

## Features

- **Camera Calibration:** Calibrate the workspace with a visual overlay and set pixel-to-mm scaling.
- **Hole Detection:** Detect holes and damage in felt using adjustable kernel size and iteration sliders.
- **Large Hole Handling:** Automatically saves DXF outlines for large holes and generates oversized patch contours for manual repair.
- **Toolpath Generation:** Choose between circular, square, or cross repair patterns for each detected hole.
- **Robot Integration:** Sends toolpaths to a UR robot arm for automated felting repair, with live visual feedback.
- **User-Friendly GUI:** Built with PyQt5, featuring a dark theme, clear controls, and real-time status updates.

## Robot Control

Robot control is provided via the [Rope Robotics](https://github.com/rope-robotics/URBasic) repository, using the `URBasic` Python library for communication with Universal Robots arms.

## Usage

1. **Calibrate Camera:** Align the workspace and confirm calibration.
2. **Detect Holes:** Adjust vision parameters with sliders and detect holes in the felt.
3. **Generate Tool Path:** Select repair shape and generate toolpaths for each hole.
4. **Repair Hole:** Start the robotic repair process and monitor progress visually.

## Requirements

- Python 3.x
- OpenCV (`cv2`)
- PyQt5
- ezdxf
- [URBasic](https://github.com/rope-robotics/URBasic) (for UR robot communication)

## Hardware

- UR robot arm (tested with UR series)
- Camera (positioned ~20cm above felt)
- Felt material

## License

MIT License

---

**For more details, see the code and comments in `ScanNewCal.py`. Contributions and
